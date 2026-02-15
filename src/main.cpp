#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // Added for Semaphores

// --- CONFIGURATION ---
const char* wifi_ssid = "streaming";
const char* wifi_pass = "12345678";
const int udpPort = 12345;

#define MAX_JPEG_SIZE 20000
#define BUFFER_COUNT 2
#define SCREEN_W 160
#define SCREEN_H 128

// --- OBJECTS ---
TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;
TFT_eSprite* spr[2];
uint8_t sprIdx = 0;
SemaphoreHandle_t frameSemaphore; // THE WAKE-UP SIGNAL

// --- NETWORK & STATS VARIABLES ---
// (No changes to these variables)
uint32_t stat_packetsReceived = 0;
uint32_t stat_framesReceived = 0;
uint32_t stat_framesDropped = 0;
uint32_t stat_oversizedFrames = 0;
uint32_t stat_decodeTimeTotal = 0;
uint32_t stat_lastReportTime = 0;
uint32_t stat_totalJpegBytes = 0;
uint32_t stat_prevFrameTime = 0;
float stat_jitter = 0.0;

uint8_t* jpegBuffers[BUFFER_COUNT];
volatile int frameSizes[BUFFER_COUNT];
volatile int writeIdx = 0;
volatile bool frameReady = false;

uint8_t rxBuffer[1470];
uint32_t currentFrameSize = 0;
uint8_t expectedChunk = 0;
bool isFrameCorrupt = false;
volatile unsigned long lastPacketTime = 0;
uint8_t animFrame = 0;
unsigned long lastAnimUpdate = 0;

bool debugEnabled = false;
IPAddress remoteIP;
uint16_t remotePort = 0;
char debugBuffer[512]; 

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

// ==============================
// IDLE COUNTER (No changes here)
// ==============================
volatile uint32_t idleCounterCore0 = 0;
volatile uint32_t idleCounterCore1 = 0;
uint32_t lastIdleCore0 = 0;
uint32_t lastIdleCore1 = 0;
uint32_t idleMaxCore0 = 0;
uint32_t idleMaxCore1 = 0;

void idleTaskCore0(void *param) { while (true) { idleCounterCore0++; vTaskDelay(1); } }
void idleTaskCore1(void *param) { while (true) { idleCounterCore1++; vTaskDelay(1); } }

float getCore0Usage() {
    uint32_t now = idleCounterCore0;
    uint32_t delta = now - lastIdleCore0;
    lastIdleCore0 = now;
    if (idleMaxCore0 == 0 && delta > 0) idleMaxCore0 = delta;
    if (delta > idleMaxCore0) idleMaxCore0 = delta;
    if (idleMaxCore0 == 0) return 0.0;
    float idlePct = ((float)delta / (float)idleMaxCore0) * 100.0;
    return 100.0 - (idlePct > 100.0 ? 100.0 : idlePct);
}

float getCore1Usage() {
    uint32_t now = idleCounterCore1;
    uint32_t delta = now - lastIdleCore1;
    lastIdleCore1 = now;
    if (idleMaxCore1 == 0 && delta > 0) idleMaxCore1 = delta;
    if (delta > idleMaxCore1) idleMaxCore1 = delta;
    if (idleMaxCore1 == 0) return 0.0;
    float idlePct = ((float)delta / (float)idleMaxCore1) * 100.0;
    return 100.0 - (idlePct > 100.0 ? 100.0 : idlePct);
}

// --- GRAPHICS FUNCTIONS ---
// (No changes to drawScannerEffect or tft_output)
void drawScannerEffect(String status, bool wifiConnected) {
    TFT_eSprite* uiSpr = spr[0];
    uiSpr->fillScreen(TFT_BLACK);
    uint16_t headerColor = wifiConnected ? 0x03E0 : 0x001F;
    String headerText = wifiConnected ? "NETWORK LOADED" : "SYSTEM LOADING";
    uiSpr->fillRect(0, 0, SCREEN_W, 16, headerColor);
    uiSpr->setTextColor(TFT_WHITE);
    uiSpr->setTextDatum(MC_DATUM);
    uiSpr->drawString(headerText, SCREEN_W / 2, 8);
    uiSpr->drawFastHLine(0, 17, SCREEN_W, TFT_DARKGREY);
    for (int i = 0; i < SCREEN_W; i += 20) uiSpr->drawFastVLine(i, 20, SCREEN_H, 0x0841);
    for (int i = 20; i < SCREEN_H; i += 20) uiSpr->drawFastHLine(0, i, SCREEN_W, 0x0841);
    uiSpr->drawRoundRect(15, 50, SCREEN_W - 30, 40, 4, TFT_CYAN);
    uiSpr->setTextColor(TFT_CYAN);
    uiSpr->setTextDatum(MC_DATUM);
    uiSpr->drawString(status, SCREEN_W / 2, 70);
    int centerX = SCREEN_W / 2;
    int animY = 105;
    if (!wifiConnected) {
        int bars = (animFrame % 4) + 1;
        for (int i = 0; i < 4; i++) {
            uint16_t barCol = (i < bars) ? TFT_CYAN : 0x2104;
            uiSpr->fillRect(centerX - 15 + (i * 8), animY - (i * 3), 5, 5 + (i * 3), barCol);
        }
    } else {
        uint16_t animColor = 0x07E0;
        float angle = animFrame * 0.3;
        int size = 9;
        uiSpr->drawTriangle(centerX + cos(angle)*size, animY + sin(angle)*size,
                         centerX + cos(angle+2.09)*size, animY + sin(angle+2.09)*size,
                         centerX + cos(angle+4.18)*size, animY + sin(angle+4.18)*size, animColor);
        uiSpr->drawCircle(centerX, animY, 14, 0x03E0);
    }
    uiSpr->pushSprite(0, 0);
    animFrame++;
}

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (x < 0 || y < 0 || (x + w) > SCREEN_W || (y + h) > SCREEN_H) return 0;
  spr[sprIdx]->pushImage(x, y, w, h, bitmap);
  return 1;
}

// ==============================
// NETWORK TASK (Modified)
// ==============================
void networkTask(void * parameter) {
  udp.begin(udpPort);
  while (true) {
    int packetSize = udp.parsePacket();

    if (millis() - lastPacketTime > 2000) {
      udp.beginPacket(IPAddress(255,255,255,255), udpPort);
      udp.print("ESP32_READY");
      udp.endPacket();
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    if (packetSize > 2) {
      stat_packetsReceived++;
      lastPacketTime = millis();
      udp.read(rxBuffer, 1470);

      if (rxBuffer[0] == 0xAA) {
        if (rxBuffer[1] == 0xBB) {
            remoteIP = udp.remoteIP();
            remotePort = udp.remotePort();

            uint8_t chunkID = rxBuffer[2];
            uint8_t totalChunks = rxBuffer[3];
            int dataLen = packetSize - 4;

            if (chunkID == 0) {
                currentFrameSize = 0;
                expectedChunk = 0;
                isFrameCorrupt = false;
            }

            if (!isFrameCorrupt) {
                if (chunkID != expectedChunk) {
                    isFrameCorrupt = true;
                    stat_framesDropped++;
                }
                else {
                    int targetIdx = (writeIdx + 1) % BUFFER_COUNT;
                    if (currentFrameSize + dataLen < MAX_JPEG_SIZE) {
                        memcpy(&jpegBuffers[targetIdx][currentFrameSize], &rxBuffer[4], dataLen);
                        currentFrameSize += dataLen;
                        expectedChunk++;

                        if (chunkID == totalChunks - 1) {
                            frameSizes[targetIdx] = currentFrameSize;
                            writeIdx = targetIdx;
                            frameReady = true;
                            // WAKE UP CORE 1 IMMEDIATELY
                            xSemaphoreGive(frameSemaphore);
                        }
                    } else {
                        isFrameCorrupt = true;
                        stat_oversizedFrames++;
                    }
                }
            }
        }
        else if (rxBuffer[1] == 0xCC) {
            if (rxBuffer[2] == 0x01) {
                debugEnabled = (rxBuffer[3] == 1);
                remoteIP = udp.remoteIP();
                remotePort = udp.remotePort();
            }
        }
      }
    }
    vTaskDelay(1);
  }
}

bool streamStarted = false;

// ==============================
// SETUP (Modified)
// ==============================
void setup() {
  for (int i = 0; i < BUFFER_COUNT; i++) {
    jpegBuffers[i] = (uint8_t*)malloc(MAX_JPEG_SIZE);
  }

  // Initialize Semaphore before tasks start
  frameSemaphore = xSemaphoreCreateBinary();

  tft.init();
  tft.initDMA();
  tft.setRotation(1);
  tft.setSwapBytes(false);

  spr[0] = new TFT_eSprite(&tft);
  spr[1] = new TFT_eSprite(&tft);
  spr[0]->createSprite(tft.width(), tft.height());
  spr[1]->createSprite(tft.width(), tft.height());

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true);
  TJpgDec.setCallback(tft_output);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(wifi_ssid, wifi_pass);

  xTaskCreatePinnedToCore(idleTaskCore0, "IdleC0", 1024, NULL, 0, NULL, 0);
  xTaskCreatePinnedToCore(idleTaskCore1, "IdleC1", 1024, NULL, 0, NULL, 1);

  while (WiFi.status() != WL_CONNECTED) {
    drawScannerEffect("LINKING WIFI", false);
    delay(150);
  }

  xTaskCreatePinnedToCore(networkTask, "WiFiTask", 10000, NULL, 2, NULL, 0);

  stat_lastReportTime = millis();
  lastPacketTime = millis();
  tft.startWrite();
  streamStarted = false;
}

// ==============================
// MAIN LOOP (Modified)
// ==============================
void loop() {
  // Instead of checking frameReady constantly, we wait for the semaphore.
  // We use a 10ms timeout so the loop still runs periodically for UI/Stats.
  if (xSemaphoreTake(frameSemaphore, 10 / portTICK_PERIOD_MS) == pdTRUE) {
      
      if (!streamStarted) {
        streamStarted = true;
        tft.fillScreen(TFT_BLACK);
      }
      
      uint32_t startTime = millis();

      // Jitter Calc
      if (stat_prevFrameTime > 0) {
        int32_t interFrameTime = startTime - stat_prevFrameTime;
        static uint32_t lastInterval = 0;
        if (lastInterval > 0) {
            int32_t diff = interFrameTime - lastInterval;
            stat_jitter += (abs(diff) - stat_jitter) / 16.0;
        }
        lastInterval = interFrameTime;
      }
      stat_prevFrameTime = startTime;
      stat_totalJpegBytes += frameSizes[writeIdx];

      // Decode and Draw
      tft.dmaWait(); // Wait for last frame to finish pushing
      TJpgDec.drawJpg(0, 0, jpegBuffers[writeIdx], frameSizes[writeIdx]);
      tft.pushImageDMA(0, 0, tft.width(), tft.height(), (uint16_t*)spr[sprIdx]->getPointer());

      sprIdx = !sprIdx;
      stat_decodeTimeTotal += (millis() - startTime);
      stat_framesReceived++;
      frameReady = false; 
  }
  else {
    // If no frame arrived within 10ms, check if we need to show the idle animation
    if (WiFi.status() == WL_CONNECTED && !streamStarted) {
      if (millis() - lastAnimUpdate > 200) {
        drawScannerEffect("AWAITING STREAM", true);
        lastAnimUpdate = millis();
      }
    }
  }

  // --- STATISTICS REPORTING (No changes to this logic) ---
  uint32_t timeDiff = millis() - stat_lastReportTime;
  if (timeDiff > 1000) { 
    if (debugEnabled && remotePort != 0) {
        float cpuTemp = (temprature_sens_read() - 32) / 1.8;
        float fps = (float)stat_framesReceived / (timeDiff / 1000.0);
        float avgDecode = stat_framesReceived > 0 ? (float)stat_decodeTimeTotal / stat_framesReceived : 0;
        float avgKb = stat_framesReceived > 0 ? ((float)stat_totalJpegBytes / stat_framesReceived) / 1024.0 : 0;
        float cpuUtilDecode = ((float)stat_decodeTimeTotal / timeDiff) * 100.0;
        float wifiCoreCpu = getCore0Usage();
        float displayCoreCpu = getCore1Usage();

        snprintf(debugBuffer, sizeof(debugBuffer),
            "%c%cESP32 STATS REPORT|"
            "FPS: %.1f  Temp: %.1fC|"
            "Jitter: %.1f ms|"
            "Decode: %.1f ms | Size: %.1f KB|"
            "Decode CPU: %.1f %%|"
            "WiFi Core: %.1f %%|"
            "Display Core: %.1f %%|"
            "Drops: %u | Oversize: %u",
            0xAB, 0xCD,
            fps, cpuTemp, stat_jitter, avgDecode, avgKb,
            cpuUtilDecode,
            wifiCoreCpu, displayCoreCpu,
            stat_framesDropped, stat_oversizedFrames
        );
        udp.beginPacket(remoteIP, remotePort);
        udp.write((uint8_t*)debugBuffer, strlen(debugBuffer));
        udp.endPacket();
    }
    stat_packetsReceived = 0;
    stat_framesReceived = 0;
    stat_framesDropped = 0;
    stat_oversizedFrames = 0;
    stat_decodeTimeTotal = 0;
    stat_totalJpegBytes = 0;
    stat_lastReportTime = millis();
  }
}