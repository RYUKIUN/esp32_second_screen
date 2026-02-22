#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFiUdp.h>
#include <esp_wifi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ================= CONFIG =================
const char* wifi_ssid = "streaming";
const char* wifi_pass = "12345678";
const int udpPort = 12345;

#define MAX_JPEG_SIZE 20000
#define BUFFER_COUNT 2
#define SCREEN_W 160
#define SCREEN_H 128

// ================= OBJECTS =================
TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;
TFT_eSprite* spr[2];
uint8_t sprIdx = 0;
SemaphoreHandle_t frameSemaphore;

// ================= STATS (MINIMAL) =================
uint32_t stat_framesReceived = 0;
uint32_t stat_lastReportTime = 0;
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

// V-Sync Emulation Variable
uint32_t lastPushTime = 0;

bool debugEnabled = false;
IPAddress remoteIP;
uint16_t remotePort = 0;
char debugBuffer[256];

#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif

// ================= JPEG CALLBACK =================
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (x < 0 || y < 0 || (x + w) > SCREEN_W || (y + h) > SCREEN_H) return 0;
  spr[sprIdx]->pushImage(x, y, w, h, bitmap); //
  return 1;
}

// ================= NETWORK TASK =================
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
                            xSemaphoreGive(frameSemaphore);
                        }
                    }
                    else {
                        isFrameCorrupt = true;
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

// ================= SETUP =================
void setup() {
  for (int i = 0; i < BUFFER_COUNT; i++) {
    jpegBuffers[i] = (uint8_t*)malloc(MAX_JPEG_SIZE);
  }

  frameSemaphore = xSemaphoreCreateBinary();

  tft.init();
  tft.initDMA(); //
  tft.setRotation(1);
  tft.setSwapBytes(false);

  // --- METHOD 1: Boost Internal Refresh Rate (ST7735) ---
  tft.startWrite();
  tft.writecommand(0xB1); // Frame Rate Control (In normal mode/Full colors)
  tft.writedata(0x01);    // Fastest rate: RTNA set to 0x01
  tft.writedata(0x2C);    // Front porch
  tft.writedata(0x2D);    // Back porch
  tft.endWrite();

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

  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
  }

  xTaskCreatePinnedToCore(networkTask, "WiFiTask", 10000, NULL, 2, NULL, 0);

  stat_lastReportTime = millis();
  lastPacketTime = millis();
  tft.startWrite();
}

// ================= MAIN LOOP =================
void loop() {
  if (xSemaphoreTake(frameSemaphore, 10 / portTICK_PERIOD_MS) == pdTRUE) {

      if (!streamStarted) {
        streamStarted = true;
        tft.fillScreen(TFT_BLACK);
      }

      uint32_t startTime = millis();

      // Jitter calc
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

      // --- METHOD 3: V-Sync Emulation Delay ---
      // Wait until at least 10ms (for 100Hz refresh) has passed since last push
      // to avoid colliding with the scanline
      while ((micros() - lastPushTime) < 10000) {
          ets_delay_us(100); 
      }

      tft.dmaWait(); //
      TJpgDec.drawJpg(0, 0, jpegBuffers[writeIdx], frameSizes[writeIdx]);
      
      lastPushTime = micros();
      tft.pushImageDMA(0, 0, tft.width(), tft.height(),
                       (uint16_t*)spr[sprIdx]->getPointer()); //

      sprIdx = !sprIdx;
      stat_framesReceived++;
  }

  // ================= STAT REPORT =================
  uint32_t timeDiff = millis() - stat_lastReportTime;
  if (timeDiff > 1000) {
    if (debugEnabled && remotePort != 0) {
        float cpuTemp = (temprature_sens_read() - 32) / 1.8;
        float fps = (float)stat_framesReceived / (timeDiff / 1000.0);

        uint32_t totalHeap = ESP.getHeapSize();
        uint32_t freeHeap = ESP.getFreeHeap();
        float ramPercentLeft = ((float)freeHeap / totalHeap) * 100.0;

        snprintf(debugBuffer, sizeof(debugBuffer),
            "%c%cESP32 STATS|"
            "FPS: %.1f|"
            "Temp: %.1fC|"
            "Jitter: %.1f ms|"
            "RAM Left: %.1f%%",
            0xAB, 0xCD, fps, cpuTemp, stat_jitter, ramPercentLeft
        );

        udp.beginPacket(remoteIP, remotePort);
        udp.write((uint8_t*)debugBuffer, strlen(debugBuffer));
        udp.endPacket();
    }
    stat_framesReceived = 0;
    stat_lastReportTime = millis();
  }
}