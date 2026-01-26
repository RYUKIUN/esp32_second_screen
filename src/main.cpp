#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFiUdp.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>

// --- CONFIGURATION ---
const char* wifi_ssid = "streaming"; 
const char* wifi_pass = "12345678";
const int udpPort = 12345;
const int BOOT_PIN = 0; 

#define MAX_JPEG_SIZE 20000 
#define BUFFER_COUNT 2
#define SCREEN_W 160
#define SCREEN_H 128

// --- OBJECTS ---
TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;
TFT_eSprite* spr[2]; 
uint8_t sprIdx = 0;          

// --- NETWORK & STATS VARIABLES ---
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

// --- SLEEP & ANIMATION CONTROL ---
bool sleepCheckPerformed = false;
volatile bool isSleeping = false;
const unsigned long BOOT_CHECK_TIMEOUT = 20000; 

uint8_t animFrame = 0;
unsigned long lastAnimUpdate = 0;

// --- COOL GRAPHICS FUNCTIONS ---

void drawHeader(String title, uint16_t color) {
    tft.fillRect(0, 0, SCREEN_W, 16, color);
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(title, SCREEN_W / 2, 8);
    tft.drawFastHLine(0, 17, SCREEN_W, TFT_DARKGREY);
}

// Unified function for Loading (WiFi) and Loaded (Waiting for Stream)
void drawScannerEffect(String status, bool wifiConnected) {
    tft.fillScreen(TFT_BLACK);
    
    // Background Grid
    for (int i = 0; i < SCREEN_W; i += 20) tft.drawFastVLine(i, 20, SCREEN_H, 0x0841); 
    for (int i = 20; i < SCREEN_H; i += 20) tft.drawFastHLine(0, i, SCREEN_W, 0x0841);
    
    // Header changes color based on state
    drawHeader(wifiConnected ? "NETWORK LOADED" : "SYSTEM LOADING", wifiConnected ? 0x03E0 : 0x001F);
    
    tft.drawRoundRect(15, 50, SCREEN_W - 30, 40, 4, TFT_CYAN);
    tft.setTextColor(TFT_CYAN);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(status, SCREEN_W / 2, 70);

    int centerX = SCREEN_W / 2;
    int animY = 105;

    if (!wifiConnected) {
        // ANIMATION: Pulsing Signal Bars (WiFi Loading)
        int bars = (animFrame % 4) + 1;
        for (int i = 0; i < 4; i++) {
            uint16_t barCol = (i < bars) ? TFT_CYAN : 0x2104; // Dim gray
            tft.fillRect(centerX - 15 + (i * 8), animY - (i * 3), 5, 5 + (i * 3), barCol);
        }
    } else {
        // ANIMATION: Rotating Tech Triangle (Waiting for Stream)
        float angle = animFrame * 0.3;
        int size = 9;
        int x0 = centerX + cos(angle) * size;
        int y0 = animY + sin(angle) * size;
        int x1 = centerX + cos(angle + 2.09) * size;
        int y1 = animY + sin(angle + 2.09) * size;
        int x2 = centerX + cos(angle + 4.18) * size;
        int y2 = animY + sin(angle + 4.18) * size;
        
        tft.drawTriangle(x0, y0, x1, y1, x2, y2, 0x07FF); // Cyan-White
        tft.drawCircle(centerX, animY, 14, 0x03E0);      // Green ring
    }
    animFrame++;
}

void drawSleepGraphic() {
    tft.fillScreen(TFT_BLACK);
    for (int i = 0; i < 30; i++) tft.drawPixel(random(SCREEN_W), random(SCREEN_H), TFT_WHITE);
    // Crescent Moon
    tft.fillCircle(SCREEN_W / 2, SCREEN_H / 2 - 10, 20, 0xFFE0); 
    tft.fillCircle(SCREEN_W / 2 + 8, SCREEN_H / 2 - 15, 18, TFT_BLACK); 
    
    tft.setTextColor(TFT_LIGHTGREY);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("SYSTEM HIBERNATED", SCREEN_W / 2, SCREEN_H - 30);
}

void drawSwitchingGraphic(int progress) {
    tft.fillScreen(TFT_BLACK);
    drawHeader("PARTITION SWITCH", TFT_RED);
    int centerX = SCREEN_W / 2;
    int centerY = SCREEN_H / 2 + 10;
    
    tft.drawCircle(centerX, centerY, 32, 0x4208); // Dark circle
    float endAngle = (progress / 100.0) * 360.0;
    for (int i = 0; i < endAngle; i += 10) {
        float rad = (i - 90) * 0.0174533;
        tft.fillCircle(centerX + cos(rad) * 32, centerY + sin(rad) * 32, 3, TFT_RED);
    }
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(MC_DATUM);
    tft.drawNumber(progress, centerX, centerY, 2);
    tft.drawString("%", centerX + 18, centerY);
}

// --- APP SWITCHER ---
void switchToOtherApp() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next = (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) 
        ? esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL)
        : esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

    if (next != NULL) {
        esp_ota_set_boot_partition(next);
        delay(100);
        esp_restart();
    }
}

// --- TFT RENDERING CALLBACK ---
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (x < 0 || y < 0 || (x + w) > 160 || (y + h) > 128) return 0;
  spr[sprIdx]->pushImage(x, y, w, h, bitmap);
  return 1;
}

// --- NETWORK TASK (Core 0) ---
void networkTask(void * parameter) {
  udp.begin(udpPort);
  while (true) {
    if (isSleeping) { vTaskDelay(100); continue; }
    int packetSize = udp.parsePacket();
    
    if (millis() - lastPacketTime > 2000) {
      udp.beginPacket(IPAddress(255,255,255,255), udpPort);
      udp.print("ESP32_READY");
      udp.endPacket();
      vTaskDelay(1000 / portTICK_PERIOD_MS); 
    }

    if (packetSize > 3) {
      stat_packetsReceived++;
      lastPacketTime = millis();
      udp.read(rxBuffer, 1470); 
      if (rxBuffer[0] != 0xAA || rxBuffer[1] != 0xBB) { taskYIELD(); continue; }

      uint8_t chunkID = rxBuffer[2]; 
      int dataLen = packetSize - 3;

      if (chunkID == 0) { currentFrameSize = 0; expectedChunk = 0; isFrameCorrupt = false; }

      if (!isFrameCorrupt) {
        if (chunkID != expectedChunk) { isFrameCorrupt = true; stat_framesDropped++; } 
        else {
          int targetIdx = (writeIdx + 1) % BUFFER_COUNT;
          if (currentFrameSize + dataLen < MAX_JPEG_SIZE) {
            memcpy(&jpegBuffers[targetIdx][currentFrameSize], &rxBuffer[3], dataLen);
            currentFrameSize += dataLen;
            expectedChunk++;
            if (dataLen < 1000) { 
               frameSizes[targetIdx] = currentFrameSize;
               writeIdx = targetIdx;
               frameReady = true;
               stat_framesReceived++;
            }
          } else { isFrameCorrupt = true; stat_oversizedFrames++; }
        }
      }
    }
    vTaskDelay(1);
  }
}

// --- SLEEP MODE ---
void enterLightSleep() {
  drawSleepGraphic();
  delay(1500);
  isSleeping = true; 
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOOT_PIN, 0); 
  esp_light_sleep_start();

  // WAKE SEQUENCE
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  while (WiFi.status() != WL_CONNECTED) { 
      drawScannerEffect("RECONNECTING...", false);
      delay(100); 
  }
  lastPacketTime = millis(); 
  isSleeping = false;        
}

void setup() {
  Serial.begin(115200);
  pinMode(BOOT_PIN, INPUT_PULLUP);
  for (int i=0; i<BUFFER_COUNT; i++) jpegBuffers[i] = (uint8_t*)malloc(MAX_JPEG_SIZE);

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
  WiFi.begin(wifi_ssid, wifi_pass);

  // Initial WiFi Loading Animation
  while (WiFi.status() != WL_CONNECTED) { 
    drawScannerEffect("LINKING WIFI", false);
    delay(150); 
  }

  xTaskCreatePinnedToCore(networkTask, "WiFiTask", 10000, NULL, 2, NULL, 0);
  stat_lastReportTime = millis();
  lastPacketTime = millis(); 
  tft.startWrite(); 
}

void loop() {
  // --- BUTTON CHECK (APP SWITCHER) ---
  static unsigned long btnPressStart = 0;
  static bool btnHeld = false;
  
  if (digitalRead(BOOT_PIN) == LOW) {
      if (!btnHeld) { btnPressStart = millis(); btnHeld = true; }
      unsigned long duration = millis() - btnPressStart;
      if (duration > 800) drawSwitchingGraphic(map(min(duration, 4000UL), 800, 4000, 0, 100));
      if (duration > 4000) switchToOtherApp();
  } else { btnHeld = false; }

  // --- SLEEP CHECK ---
  if (!sleepCheckPerformed && millis() > BOOT_CHECK_TIMEOUT) {
      sleepCheckPerformed = true; 
      if (WiFi.status() != WL_CONNECTED || (millis() - lastPacketTime > 10000)) enterLightSleep();
  }

  // --- RENDERING & IDLE ANIMATION ---
  if (!isSleeping) {
      if (frameReady) {
          uint32_t startTime = millis();
          
          // Jitter Calculation
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

          TJpgDec.drawJpg(0, 0, jpegBuffers[writeIdx], frameSizes[writeIdx]);
          tft.dmaWait(); 
          tft.pushImageDMA(0, 0, tft.width(), tft.height(), (uint16_t*)spr[sprIdx]->getPointer());
          sprIdx = !sprIdx; 
          stat_decodeTimeTotal += (millis() - startTime);
          frameReady = false; 
      } 
      else if (millis() - lastPacketTime > 1200) {
          // No stream data? Play the "Waiting/Loaded" animation
          if (millis() - lastAnimUpdate > 50) { 
              drawScannerEffect("WAITING FOR DATA", true);
              lastAnimUpdate = millis();
          }
      }
  }

  // --- STATISTICS REPORTING ---
  uint32_t timeDiff = millis() - stat_lastReportTime;
  if (timeDiff > 2000) {
    float fps = (float)stat_framesReceived / (timeDiff / 1000.0);
    float avgDecode = stat_framesReceived > 0 ? (float)stat_decodeTimeTotal / stat_framesReceived : 0;
    float avgJpegSize = stat_framesReceived > 0 ? ((float)stat_totalJpegBytes / stat_framesReceived) / 1024.0 : 0;
    float cpuUtil = ((float)stat_decodeTimeTotal / timeDiff) * 100.0;

    Serial.println("\n--- STREAM STATS (2s) ---");
    Serial.printf("FPS:                %.2f\n", fps);
    Serial.printf("Avg Decode Time:    %.2f ms\n", avgDecode);
    Serial.printf("Avg JPEG Size:      %.2f KB\n", avgJpegSize);
    Serial.printf("CPU Utilization:    %.2f %%\n", cpuUtil);
    Serial.printf("Jitter (ms):        %.2f\n", stat_jitter);
    Serial.printf("Frames Received:    %u\n", stat_framesReceived);
    Serial.printf("Packet Loss/Drops:  %u\n", stat_framesDropped);
    Serial.printf("Oversized Drops:    %u\n", stat_oversizedFrames);
    Serial.println("-------------------------");

    stat_packetsReceived = 0;
    stat_framesReceived = 0;
    stat_framesDropped = 0;
    stat_oversizedFrames = 0;
    stat_decodeTimeTotal = 0;
    stat_totalJpegBytes = 0;
    stat_lastReportTime = millis();
  }
}