#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFiUdp.h>

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

uint8_t animFrame = 0;
unsigned long lastAnimUpdate = 0;

// --- GRAPHICS FUNCTIONS ---

// Modified to draw to a sprite to prevent flickering
void drawScannerEffect(String status, bool wifiConnected) {
    // Use spr[0] as our back-buffer for UI to prevent flicker
    TFT_eSprite* uiSpr = spr[0];
    
    uiSpr->fillScreen(TFT_BLACK);

    // 1. Draw Header to Sprite
    uint16_t headerColor = wifiConnected ? 0x03E0 : 0x001F;
    String headerText = wifiConnected ? "NETWORK LOADED" : "SYSTEM LOADING";
    
    uiSpr->fillRect(0, 0, SCREEN_W, 16, headerColor);
    uiSpr->setTextColor(TFT_WHITE);
    uiSpr->setTextDatum(MC_DATUM);
    uiSpr->drawString(headerText, SCREEN_W / 2, 8);
    uiSpr->drawFastHLine(0, 17, SCREEN_W, TFT_DARKGREY);

    // 2. Draw Grid
    for (int i = 0; i < SCREEN_W; i += 20) uiSpr->drawFastVLine(i, 20, SCREEN_H, 0x0841); 
    for (int i = 20; i < SCREEN_H; i += 20) uiSpr->drawFastHLine(0, i, SCREEN_W, 0x0841);
    
    // 3. Draw Status Box
    uiSpr->drawRoundRect(15, 50, SCREEN_W - 30, 40, 4, TFT_CYAN);
    uiSpr->setTextColor(TFT_CYAN);
    uiSpr->setTextDatum(MC_DATUM);
    uiSpr->drawString(status, SCREEN_W / 2, 70);

    int centerX = SCREEN_W / 2;
    int animY = 105;

    // 4. Draw Animation
    if (!wifiConnected) {
        // Connecting Animation (Bars)
        int bars = (animFrame % 4) + 1;
        for (int i = 0; i < 4; i++) {
            uint16_t barCol = (i < bars) ? TFT_CYAN : 0x2104;
            uiSpr->fillRect(centerX - 15 + (i * 8), animY - (i * 3), 5, 5 + (i * 3), barCol);
        }
    } else {
        // Connected / Stream Lost Animation (Rotating Triangles)
        // CHANGED: Color is now GREEN (0x07E0) instead of Cyan (0x07FF)
        // Logic: if we are here, wifi is connected but we are waiting for data
        uint16_t animColor = 0x07E0; // TFT_GREEN
        
        float angle = animFrame * 0.3;
        int size = 9;
        uiSpr->drawTriangle(centerX + cos(angle)*size, animY + sin(angle)*size, 
                         centerX + cos(angle+2.09)*size, animY + sin(angle+2.09)*size, 
                         centerX + cos(angle+4.18)*size, animY + sin(angle+4.18)*size, animColor);
        uiSpr->drawCircle(centerX, animY, 14, 0x03E0);
    }
    
    // 5. Push the completely drawn frame to screen
    uiSpr->pushSprite(0, 0);
    
    animFrame++;
}

// --- TFT RENDERING CALLBACK ---
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (x < 0 || y < 0 || (x + w) > 160 || (y + h) > 128) return 0;
  spr[sprIdx]->pushImage(x, y, w, h, bitmap);
  return 1;
}

// --- NETWORK TASK (CORE 0) ---
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
            }
          } else { isFrameCorrupt = true; stat_oversizedFrames++; }
        }
      }
    }
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200);
  
  // 1. HARDWARE & MEMORY SETUP
  for (int i=0; i<BUFFER_COUNT; i++) jpegBuffers[i] = (uint8_t*)malloc(MAX_JPEG_SIZE);

  tft.init();
  tft.initDMA(); 
  tft.setRotation(1);
  tft.setSwapBytes(false); 

  // Allocate sprites
  spr[0] = new TFT_eSprite(&tft);
  spr[1] = new TFT_eSprite(&tft);
  spr[0]->createSprite(tft.width(), tft.height());
  spr[1]->createSprite(tft.width(), tft.height());

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true); 
  TJpgDec.setCallback(tft_output);

  // 2. WIFI SETUP
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);

  while (WiFi.status() != WL_CONNECTED) { 
    drawScannerEffect("LINKING WIFI", false);
    delay(150); 
  }

  // 3. START BACKGROUND TASKS
  xTaskCreatePinnedToCore(networkTask, "WiFiTask", 10000, NULL, 2, NULL, 0);

  stat_lastReportTime = millis();
  lastPacketTime = millis(); 
  tft.startWrite(); 
}

void loop() {
  // --- RENDERING & IDLE ANIMATION ---
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
      stat_framesReceived++;
      frameReady = false; 
  } 
  else if (millis() - lastPacketTime > 1200) {
      if (millis() - lastAnimUpdate > 50) { 
          // Shows the GREEN triangle animation
          drawScannerEffect("WAITING FOR DATA", true);
          lastAnimUpdate = millis();
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

    stat_packetsReceived = 0; stat_framesReceived = 0; stat_framesDropped = 0;
    stat_oversizedFrames = 0; stat_decodeTimeTotal = 0; stat_totalJpegBytes = 0;
    stat_lastReportTime = millis();
  }
}