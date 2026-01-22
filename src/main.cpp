#include <Arduino.h>
#include <WiFi.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>
#include <WiFiUdp.h>

// --- CONFIGURATION ---
const char* wifi_ssid = "streaming"; 
const char* wifi_pass = "12345678";
const int udpPort = 12345;
const int BOOT_PIN = 0; // Standard Boot button on ESP32


#define MAX_JPEG_SIZE 20000 
#define BUFFER_COUNT 2

// --- SLEEP SETTINGS ---
// 1. "After boot, count for set amount of second" (e.g., 10s)
const unsigned long BOOT_CHECK_TIMEOUT = 10000; 

// --- OBJECTS ---
TFT_eSPI tft = TFT_eSPI();
WiFiUDP udp;

// --- PING-PONG BUFFERS ---
TFT_eSprite* spr[2]; 
uint8_t sprIdx = 0;          

// --- NETWORK VARIABLES ---
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
volatile unsigned long lastPacketTime = 0; // Volatile as modified in ISR/Task

// --- SLEEP CONTROL FLAGS ---
bool sleepCheckPerformed = false;
volatile bool isSleeping = false;

// --- TFT RENDERING CALLBACK ---
bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (x < 0 || y < 0 || (x + w) > 160 || (y + h) > 128) return 0;
  if (y >= tft.height()) return 0;
  spr[sprIdx]->pushImage(x, y, w, h, bitmap);
  return 1;
}

// --- NETWORK TASK (Core 0) ---
void networkTask(void * parameter) {
  udp.begin(udpPort);
  while (true) {
    // If we are sleeping, pause the task to allow WiFi to be off
    if (isSleeping) {
        vTaskDelay(100);
        continue;
    }

    int packetSize = udp.parsePacket();
    
    // Keep-alive
    if (millis() - lastPacketTime > 2000) {
      udp.beginPacket(IPAddress(255,255,255,255), udpPort);
      udp.print("ESP32_READY");
      udp.endPacket();
      delay(1000); 
    }

    if (packetSize > 3) {
      stat_packetsReceived++;
      lastPacketTime = millis();
      
      udp.read(rxBuffer, 1470); 
      
      if (rxBuffer[0] != 0xAA || rxBuffer[1] != 0xBB) {
        taskYIELD();
        continue; 
      }

      uint8_t chunkID = rxBuffer[2]; 
      uint8_t* dataPtr = &rxBuffer[3];
      int dataLen = packetSize - 3;

      if (chunkID == 0) {
        currentFrameSize = 0;
        expectedChunk = 0;
        isFrameCorrupt = false;
      }

      if (!isFrameCorrupt) {
        if (chunkID != expectedChunk) {
          isFrameCorrupt = true; 
          stat_framesDropped++; 
        } else {
          int targetIdx = (writeIdx + 1) % BUFFER_COUNT;
          
          if (currentFrameSize + dataLen < MAX_JPEG_SIZE) {
            memcpy(&jpegBuffers[targetIdx][currentFrameSize], dataPtr, dataLen);
            currentFrameSize += dataLen;
            expectedChunk++;
            
            if (dataLen < 1000) { 
               frameSizes[targetIdx] = currentFrameSize;
               writeIdx = targetIdx;
               frameReady = true;
               stat_framesReceived++;
            }
          } else {
             isFrameCorrupt = true; 
             stat_oversizedFrames++; 
          }
        }
      }
      taskYIELD();
    } else {
      vTaskDelay(1);
    }
  }
}

// --- LIGHT SLEEP FUNCTION ---
void enterLightSleep() {
  Serial.println(">> ENTERING SLEEP MODE...");
  isSleeping = true; // Pause Network Task

  // 1. Show Status on Screen
  tft.fillScreen(TFT_BROWN);

  // 2. Disable WiFi to save power
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);

  // 3. Setup Wakeup Source (BOOT Button = GPIO 0, Active Low)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOOT_PIN, 0); 

  // 4. Enter Light Sleep
  // The CPU pauses here. RAM is retained. 
  // It resumes from the next line when the button is pressed.
  esp_light_sleep_start();

  // --- WAKE UP SEQUENCE ---
  Serial.println(">> WAKING UP...");

  // 5. Reconnect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  WiFi.setSleep(false);
  tft.fillScreen(TFT_CYAN);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  
  // 6. Restart UDP
  udp.begin(udpPort);
  lastPacketTime = millis(); // Reset timer to prevent immediate re-sleep loop
  isSleeping = false;        // Resume Network Task
  tft.fillScreen(TFT_GREEN);
}



void setup() {

  Serial.begin(115200);
  pinMode(BOOT_PIN, INPUT_PULLUP);

  for (int i=0; i<BUFFER_COUNT; i++) {
    jpegBuffers[i] = (uint8_t*)malloc(MAX_JPEG_SIZE);
  }

  tft.init();
  tft.initDMA(); 
  tft.setRotation(1);
  tft.fillScreen(TFT_RED);
  tft.setSwapBytes(true); 

  spr[0] = new TFT_eSprite(&tft);
  spr[1] = new TFT_eSprite(&tft);
  spr[0]->createSprite(tft.width(), tft.height());
  spr[0]->setSwapBytes(true); 
  spr[1]->createSprite(tft.width(), tft.height());
  spr[1]->setSwapBytes(true); 

  TJpgDec.setJpgScale(1);
  TJpgDec.setSwapBytes(true); 
  TJpgDec.setCallback(tft_output);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); 
  WiFi.begin(wifi_ssid, wifi_pass);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) { 
    delay(300);
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected! IP: ");
    Serial.println(WiFi.localIP());
    tft.fillScreen(TFT_GREEN);
  }

  xTaskCreatePinnedToCore(networkTask, "WiFiTask", 10000, NULL, 2, NULL, 0);
  stat_lastReportTime = millis();
  
  // Important: Initialize lastPacketTime to millis() so the "half time" logic 
  // works correctly relative to boot.
  lastPacketTime = millis(); 

  tft.startWrite(); 
}

void loop() {
  // --- ONE-TIME SLEEP CHECK ---
  // Runs once after BOOT_CHECK_TIMEOUT (e.g., 10 seconds)
  if (!sleepCheckPerformed && millis() > BOOT_CHECK_TIMEOUT) {
      sleepCheckPerformed = true; 

      bool wifiNotConnected = (WiFi.status() != WL_CONNECTED);
      
      // "Record time of receiving packet is less than a half of set time"
      // If Timeout is 10s, Half is 5s. 
      // If lastPacketTime < 5000 (meaning we haven't heard anything in the last 5 seconds)
      bool dataIsStale = (lastPacketTime < (BOOT_CHECK_TIMEOUT / 2));

      if (wifiNotConnected || dataIsStale) {
          enterLightSleep();
      }
  }

  if (!isSleeping && frameReady) {
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

    TJpgDec.drawJpg(0, 0, jpegBuffers[writeIdx], frameSizes[writeIdx]);
    
    tft.dmaWait(); // Standard wait

    tft.pushImageDMA(0, 0, tft.width(), tft.height(), (uint16_t*)spr[sprIdx]->getPointer());
    sprIdx = !sprIdx; 
    stat_decodeTimeTotal += (millis() - startTime);
    frameReady = false; 
  }

  // --- REPORTING (FULL STATS RESTORED) ---
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