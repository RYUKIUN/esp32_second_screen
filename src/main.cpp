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
const int BOOT_PIN = 0; // Standard Boot button on ESP32

#define MAX_JPEG_SIZE 20000 
#define BUFFER_COUNT 2

// --- SLEEP SETTINGS ---
const unsigned long BOOT_CHECK_TIMEOUT = 20000; 

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
volatile unsigned long lastPacketTime = 0; 

// --- SLEEP CONTROL FLAGS ---
bool sleepCheckPerformed = false;
volatile bool isSleeping = false;

// --- APP SWITCHER FUNCTION ---
void switchToOtherApp() {
    const esp_partition_t* running = esp_ota_get_running_partition();
    const esp_partition_t* next = NULL;

    // Toggle between OTA_0 and OTA_1
    if (running->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) {
        next = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL);
    } else {
        next = esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);
    }

    if (next != NULL) {
        Serial.println(">> SWITCHING APP PARTITION...");
        esp_ota_set_boot_partition(next);
        delay(100);
        esp_restart();
    }
}

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
    if (isSleeping) {
        vTaskDelay(100);
        continue;
    }

    int packetSize = udp.parsePacket();
    
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
  isSleeping = true; 

  tft.fillScreen(TFT_BROWN);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BOOT_PIN, 0); 
  esp_light_sleep_start();

  // --- WAKE UP SEQUENCE ---
  Serial.println(">> WAKING UP...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_pass);
  WiFi.setSleep(false);
  tft.fillScreen(TFT_CYAN);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  
  udp.begin(udpPort);
  lastPacketTime = millis(); 
  isSleeping = false;        
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
  lastPacketTime = millis(); 

  tft.startWrite(); 
}

void loop() {
  // --- BUTTON CHECK (APP SWITCHER) ---
  static unsigned long btnPressStart = 0;
  static bool btnHeld = false;
  
  // Check button (Active LOW)
  if (digitalRead(BOOT_PIN) == LOW) {
      if (!btnHeld) {
          btnPressStart = millis();
          btnHeld = true;
      }
      
      unsigned long duration = millis() - btnPressStart;
      
      // Visual Indicator while holding > 1s
      if (duration > 1000 && duration < 4000) {
          tft.setTextColor(TFT_RED, TFT_BLACK);
          tft.setCursor(5, 5);
          tft.print("HOLD TO SWITCH: ");
          tft.print((4000 - (long)duration)/1000); 
      }

      // Trigger Switch after 4 seconds
      if (duration > 4000) {
          tft.fillScreen(TFT_RED);
          tft.setCursor(10, 60);
          tft.setTextColor(TFT_WHITE);
          tft.print("SWITCHING APP...");
          delay(500); 
          switchToOtherApp();
      }
  } else {
      btnHeld = false; // Reset
  }

  // --- ONE-TIME SLEEP CHECK ---
  if (!sleepCheckPerformed && millis() > BOOT_CHECK_TIMEOUT) {
      sleepCheckPerformed = true; 
      bool wifiNotConnected = (WiFi.status() != WL_CONNECTED);
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
    
    tft.dmaWait(); 

    tft.pushImageDMA(0, 0, tft.width(), tft.height(), (uint16_t*)spr[sprIdx]->getPointer());
    sprIdx = !sprIdx; 
    stat_decodeTimeTotal += (millis() - startTime);
    frameReady = false; 
  }

  // --- REPORTING (EXACT ORIGINAL LOGIC) ---
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