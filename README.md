ESP32 WiFi Screen Cast

**A low-latency, high-FPS wireless monitor for ~$5 USD.**
** You have to either let ESP32 connect to your PC hotspot, or PC connect to ESP32 hotspot **
** using virtual display driver program to create another screen ** ----> https://github.com/VirtualDrivers/Virtual-Display-Driver

Stream your PC screen to an ESP32 + ST7735 display over WiFi using raw UDP packets and JPEG compression. Unlike standard libraries, this project pushes the ESP32 to its absolute limit using Dual Core processing and DMA (Direct Memory Access) to achieve 30+ FPS.


## demo!!!
![VID_20260116_123316 (online-video-cutter com)](https://github.com/user-attachments/assets/8dadc2cb-5197-4fcb-9286-952f3ebd2310)



## Features
* High Framerate: Achieves 30-35 FPS on a standard ESP32.
* Low Latency: Uses UDP for real-time streaming (no TCP overhead).
* Dual Core: Separates WiFi networking (Core 0) from Image Decoding (Core 1).
* Hardware Acceleration: Uses SPI DMA to paint the screen without blocking the CPU.
* Python Sender: Custom desktop script with adjustable compression, scaling, and transmission throttling.

## Hardware Required
* ESP32 Development Board (ESP32-WROOM-32 is recommended).
* ST7735 TFT Display (1.8" , 128x160 resolution).
* Wires (Dupont cables).

### Wiring (Default SPI)
| VCC | 3.3V | 
| GND | GND | 
| CS | GPIO 16 |
| RESET | GPIO 17 | 
| A0 / DC | GPIO 18 |
| SDA / MOSI | GPIO 19 | 
| SCK / CLK | GPIO 21 | 
| LED | 3.3V | 

*(Note: Check your specific display's pinout, as labels may vary.)*

## Installation

### 1. ESP32 Firmware  <<<<------ important setup
1.  Open the `ESP32_Firmware` folder in Arduino IDE or PlatformIO.
3.  Configure ""PlatformIO.ini"" in the TFT_eSPI library folder to match your ST7735 driver and pins.
4.  change the wifi ssid and password to match your pc hotspot. it's at the "configuration" line        
5.  Upload the code to your ESP32.
6.  After run the code. The serial monitor should show "Waiting for Stream..." and the IP address.



### 2. PC Sender (Python) <<<<<------ important setup
1.  Install Python 3.11 (not sure if newer will be work. but current project is using this version)
2.  turn on pc hotspot or connect to ESP32 wifi after turning it on 4 second
3.  if you want to use it as seperate display. Install virtual display driver from  ------ https://github.com/VirtualDrivers/Virtual-Display-Driver --------. run it to create another display for program to capture.
4.  alway set the second screen resolution in window setting to lower than 1920*1080. the program will always looking for diplay with lower resolution than the main display. recommanded 800*600
5.  Install dependencies with this command:
----------------------------------------------------
  pip install opencv-python dxcam numpy psutil
-------------------------------------------------
 
3.  Open `capture.py` and edit the configuration:

-----------------
    UDP_IP = "192.168.1.XXX" # Set this to your ESP32's IP shown on screen
-----------------

  
4.  Run the script

## Performance Tuning Guide
To get the best FPS, you must balance **JPEG Quality** vs **Network Budget**.

| Setting | Recommendation | NOTE |
| :--- | :--- | :--- |
| FPS Target | 30 | Going higher than 30 exceeds the decoder's physical limit (~33ms). |
| JPEG Quality | 80-90% | The "Sweet Spot." 100% quality triples the file size but looks barely different. |
| Mode | switch to left or right | left to turn on anti-aliasiing, right to turn off and get the sharp image. choose what you see is the best. but recommand turn it on for better performance on esp32|


## Troubleshooting --- connect the esp32 to the serial port while it running. you will see a bunch of information. use that to troubleshoot or tell if it hit the limit yet
* Screen stays black? Check your `PlatformIO.ini` wiring configuration or lower spi speed.
* Glitchy gray bars? You have packet loss. playing around with fps and quality to solve this
* "Oversized Drop"? The image is too complex for the memory buffer. Lower the JPEG Quality.





For someone who read it to this point. this project is made possible by Gemini and tweak the performance by ME (but it is really mostly by AI at this point)
And big thanks for virtual display driver owner's. that's help a lot for not making me go crazy for creating virtual display software by scratch.
So, feel free to use it. you can report some issue if it really out of hand.
but it mostly just a setup problem at this point
