# 📺 ESP32 WiFi Screen Cast (High Performance)

**A low-latency, high-FPS wireless monitor for ~$5 USD.**

Stream your PC screen to an ESP32 + ST7735 display over WiFi using raw UDP packets and JPEG compression. Unlike standard libraries, this project pushes the ESP32 to its absolute limit using **Dual Core processing** and **DMA (Direct Memory Access)** to achieve **30+ FPS**.

> **Note:** This project relies on the **[Virtual Display Driver](https://github.com/VirtualDrivers/Virtual-Display-Driver)** to create a secondary virtual screen for capturing.

![Project Demo]
![VID_20260116_123316 (online-video-cutter com)](https://github.com/user-attachments/assets/57c621a4-8890-4f75-a39a-c1185c0ca01d)




## 🚀 Features
* **High Framerate:** Achieves **30-35 FPS** on a standard ESP32.
* **Low Latency:** Uses UDP for real-time streaming (no TCP overhead).
* **Dual Core Architecture:** Separates WiFi networking (Core 0) from Image Decoding (Core 1).
* **Hardware Acceleration:** Uses SPI DMA to paint the screen without blocking the CPU.
* **Python Sender:** Custom desktop script with adjustable compression, scaling, and transmission throttling.

## 🛠 Hardware Required
* **ESP32 Development Board** (ESP32-WROOM-32 recommended).
* **ST7735 TFT Display** (1.8" or 1.44", 128x160 resolution).
* **Dupont Wires**.

### 🔌 Wiring (Default SPI)
Based on the provided `platformio.ini` configuration:

| ST7735 Pin | ESP32 Pin | Function |
| :--- | :--- | :--- |
| **VCC** | 3.3V | Power |
| **GND** | GND | Ground |
| **CS** | GPIO 16 | Chip Select |
| **RESET** | GPIO 17 | Reset |
| **A0 / DC** | GPIO 18 | Data/Command |
| **SDA / MOSI** | GPIO 19 | SPI Data |
| **SCK / CLK** | GPIO 21 | SPI Clock |
| **LED** | 3.3V | Backlight |

*(Note: Check your specific display's pinout, as labels may vary.)*

---

## 📦 Installation

### Part 1: ESP32 Firmware (PlatformIO)
**Important:** This project uses `build_flags` in `platformio.ini` to auto-configure the screen driver. You do **not** need to manually edit the `TFT_eSPI` library files.

1.  Open the `ESP32_Firmware` folder in **VS Code** (with PlatformIO extension).
2.  Open `platformio.ini` and verify the pin definitions match your wiring.
3.  **WiFi Setup:** Open the main `.cpp` file and look for the **Configuration** section. Update the `SSID` and `PASSWORD` to match your PC's hotspot (or the router you are connecting to).
4.  **Upload** the code to your ESP32.
5.  Open the **Serial Monitor** (Baud Rate: 115200). It should show: `Waiting for Stream...` and display the **ESP32's IP Address**.

### Part 2: PC Sender (Python)
1.  **Python Setup:** Install Python **3.11** (recommended version for compatibility).
2.  **Virtual Display:**
    * Download and install the [Virtual Display Driver](https://github.com/VirtualDrivers/Virtual-Display-Driver).
    * **Resolution Rule:** The program automatically looks for a display with a lower resolution than your main monitor.
    * **Recommendation:** Set the virtual second screen resolution to **800x600** in Windows Settings.
3.  **Dependencies:** Install the required libraries with this command:
    ```bash
    pip install opencv-python dxcam numpy psutil
    ```
   
4.  **Connection:**
    * Ensure your PC Hotspot is ON (or both devices are on the same WiFi).
    * *Tip:* If using a hotspot, turn it on and wait ~4 seconds before connecting the ESP32.
5.  **Configuration:**
    * Open `capture.py`.
    * Edit the `UDP_IP` line to match the IP address shown on your ESP32's screen:
        ```python
        UDP_IP = "192.168.1.XXX"
        ```
       
6.  **Run:** Execute the script (`python capture.py`).

---

## ⚙️ Performance Tuning Guide
To get the best FPS, you must balance **JPEG Quality** vs **Network Budget**.

| Setting | Value | Note |
| :--- | :--- | :--- |
| **FPS Target** | **30** | Going higher than 30 exceeds the ESP32 decoder's physical limit (~33ms decode time). |
| **JPEG Quality** | **80-90%** | The "Sweet Spot." 100% quality triples the file size but barely looks different visually. |
| **Mode** | **Left** vs **Right** | **Left:** Turns **ON** anti-aliasing (Recommended). Slightly blurs edges but significantly improves ESP32 decoding speed.<br>**Right:** Turns **OFF** anti-aliasing. Sharper image but higher CPU load. |

---

## 🐛 Troubleshooting
Connect the ESP32 to the computer and open the Serial Monitor to see real-time logs.

* **Screen stays black?**
    * Check your wiring against the table above.
    * Check `platformio.ini` configuration.
    * Try lowering the `SPI_FREQUENCY` in `platformio.ini` if your wires are long.
* **Glitchy gray bars?**
    * You are experiencing UDP packet loss.
    * Try lowering the **FPS Target** or **JPEG Quality** slider.
    * Ensure the ESP32 is close to the WiFi antenna.
* **"Oversized Drop" Error?**
    * The current image frame is too complex (file size > 20KB) for the ESP32's memory buffer.
    * **Solution:** Lower the **JPEG Quality** slider immediately.

---

## 🤝 Acknowledgments
* **Gemini AI:** For generating the core high-performance code and optimization logic.
* **Me (The Human):** For tweaking, debugging, and testing the physical limits of the hardware.
* **[VirtualDrivers](https://github.com/VirtualDrivers/Virtual-Display-Driver):** Huge thanks for the virtual display driver, which saved this project from requiring custom driver development.

**Disclaimer:** This project pushes the ESP32 to its limit. If you encounter issues, they are likely setup-related. Feel free to report major bugs, but please double-check your wiring and WiFi environment first.
