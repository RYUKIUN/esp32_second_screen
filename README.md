# 📺 ESP32 WiFi Screen Cast (High Performance)

**A low-latency, high-FPS wireless monitor for ~$5 USD.**

Stream your PC screen to an ESP32 + ST7735 display over WiFi using raw UDP packets and JPEG compression. [cite_start]Unlike standard libraries, this project pushes the ESP32 to its absolute limit using **Dual Core processing** and **DMA (Direct Memory Access)** to achieve **30+ FPS**[cite: 2].

> [cite_start]**Note:** This project relies on the **[Virtual Display Driver](https://github.com/VirtualDrivers/Virtual-Display-Driver)** to create a secondary virtual screen for capturing[cite: 1, 11].

![Project Demo](demo.gif)

## 🚀 Features
* [cite_start]**High Framerate:** Achieves **30-35 FPS** on a standard ESP32[cite: 3].
* [cite_start]**Low Latency:** Uses UDP for real-time streaming (no TCP overhead)[cite: 4].
* [cite_start]**Dual Core Architecture:** Separates WiFi networking (Core 0) from Image Decoding (Core 1)[cite: 5].
* [cite_start]**Hardware Acceleration:** Uses SPI DMA to paint the screen without blocking the CPU[cite: 6].
* [cite_start]**Python Sender:** Custom desktop script with adjustable compression, scaling, and transmission throttling[cite: 7].

## 🛠 Hardware Required
* [cite_start]**ESP32 Development Board** (ESP32-WROOM-32 recommended)[cite: 8].
* [cite_start]**ST7735 TFT Display** (1.8" or 1.44", 128x160 resolution)[cite: 8].
* [cite_start]**Dupont Wires**[cite: 8].

### 🔌 Wiring (Default SPI)
[cite_start]Based on the provided `platformio.ini` configuration[cite: 8]:

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
**Important:** This project uses `build_flags` in `platformio.ini` to auto-configure the screen driver. [cite_start]You do **not** need to manually edit the `TFT_eSPI` library files[cite: 9].

1.  Open the `ESP32_Firmware` folder in **VS Code** (with PlatformIO extension).
2.  Open `platformio.ini` and verify the pin definitions match your wiring.
3.  **WiFi Setup:** Open the main `.cpp` file and look for the **Configuration** section. Update the `SSID` and `PASSWORD` to match your PC's hotspot (or the router you are connecting to).
4.  **Upload** the code to your ESP32.
5.  Open the **Serial Monitor** (Baud Rate: 115200). It should show: `Waiting for Stream...` and display the **ESP32's IP Address**.

### Part 2: PC Sender (Python)
1.  [cite_start]**Python Setup:** Install Python **3.11** (recommended version for compatibility)[cite: 9].
2.  **Virtual Display:**
    * [cite_start]Download and install the [Virtual Display Driver](https://github.com/VirtualDrivers/Virtual-Display-Driver)[cite: 1, 11].
    * [cite_start]**Resolution Rule:** The program automatically looks for a display with a lower resolution than your main monitor[cite: 13].
    * [cite_start]**Recommendation:** Set the virtual second screen resolution to **800x600** in Windows Settings[cite: 12, 14].
3.  **Dependencies:** Install the required libraries with this command:
    ```bash
    pip install opencv-python dxcam numpy psutil
    ```
    [cite_start][cite: 14]
4.  **Connection:**
    * Ensure your PC Hotspot is ON (or both devices are on the same WiFi).
    * [cite_start]*Tip:* If using a hotspot, turn it on and wait ~4 seconds before connecting the ESP32[cite: 10].
5.  **Configuration:**
    * Open `capture.py`.
    * Edit the `UDP_IP` line to match the IP address shown on your ESP32's screen:
        ```python
        UDP_IP = "192.168.1.XXX"
        ```
        [cite_start][cite: 14]
6.  **Run:** Execute the script (`python capture.py`).

---

## ⚙️ Performance Tuning Guide
To get the best FPS, you must balance **JPEG Quality** vs **Network Budget**.

| Setting | Value | Note |
| :--- | :--- | :--- |
| **FPS Target** | **30** | [cite_start]Going higher than 30 exceeds the ESP32 decoder's physical limit (~33ms decode time)[cite: 15, 16]. |
| **JPEG Quality** | **80-90%** | The "Sweet Spot." [cite_start]100% quality triples the file size but barely looks different visually[cite: 16, 17]. |
| **Mode** | **Left** vs **Right** | **Left:** Turns **ON** anti-aliasing (Recommended). Slightly blurs edges but significantly improves ESP32 decoding speed.<br>**Right:** Turns **OFF** anti-aliasing. [cite_start]Sharper image but higher CPU load[cite: 18, 19]. |

---

## 🐛 Troubleshooting
[cite_start]Connect the ESP32 to the computer and open the Serial Monitor to see real-time logs[cite: 20].

* **Screen stays black?**
    * Check your wiring against the table above.
    * Check `platformio.ini` configuration.
    * [cite_start]Try lowering the `SPI_FREQUENCY` in `platformio.ini` if your wires are long[cite: 22].
* **Glitchy gray bars?**
    * You are experiencing UDP packet loss.
    * Try lowering the **FPS Target** or **JPEG Quality** slider.
    * [cite_start]Ensure the ESP32 is close to the WiFi antenna[cite: 22].
* **"Oversized Drop" Error?**
    * The current image frame is too complex (file size > 20KB) for the ESP32's memory buffer.
    * [cite_start]**Solution:** Lower the **JPEG Quality** slider immediately[cite: 23, 24].

---

## 🤝 Acknowledgments
* [cite_start]**Gemini AI:** For generating the core high-performance code and optimization logic[cite: 25].
* [cite_start]**Me (The Human):** For tweaking, debugging, and testing the physical limits of the hardware[cite: 25].
* [cite_start]**[VirtualDrivers](https://github.com/VirtualDrivers/Virtual-Display-Driver):** Huge thanks for the virtual display driver, which saved this project from requiring custom driver development[cite: 26].

**Disclaimer:** This project pushes the ESP32 to its limit. If you encounter issues, they are likely setup-related. [cite_start]Feel free to report major bugs, but please double-check your wiring and WiFi environment first[cite: 27, 28].
