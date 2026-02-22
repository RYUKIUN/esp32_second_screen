# 📺 ESP32 WiFi Screen Cast (High Performance & Auto-Tuning)

**A low-latency, high-FPS wireless monitor for ~$5 USD.**

Stream your PC screen to an ESP32 + ST7735 display over WiFi using raw UDP packets and dynamic JPEG compression. Unlike standard libraries, this project pushes the ESP32 to its absolute limit using **Dual Core processing**, **DMA (Direct Memory Access)**, and **V-Sync Emulation** to achieve highly stable framerates.

> **Note:** This project relies on the **[Virtual Display Driver](https://github.com/VirtualDrivers/Virtual-Display-Driver)** to create a secondary virtual screen for capturing.


![VID_20260222_222110-ezgif com-video-to-gif-converter (1)](https://github.com/user-attachments/assets/1c9d86de-bc17-4742-9170-f2eb2e47b324)



## 🚀 Features
* **Auto-Discovery:** No need to hardcode IP addresses; the Python script automatically finds your ESP32 on the network.
* **Two-Way Telemetry:** Real-time ESP32 stats (FPS, CPU Temp, Network Jitter, RAM) are sent back and displayed on your PC.
* **AI-Assisted Dynamic Tuning:** A background profiler dynamically adjusts dithering, sharpness, and 4:4:4 / 4:2:0 subsampling based on scene complexity (SSIM) to stay under the strict 5.8KB network limit.
* **Dual Core Architecture:** Separates WiFi networking (Core 0) from Image Decoding (Core 1).
* **Hardware V-Sync:** Emulates V-Sync delays and boosts the internal ST7735 refresh rate to prevent screen tearing.
* **Cursor Tracking:** The Python sender automatically captures and draws your mouse cursor onto the stream.

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
3.  **WiFi Setup:** Open `main.cpp` and look for the **CONFIG** section. Update the `wifi_ssid` and `wifi_pass` to match your PC's hotspot or local network.
4.  **Upload** the code to your ESP32.
5.  The screen will remain black initially while it waits for the UDP stream to start.

### Part 2: PC Sender (Python)
1.  **Python Setup:** Install Python **3.11** (recommended version).
2.  **Virtual Display:**
    * Download and install the [Virtual Display Driver](https://github.com/VirtualDrivers/Virtual-Display-Driver).
    * **Recommendation:** Set the virtual second screen resolution to **1366*7668** in Windows Settings. The script automatically targets the monitor with a resolution under 1920 pixels wide.
3.  **Dependencies:** Install the required libraries with this command:
    ```bash
    pip install opencv-python mss numpy psutil scikit-image
    ```
4.  **Connection:** Ensure your PC and the ESP32 are on the same WiFi network (or use a PC Mobile Hotspot for the lowest latency).
5.  **Run:** Execute the script:
    ```bash
    python capture.py
    ```
    The script will automatically discover the ESP32 and begin streaming.

---

## ⚙️ Control Panel & Tuning
When you run `capture.py`, an OpenCV control window will appear. Use the trackbars to optimize your stream:

| Trackbar | Function |
| :--- | :--- |
| **Max FPS** | Caps the capture rate. **40 FPS** is the default sweet spot. |
| **Base Qual** | Sets the maximum desired JPEG quality (0-95). |
| **Mode: FAST/TUNE** | **0 (FAST):** Uses simple heuristics for quick compression.<br>**1 (TUNE):** Enables the exhaustive background profiler to constantly calculate the highest possible visual quality (using structural similarity) that fits the network budget. |
| **Debug Info** | Toggles the real-time ESP32 telemetry overlay (FPS, Temp, Jitter, RAM) on your PC preview window. |

*(Note: Tuning profiles are automatically saved to `stream_profiles.json` upon exiting so the system learns your screen's content over time!)*

---

## 🐛 Troubleshooting

* **Screen stays black?**
    * Check your wiring against the table above.
    * Ensure the ESP32 and PC are on the same network. 
    * If auto-discovery fails, verify no firewall is blocking UDP Port `12345`.
* **High Jitter or Glitchy Frames?**
    * You are experiencing UDP packet loss. Turn on the **Debug Info** slider to monitor the `Jitter` stat from the ESP32.
    * Move the ESP32 closer to the router/PC.
    * Lower the **Base Qual** slider to reduce the payload size.
* **High CPU Usage on PC?**
    * The `TUNE` mode runs an exhaustive background profiler. If your PC struggles, switch the Mode trackbar to `0 (FAST)`.d
