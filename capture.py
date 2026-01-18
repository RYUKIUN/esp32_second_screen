import socket
import time
import cv2
import mss
import numpy as np
import os
import psutil
import ctypes

# --- CONFIGURATION DEFAULTS ---
PORT = 12345
ESP_W, ESP_H = 160, 128
DEFAULT_JPEG_QUALITY = 70
MAX_UDP_PAYLOAD = 1024
DEFAULT_FPS = 30
WINDOW_NAME = "Stream Control"

# --- WINDOWS TIMER RESOLUTION FIX ---
# This forces Windows to allow sleeps as short as 1ms (default is 15.6ms)
# effectively fixing packet pacing without burning CPU.
def set_high_resolution_timer():
    if os.name == 'nt':
        try:
            ctypes.windll.winmm.timeBeginPeriod(1)
            print("⏱️  System timer resolution set to 1ms.")
        except Exception as e:
            print(f"⚠️ Could not set timer resolution: {e}")

def reset_resolution_timer():
    if os.name == 'nt':
        try:
            ctypes.windll.winmm.timeEndPeriod(1)
        except:
            pass

# --- SYSTEM PRIORITY ---
def set_high_priority():
    try:
        p = psutil.Process(os.getpid())
        if os.name == 'nt':
            p.nice(psutil.HIGH_PRIORITY_CLASS)
        else:
            p.nice(-10)
        print("🚀 Process priority set to HIGH.")
    except Exception as e:
        print(f"⚠️ Failed to set priority: {e}")

# --- DISPLAY DISCOVERY (MSS) ---
def select_display_mss():
    print("\n--- DETECTING MONITORS (MSS) ---")
    with mss.mss() as sct:
        monitors = sct.monitors
        # monitors[0] is 'All Monitors', skip it.
        for i, mon in enumerate(monitors):
            if i == 0: continue
            
            w, h = mon["width"], mon["height"]
            print(f"Monitor {i}: {w}x{h}")
            
            if w < 1920:
                print(f"✅ MATCH FOUND! Using Monitor {i}")
                return i
        
        print("⚠️ No small display found. Defaulting to Monitor 1.")
        return 1

# --- NETWORK DISCOVERY ---
def find_esp32():
    print(f"\n--- SCANNING FOR ESP32 ON PORT {PORT} ---")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        sock.bind(('0.0.0.0', PORT))
        sock.settimeout(5.0)
        print("Waiting for 'ESP32_READY' signal...")
        
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if "ESP32_READY" in data.decode('utf-8', errors='ignore'):
                    print(f"✅ FOUND ESP32 AT: {addr[0]}")
                    return addr[0]
            except socket.timeout:
                continue
    except Exception as e:
        print(f"❌ Network Error: {e}")
        return None
    finally:
        sock.close()

def nothing(x):
    pass

# --- MAIN STREAMING LOGIC ---
def stream_mss_udp(target_ip, monitor_idx):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    sct = mss.mss()
    monitor_settings = sct.monitors[monitor_idx]
    
    cv2.namedWindow(WINDOW_NAME)
    cv2.createTrackbar("FPS", WINDOW_NAME, DEFAULT_FPS, 60, nothing)
    cv2.createTrackbar("Quality", WINDOW_NAME, DEFAULT_JPEG_QUALITY, 100, nothing)
    cv2.createTrackbar("Mode (0:Area 1:Lin)", WINDOW_NAME, 0, 1, nothing)
    
    print(f"📡 Streaming Monitor {monitor_idx} to {target_ip}...")

    try:
        while True:
            loop_start = time.perf_counter()
            
            # 1. Inputs
            current_fps = cv2.getTrackbarPos("FPS", WINDOW_NAME)
            quality_val = cv2.getTrackbarPos("Quality", WINDOW_NAME)
            mode_val = cv2.getTrackbarPos("Mode (0:Area 1:Lin)", WINDOW_NAME)

            if current_fps < 1: current_fps = 1
            if quality_val < 10: quality_val = 10

            target_frame_time = 1.0 / current_fps
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality_val]

            # 2. Capture & Resize
            sct_img = sct.grab(monitor_settings)
            img_np = np.array(sct_img)
            frame = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)

            if mode_val == 0:
                resized = cv2.resize(frame, (ESP_W, ESP_H), interpolation=cv2.INTER_AREA)
            else:
                resized = cv2.resize(frame, (ESP_W, ESP_H), interpolation=cv2.INTER_LINEAR)
            
            # Preview
            preview_display = cv2.resize(resized, (320, 256), interpolation=cv2.INTER_NEAREST)
            cv2.imshow(WINDOW_NAME, preview_display)

            # 3. Encode
            _, encoded_img = cv2.imencode('.jpg', resized, encode_param)
            data = encoded_img.tobytes()
            
            # 4. Transmission (With Timer-Corrected Sleep)
            total_len = len(data)
            chunks = (total_len + MAX_UDP_PAYLOAD - 1) // MAX_UDP_PAYLOAD
            
            transmit_budget = target_frame_time * 0.5
            chunk_delay = transmit_budget / chunks if chunks > 0 else 0

            for i in range(chunks):
                start = i * MAX_UDP_PAYLOAD
                end = min((i + 1) * MAX_UDP_PAYLOAD, total_len)
                
                header = bytes([0xAA, 0xBB, i])
                sock.sendto(header + data[start:end], (target_ip, PORT))
                
                # Standard sleep now works correctly (down to 1ms) 
                # because of timeBeginPeriod(1)
                if chunk_delay > 0:
                    time.sleep(chunk_delay)

            # 5. Loop Timing
            elapsed = time.perf_counter() - loop_start
            wait_time = target_frame_time - elapsed
            
            if wait_time > 0:
                if cv2.waitKey(int(wait_time * 1000)) & 0xFF == ord('q'):
                    break
            else:
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        sock.close()
        sct.close()
        print("🛑 Stream ended.")

if __name__ == "__main__":
    set_high_priority()
    set_high_resolution_timer() # <--- THE FIX
    
    try:
        mon_idx = select_display_mss()
        found_ip = find_esp32()
        
        if found_ip:
            time.sleep(0.5)
            stream_mss_udp(found_ip, mon_idx)
        else:
            print("❌ ESP32 not found.")
    finally:
        reset_resolution_timer() # Reset system timer on exit