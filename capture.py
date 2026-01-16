import socket
import time
import cv2
import dxcam
import numpy as np
import os
import psutil
import gc # - Added for COM memory cleanup

# --- CONFIGURATION DEFAULTS ---
PORT = 12345
ESP_W, ESP_H = 160, 128
DEFAULT_JPEG_QUALITY = 70
MAX_UDP_PAYLOAD = 1024
DEFAULT_FPS = 27
WINDOW_NAME = "Stream Control"

# --- ROBUST DISPLAY DISCOVERY ---
def auto_select_display():
    print("\n--- PROBING DISPLAYS (Please wait) ---")
    
    MAX_GPUS = 2
    MAX_MONITORS = 3

    for gpu_idx in range(MAX_GPUS):
        for monitor_idx in range(MAX_MONITORS):
            try:
                temp_cam = dxcam.create(device_idx=gpu_idx, output_idx=monitor_idx)
                width, height = temp_cam.width, temp_cam.height
                print(f"Checked GPU {gpu_idx} Monitor {monitor_idx}: {width}x{height}")
                
                if width < 1920 and height < 1080:
                    print(f"✅ MATCH FOUND! Using GPU {gpu_idx}, Monitor {monitor_idx}")
                    del temp_cam 
                    gc.collect() # - Force cleanup to avoid COM access violation
                    return gpu_idx, monitor_idx
                
                del temp_cam
                gc.collect()
            except Exception:
                continue
                
    print("⚠️ No small displays found. Defaulting to GPU 0, Monitor 0.")
    return 0, 0

# --- SYSTEM PRIORITY ---
def set_high_priority():
    try:
        p = psutil.Process(os.getpid())
        if os.name == 'nt':
            p.nice(psutil.REALTIME_PRIORITY_CLASS)
        else:
            p.nice(10)
        print("🚀 Process priority set to HIGH.")
    except Exception as e:
        print(f"⚠️ Failed to set priority: {e}")

# --- AUTO-DISCOVERY ---
def find_esp32():
    print(f"\n--- SCANNING FOR ESP32 ON PORT {PORT} ---")
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        listener.bind(('0.0.0.0', PORT))
    except OSError:
        print(f"❌ Error: Port {PORT} is busy.")
        return None

    listener.settimeout(5.0)
    print("Waiting for 'ESP32_READY' signal...")
    
    try:
        while True:
            data, addr = listener.recvfrom(1024)
            message = data.decode('utf-8', errors='ignore')
            if "ESP32_READY" in message:
                print(f"✅ FOUND ESP32 AT: {addr[0]}")
                listener.close()
                return addr[0]
    except socket.timeout:
        print("⚠️ Scan timed out. Is the ESP32 on?")
        return None
    except Exception as e:
        print(f"⚠️ Scanner Error: {e}")
        return None

def nothing(x):
    pass

# --- MAIN STREAMING LOGIC ---
def stream_dxcam_udp(target_ip, gpu_idx, mon_idx):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    try:
        camera = dxcam.create(device_idx=gpu_idx, output_idx=mon_idx, output_color="BGR", max_buffer_len=1)
        print(f"📷 DXCam Stream Started on GPU {gpu_idx}, Monitor {mon_idx}")
    except Exception as e:
        print(f"❌ Error initializing DXCam: {e}")
        return

    cv2.namedWindow(WINDOW_NAME)
    cv2.createTrackbar("FPS", WINDOW_NAME, DEFAULT_FPS, 60, nothing)
    cv2.createTrackbar("Quality", WINDOW_NAME, DEFAULT_JPEG_QUALITY, 100, nothing)
    cv2.createTrackbar("Mode (0:Area 1:Lin)", WINDOW_NAME, 0, 1, nothing)

    camera.start(target_fps=60, video_mode=True) 
    print(f"📡 Streaming to {target_ip}...")

    while True:
        frame_start = time.time()
        
        current_fps = cv2.getTrackbarPos("FPS", WINDOW_NAME)
        quality_val = cv2.getTrackbarPos("Quality", WINDOW_NAME)
        mode_val = cv2.getTrackbarPos("Mode (0:Area 1:Lin)", WINDOW_NAME)

        if current_fps < 1: current_fps = 1
        if quality_val < 10: quality_val = 10

        target_frame_time = 1.0 / current_fps
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality_val]

        # - Added check to prevent silent hang if GPU doesn't return frame
        img = camera.get_latest_frame()

        if img is not None:
            if mode_val == 0:
                resized = cv2.resize(img, (ESP_W, ESP_H), interpolation=cv2.INTER_AREA)
            else:
                resized = cv2.resize(img, (ESP_W, ESP_H), interpolation=cv2.INTER_LINEAR)
            
            preview_display = cv2.resize(resized, (320, 256), interpolation=cv2.INTER_NEAREST)
            cv2.imshow(WINDOW_NAME, preview_display)

            _, encoded_img = cv2.imencode('.jpg', resized, encode_param)
            data = encoded_img.tobytes()
            
            total_len = len(data)
            chunks = (total_len + MAX_UDP_PAYLOAD - 1) // MAX_UDP_PAYLOAD
            
            transmit_budget = (target_frame_time * 0.2) 
            chunk_delay = transmit_budget / chunks if chunks > 0 else 0

            for i in range(chunks):
                start = i * MAX_UDP_PAYLOAD
                end = min((i + 1) * MAX_UDP_PAYLOAD, total_len)
                
                # MAGIC HEADER: Prepend 0xAA 0xBB
                header = bytes([0xAA, 0xBB, i])
                sock.sendto(header + data[start:end], (target_ip, PORT))
                if chunk_delay > 0: time.sleep(chunk_delay)

        elapsed = time.time() - frame_start
        wait_time = target_frame_time - elapsed
        
        if wait_time > 0:
            if cv2.waitKey(int(wait_time * 1000)) & 0xFF == ord('q'):
                break
        else:
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    camera.stop()
    cv2.destroyAllWindows()
    sock.close()
    print("🛑 Stream ended.")

if __name__ == "__main__":
    try:
        set_high_priority()
        gpu_idx, mon_idx = auto_select_display()
        found_ip = find_esp32()
        
        if found_ip:
            time.sleep(0.5) 
            stream_dxcam_udp(found_ip, gpu_idx, mon_idx)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopped by User")