import socket
import time
import cv2
import mss
import numpy as np
import os
import psutil
import ctypes
import threading
from queue import Queue

# --- CONFIGURATION ---
PORT = 12345
ESP_W, ESP_H = 160, 128
DEFAULT_JPEG_QUALITY = 70
MAX_UDP_PAYLOAD = 1024
DEFAULT_FPS = 80
WINDOW_NAME = "Stream Control"

# Thread-safe queue
frame_queue = Queue(maxsize=1)

# --- SYSTEM HELPERS ---
def set_high_resolution_timer():
    if os.name == 'nt':
        try: ctypes.windll.winmm.timeBeginPeriod(1)
        except: pass

def reset_resolution_timer():
    if os.name == 'nt':
        try: ctypes.windll.winmm.timeEndPeriod(1)
        except: pass

def set_high_priority():
    try:
        p = psutil.Process(os.getpid())
        if os.name == 'nt': p.nice(psutil.HIGH_PRIORITY_CLASS)
        else: p.nice(-10)
        print("🚀 Process priority set to HIGH.")
    except: pass

# --- MOUSE HELPERS ---
class POINT(ctypes.Structure):
    _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]

def get_mouse_pos():
    if os.name == 'nt':
        pt = POINT()
        ctypes.windll.user32.GetCursorPos(ctypes.byref(pt))
        return pt.x, pt.y
    return 0, 0

# --- DISCOVERY ---
def select_display_mss():
    print("\n--- DETECTING MONITORS ---")
    with mss.mss() as sct:
        monitors = sct.monitors
        for i, mon in enumerate(monitors):
            if i == 0: continue
            print(f"Monitor {i}: {mon['width']}x{mon['height']}")
            if mon["width"] < 1920:
                print(f"✅ Auto-selected Monitor {i}")
                return i
        return 1

def find_esp32():
    print(f"\n--- SCANNING FOR ESP32 (Port {PORT}) ---")
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

# --- CAPTURE THREAD ---
def capture_worker(monitor_idx):
    with mss.mss() as sct:
        monitor = sct.monitors[monitor_idx]
        m_h, m_w = monitor["height"], monitor["width"]
        
        while True:
            sct_img = sct.grab(monitor)
            img_bgra = np.frombuffer(sct_img.raw, dtype=np.uint8).reshape((m_h, m_w, 4))
            frame = img_bgra[:, :, :3].copy()
            
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put(frame)

# --- MAIN STREAM THREAD ---
def stream_mss_udp(target_ip, monitor_idx):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0)) 
    sock.setblocking(False)   
    
    # UI Setup
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 400, 500)
    
    # CONTROLS
    cv2.createTrackbar("FPS", WINDOW_NAME, DEFAULT_FPS, 60, lambda x: None)
    cv2.createTrackbar("Quality", WINDOW_NAME, DEFAULT_JPEG_QUALITY, 100, lambda x: None)
    cv2.createTrackbar("Mode (0:Area 1:Lin)", WINDOW_NAME, 0, 1, lambda x: None)
    cv2.createTrackbar("Debug Info", WINDOW_NAME, 0, 1, lambda x: None)

    t = threading.Thread(target=capture_worker, args=(monitor_idx,), daemon=True)
    t.start()
    
    with mss.mss() as sct:
        m = sct.monitors[monitor_idx]
        m_left, m_top, m_w, m_h = m["left"], m["top"], m["width"], m["height"]

    print(f"🚀 Stream Started -> {target_ip}")
    print("ℹ️  Use the 'Debug Info' slider in the window to toggle stats.")

    last_debug_send = 0
    latest_esp_log = "Waiting for stats..."
    
    try:
        while True:
            t_loop_start = time.perf_counter()
            
            # --- 1. HANDLE UI INPUTS ---
            fps = cv2.getTrackbarPos("FPS", WINDOW_NAME)
            qual = cv2.getTrackbarPos("Quality", WINDOW_NAME)
            mode = cv2.getTrackbarPos("Mode (0:Area 1:Lin)", WINDOW_NAME)
            debug_state = cv2.getTrackbarPos("Debug Info", WINDOW_NAME)
            
            target_dt = 1.0 / max(1, fps)

            # --- 2. RECEIVE UDP STATS ---
            try:
                while True: 
                    data, _ = sock.recvfrom(1024)
                    if len(data) > 2 and data[0] == 0xAB and data[1] == 0xCD:
                        latest_esp_log = data[2:].decode('utf-8', errors='ignore')
            except BlockingIOError: pass
            except Exception: pass

            # --- 3. SEND CONTROL PACKET ---
            if time.time() - last_debug_send > 0.5:
                packet = bytes([0xAA, 0xCC, 0x01, debug_state])
                sock.sendto(packet, (target_ip, PORT))
                last_debug_send = time.time()

            # --- 4. GET FRAME ---
            if frame_queue.empty():
                time.sleep(0.001)
                continue
            frame = frame_queue.get()

            # --- 5. PROCESS FRAME ---
            mx, my = get_mouse_pos()
            rel_x, rel_y = mx - m_left, my - m_top
            
            if 0 <= rel_x < m_w and 0 <= rel_y < m_h:
                cv2.circle(frame, (rel_x, rel_y), 8, (255, 255, 255), 2)
                cv2.circle(frame, (rel_x, rel_y), 5, (0, 0, 255), -1)
            
            interp = cv2.INTER_AREA if mode == 0 else cv2.INTER_LINEAR
            resized = cv2.resize(frame, (ESP_W, ESP_H), interpolation=interp)

            # --- 6. ENCODE (COMPRESS) ---
            # We use 'encoded_buf' for both sending and decoding locally
            ret, encoded_buf = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), max(10, qual)])
            
            if ret:
                data = encoded_buf.tobytes()
                
                # --- SEND ---
                total = len(data)
                chunks = (total + MAX_UDP_PAYLOAD - 1) // MAX_UDP_PAYLOAD
                for i in range(chunks):
                    start, end = i * MAX_UDP_PAYLOAD, min((i+1)*MAX_UDP_PAYLOAD, total)
                    header = bytes([0xAA, 0xBB, i, chunks])
                    sock.sendto(header + data[start:end], (target_ip, PORT))
                    if i % 2 == 0: time.sleep(0)

                # --- 7. DECODE FOR PREVIEW (SHOW COMPRESSION ARTIFACTS) ---
                # This ensures the preview looks exactly like what the ESP32 receives
                decompressed_frame = cv2.imdecode(encoded_buf, cv2.IMREAD_COLOR)
                
                # Upscale for visibility on PC screen
                preview = cv2.resize(decompressed_frame, (400, 320), interpolation=cv2.INTER_NEAREST)

                # If UI Toggle is ON (1), draw the overlay
                if debug_state == 1:
                    overlay = preview.copy()
                    cv2.rectangle(overlay, (0, 0), (400, 320), (0, 0, 0), -1)
                    preview = cv2.addWeighted(overlay, 0.85, preview, 0.15, 0)
                    
                    y_offset = 30
                    for line in latest_esp_log.split('|'):
                        cv2.putText(preview, line.strip(), (10, y_offset), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                        y_offset += 25
                    
                    cv2.putText(preview, "[DEBUG ON]", (300, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                
                cv2.imshow(WINDOW_NAME, preview)

            # --- 8. LOOP TIMING ---
            elapsed = time.perf_counter() - t_loop_start
            wait = max(1, int((target_dt - elapsed) * 1000))
            if cv2.waitKey(wait) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt: pass
    finally:
        cv2.destroyAllWindows()
        sock.close()
        print("🛑 Stream Closed")

if __name__ == "__main__":
    set_high_priority()
    set_high_resolution_timer()
    try:
        mon_idx = select_display_mss()
        found_ip = find_esp32()
        if found_ip:
            time.sleep(0.5)
            stream_mss_udp(found_ip, mon_idx)
        else:
            print("❌ ESP32 not found.")
    finally:
        reset_resolution_timer()