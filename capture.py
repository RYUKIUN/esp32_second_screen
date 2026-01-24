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
DEBUG_PERFORMANCE = True
PORT = 12345
ESP_W, ESP_H = 160, 128
DEFAULT_JPEG_QUALITY = 70
MAX_UDP_PAYLOAD = 1024
DEFAULT_FPS = 80
WINDOW_NAME = "Stream Control"

# Thread-safe queue for frames (Max 1 frame to keep latency low)
frame_queue = Queue(maxsize=1)

# --- SYSTEM HELPERS ---
def set_high_resolution_timer():
    if os.name == 'nt':
        try:
            ctypes.windll.winmm.timeBeginPeriod(1)
            print("⏱️  System timer resolution set to 1ms.")
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

# --- CAPTURE THREAD (PRODUCER) ---
def capture_worker(monitor_idx):
    with mss.mss() as sct:
        monitor = sct.monitors[monitor_idx]
        m_h, m_w = monitor["height"], monitor["width"]
        
        while True:
            # Grab frame
            sct_img = sct.grab(monitor)
            
            # Fast raw conversion (Ryzen optimized)
            img_bgra = np.frombuffer(sct_img.raw, dtype=np.uint8).reshape((m_h, m_w, 4))
            frame = img_bgra[:, :, :3].copy()
            
            # Update queue (Overwrite old frame if queue is full)
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put(frame)

# --- MAIN STREAM THREAD (CONSUMER) ---
def stream_mss_udp(target_ip, monitor_idx):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # Setup UI
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 400, 500)
    cv2.createTrackbar("FPS", WINDOW_NAME, DEFAULT_FPS, 60, lambda x: None)
    cv2.createTrackbar("Quality", WINDOW_NAME, DEFAULT_JPEG_QUALITY, 100, lambda x: None)
    cv2.createTrackbar("Mode (0:Area 1:Lin)", WINDOW_NAME, 0, 1, lambda x: None)

    # Start Capture Thread
    t = threading.Thread(target=capture_worker, args=(monitor_idx,), daemon=True)
    t.start()
    
    # Get monitor offset for mouse calc
    with mss.mss() as sct:
        m = sct.monitors[monitor_idx]
        m_left, m_top, m_w, m_h = m["left"], m["top"], m["width"], m["height"]

    frame_count = 0
    stat_start = time.time()
    acc_proc, acc_enc, acc_send = 0, 0, 0

    print(f"🚀 Threaded Stream Started -> {target_ip}")

    try:
        while True:
            t_loop_start = time.perf_counter()
            
            # 1. UI Inputs
            fps = cv2.getTrackbarPos("FPS", WINDOW_NAME)
            qual = cv2.getTrackbarPos("Quality", WINDOW_NAME)
            mode = cv2.getTrackbarPos("Mode (0:Area 1:Lin)", WINDOW_NAME)
            target_dt = 1.0 / max(1, fps)

            # 2. Get latest frame
            if frame_queue.empty():
                time.sleep(0.001)
                continue
            frame = frame_queue.get()

            # 3. Process (Mouse + Resize)
            t_proc_s = time.perf_counter()
            mx, my = get_mouse_pos()
            rel_x, rel_y = mx - m_left, my - m_top
            
            if 0 <= rel_x < m_w and 0 <= rel_y < m_h:
                cv2.circle(frame, (rel_x, rel_y), 8, (255, 255, 255), 2)
                cv2.circle(frame, (rel_x, rel_y), 5, (0, 0, 255), -1)
            
            interp = cv2.INTER_AREA if mode == 0 else cv2.INTER_LINEAR
            resized = cv2.resize(frame, (ESP_W, ESP_H), interpolation=interp)
            acc_proc += (time.perf_counter() - t_proc_s)

            # 4. Encode
            t_enc_s = time.perf_counter()
            _, encoded = cv2.imencode('.jpg', resized, [int(cv2.IMWRITE_JPEG_QUALITY), max(10, qual)])
            data = encoded.tobytes()
            acc_enc += (time.perf_counter() - t_enc_s)

            # 5. Send
            t_snd_s = time.perf_counter()
            total = len(data)
            chunks = (total + MAX_UDP_PAYLOAD - 1) // MAX_UDP_PAYLOAD
            for i in range(chunks):
                start, end = i * MAX_UDP_PAYLOAD, min((i+1)*MAX_UDP_PAYLOAD, total)
                sock.sendto(bytes([0xAA, 0xBB, i]) + data[start:end], (target_ip, PORT))
                if chunks > 5: time.sleep(0.0001) # Tiny pacing
            acc_send += (time.perf_counter() - t_snd_s)

            # 6. Show Preview (Delayed to prioritize network)
            cv2.imshow(WINDOW_NAME, resized)

            # Stats
            frame_count += 1
            if time.time() - stat_start >= 1.0:
                if DEBUG_PERFORMANCE:
                    avg = lambda x: (x / frame_count) * 1000
                    print(f"[THREADED] FPS: {frame_count/(time.time()-stat_start):.1f} | "
                          f"Proc:{avg(acc_proc):.1f}ms Enc:{avg(acc_enc):.1f}ms Send:{avg(acc_send):.1f}ms")
                frame_count, acc_proc, acc_enc, acc_send = 0, 0, 0, 0
                stat_start = time.time()

            # Timing
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
            print("❌ ESP32 not found. Check if it's powered on and on the same Wi-Fi.")
    finally:
        reset_resolution_timer()