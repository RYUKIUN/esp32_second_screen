import socket
import time
import cv2
import mss
import numpy as np
import os
import psutil
import ctypes
import threading
import json
from queue import Queue
from skimage.metrics import structural_similarity as ssim

# --- GLOBAL SETTINGS & BIAS ---
PORT = 12345
ESP_W, ESP_H = 160, 128
MAX_FRAME_SIZE = 5800  # 5.8KB limit
MAX_UDP_PAYLOAD = 1024
DEFAULT_FPS = 40
WINDOW_NAME = "Stream Control"
PROFILE_FILE = "stream_profiles.json"

# BIAS CONTROL: 
# Higher = More "afraid" of sharpness (prefers soft/clean).
# Lower = Allows more aggressive sharpening.
SHARP_BIAS = -0.04  

# --- GLOBAL STATE ---
frame_queue = Queue(maxsize=1)
settings_lock = threading.Lock()
stream_profiles = {}
current_mode = 0  
stop_event = threading.Event()

STATIC_NOISE = np.zeros((ESP_H, ESP_W, 3), dtype=np.int8)
cv2.randn(STATIC_NOISE, 0, 2)

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
        if os.name == 'nt': p.nice(psutil.NORMAL_PRIORITY_CLASS)
        else: p.nice(-10)
    except: pass

def get_mouse_pos():
    if os.name == 'nt':
        class POINT(ctypes.Structure): _fields_ = [("x", ctypes.c_long), ("y", ctypes.c_long)]
        pt = POINT()
        ctypes.windll.user32.GetCursorPos(ctypes.byref(pt))
        return pt.x, pt.y
    return 0, 0

# --- PROFILE PERSISTENCE ---
def load_profiles():
    global stream_profiles
    if os.path.exists(PROFILE_FILE):
        try:
            with open(PROFILE_FILE, 'r') as f:
                stream_profiles = json.load(f)
            print(f"[*] Loaded {len(stream_profiles)} profiles.")
        except: stream_profiles = {}

def save_profiles():
    with settings_lock:
        try:
            with open(PROFILE_FILE, 'w') as f:
                json.dump(stream_profiles, f, indent=4)
            print(f"\n[!] JSON Saved: {os.getcwd()}\\{PROFILE_FILE}")
        except Exception as e: print(f"Save Error: {e}")

# --- CONTENT ANALYSIS (Heuristics) ---
def get_scene_key(frame_small):
    gray = cv2.cvtColor(frame_small, cv2.COLOR_BGR2GRAY)
    var = cv2.meanStdDev(gray)[1][0][0]
    edge_density = cv2.Laplacian(gray, cv2.CV_64F).var()
    v_cat = int(np.clip(var / 32, 0, 3))
    e_cat = int(np.clip(edge_density / 100, 0, 3))
    return f"v{v_cat}_e{e_cat}", var, edge_density

# --- EXHAUSTIVE BACKGROUND PROFILER ---
def background_profiler():
    global current_mode
    while not stop_event.is_set():
        for _ in range(100): # Sleep 10s in small chunks for responsive exit
            if stop_event.is_set(): return
            time.sleep(0.1)
            
        if current_mode == 0 or frame_queue.empty(): continue
            
        raw_frame = frame_queue.get()
        target = cv2.resize(raw_frame, (ESP_W, ESP_H), interpolation=cv2.INTER_AREA)
        key, var, edges = get_scene_key(target)
        
        best_overall_score = -999.0
        best_cfg = {"dither": 0, "sharp": 0.0, "sub": 444, "q": 60}
        
        # SEARCH SPACE: Dither x Sharp x Subsampling
        for d in [0, 2, 4]:
            for s in [0.0, 0.1, 0.2, 0.3]:
                for subsampling in [444, 420]:
                    # 1. Apply Pre-filters
                    test_img = target.copy()
                    if d > 0:
                        n = (STATIC_NOISE.astype(np.float32)*(d/2.0)).astype(np.int8)
                        test_img = cv2.add(test_img, n, dtype=cv2.CV_8U)
                    if s > 0:
                        k = np.array([[0, -s, 0], [-s, 1 + 4*s, -s], [0, -s, 0]])
                        test_img = cv2.filter2D(test_img, -1, k)
                    
                    # 2. Binary Search for Max Quality that fits 5.8KB
                    sub_flag = cv2.IMWRITE_JPEG_SAMPLING_FACTOR_444 if subsampling == 444 else cv2.IMWRITE_JPEG_SAMPLING_FACTOR_420
                    low_q, high_q, found_q = 10, 95, 10
                    while low_q <= high_q:
                        mid_q = (low_q + high_q) // 2
                        _, enc = cv2.imencode('.jpg', test_img, [int(cv2.IMWRITE_JPEG_QUALITY), mid_q, int(cv2.IMWRITE_JPEG_SAMPLING_FACTOR), sub_flag])
                        if len(enc) <= MAX_FRAME_SIZE:
                            found_q = mid_q
                            low_q = mid_q + 1
                        else:
                            high_q = mid_q - 1
                    
                    # 3. Score the Result
                    _, final_enc = cv2.imencode('.jpg', test_img, [int(cv2.IMWRITE_JPEG_QUALITY), found_q, int(cv2.IMWRITE_JPEG_SAMPLING_FACTOR), sub_flag])
                    decoded = cv2.imdecode(final_enc, 1)
                    
                    visual_score = ssim(target, decoded, channel_axis=2)
                    
                    # Apply Biases:
                    # - Penalize High Sharpness (the 65/35 bias)
                    # - Slight penalty for 4:2:0 (we prefer 4:4:4 for text clarity)
                    adjusted_score = visual_score - (s * SHARP_BIAS)
                    if subsampling == 420: adjusted_score -= 0.02 
                    
                    if adjusted_score > best_overall_score:
                        best_overall_score = adjusted_score
                        best_cfg = {"dither": d, "sharp": s, "sub": subsampling, "q": found_q}
        
        with settings_lock:
            stream_profiles[key] = best_cfg
            print(f"[EXHAUSTIVE] {key} Optimized -> D:{d} S:{s} Sub:{best_cfg['sub']} Q:{best_cfg['q']}")

# --- CAPTURE & STREAM (Restored Full Features) ---
def select_display_mss():
    with mss.mss() as sct:
        for i, mon in enumerate(sct.monitors):
            if i == 0: continue
            if mon["width"] < 1920: return i
        return 1

def capture_worker(monitor_idx):
    with mss.mss() as sct:
        monitor = sct.monitors[monitor_idx]
        while not stop_event.is_set():
            sct_img = sct.grab(monitor)
            frame = np.frombuffer(sct_img.raw, dtype=np.uint8).reshape((monitor["height"], monitor["width"], 4))[:,:,:3].copy()
            if frame_queue.full():
                try: frame_queue.get_nowait()
                except: pass
            frame_queue.put(frame)

def stream_mss_udp(target_ip, monitor_idx):
    global current_mode
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 0))
    sock.setblocking(False)
    
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 400, 520)
    cv2.createTrackbar("Max FPS", WINDOW_NAME, DEFAULT_FPS, 60, lambda x: None)
    cv2.createTrackbar("Base Qual", WINDOW_NAME, 60, 95, lambda x: None)
    cv2.createTrackbar("Mode: FAST/TUNE", WINDOW_NAME, 0, 1, lambda x: None)
    cv2.createTrackbar("Debug Info", WINDOW_NAME, 1, 1, lambda x: None)

    threading.Thread(target=capture_worker, args=(monitor_idx,), daemon=True).start()
    threading.Thread(target=background_profiler, daemon=True).start()
    
    with mss.mss() as sct:
        m = sct.monitors[monitor_idx]
        m_left, m_top, m_w, m_h = m["left"], m["top"], m["width"], m["height"]

    latest_esp_log = "Waiting..."
    last_debug_send = 0

    try:
        while True:
            if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1: break
            t_start = time.perf_counter()
            
            fps = cv2.getTrackbarPos("Max FPS", WINDOW_NAME)
            user_qual = cv2.getTrackbarPos("Base Qual", WINDOW_NAME)
            current_mode = cv2.getTrackbarPos("Mode: FAST/TUNE", WINDOW_NAME)
            debug_state = cv2.getTrackbarPos("Debug Info", WINDOW_NAME)

            # Receive Stats
            try:
                while True:
                    data, _ = sock.recvfrom(1024)
                    if len(data) > 2 and data[0] == 0xAB: latest_esp_log = data[2:].decode('utf-8', errors='ignore')
            except: pass

            # Send Control
            if time.time() - last_debug_send > 0.5:
                sock.sendto(bytes([0xAA, 0xCC, 0x01, debug_state]), (target_ip, PORT))
                last_debug_send = time.time()

            if frame_queue.empty(): continue
            frame = frame_queue.get()

            # Cursor
            mx, my = get_mouse_pos()
            rx, ry = mx - m_left, my - m_top
            if 0 <= rx < m_w and 0 <= ry < m_h:
                cv2.circle(frame, (rx, ry), 8, (255, 255, 255), 2)
                cv2.circle(frame, (rx, ry), 5, (0, 0, 255), -1)

            resized = cv2.resize(frame, (ESP_W, ESP_H), interpolation=cv2.INTER_AREA)
            key, var, edges = get_scene_key(resized)

            # APPLY LOGIC
            d_amt, s_amt, sub, q_final = 0, 0.0, 444, user_qual
            
            # 1. Start with Profile (if exists)
            with settings_lock:
                if key in stream_profiles:
                    d_amt = stream_profiles[key]["dither"]
                    s_amt = stream_profiles[key]["sharp"]
                    sub = stream_profiles[key]["sub"]
                    q_final = min(user_qual, stream_profiles[key]["q"])
            
            # 2. Heuristic Overrides (if in FAST mode or no profile)
            if current_mode == 0 or key not in stream_profiles:
                d_amt = 2 if var < 15 else 0
                s_amt = 0.15 if edges > 300 else 0.0
                sub = 420

            # Filters
            if d_amt > 0:
                n = (STATIC_NOISE.astype(np.float32)*(d_amt/2.0)).astype(np.int8)
                resized = cv2.add(resized, n, dtype=cv2.CV_8U)
            if s_amt > 0:
                k = np.array([[0, -s_amt, 0], [-s_amt, 1 + 4*s_amt, -s_amt], [0, -s_amt, 0]])
                resized = cv2.filter2D(resized, -1, k)

            # Final Safety Encoding
            sub_flag = cv2.IMWRITE_JPEG_SAMPLING_FACTOR_420 if sub == 420 else cv2.IMWRITE_JPEG_SAMPLING_FACTOR_444
            while q_final > 10:
                params = [int(cv2.IMWRITE_JPEG_QUALITY), q_final, int(cv2.IMWRITE_JPEG_SAMPLING_FACTOR), sub_flag]
                _, encoded = cv2.imencode('.jpg', resized, params)
                if len(encoded) <= MAX_FRAME_SIZE: break
                q_final -= 5

            # Transmit
            total = len(encoded)
            chunks = (total + MAX_UDP_PAYLOAD - 1) // MAX_UDP_PAYLOAD
            data_bytes = encoded.tobytes()
            for i in range(chunks):
                header = bytes([0xAA, 0xBB, i, chunks])
                sock.sendto(header + data_bytes[i*MAX_UDP_PAYLOAD : (i+1)*MAX_UDP_PAYLOAD], (target_ip, PORT))

            # Preview & Debug Overlay
            preview = cv2.resize(resized, (400, 320), interpolation=cv2.INTER_NEAREST)
            if debug_state == 1:
                overlay = preview.copy()
                cv2.rectangle(overlay, (0, 0), (400, 320), (0, 0, 0), -1)
                preview = cv2.addWeighted(overlay, 0.85, preview, 0.15, 0)
                y_offset = 30
                for line in latest_esp_log.split('|'):
                    cv2.putText(preview, line.strip(), (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
                    y_offset += 25
            else:
                info = f"{len(encoded)}B | Q:{q_final} | Sub:{sub} | {'[Tuning]' if current_mode else '[Fast]'}"
                cv2.putText(preview, info, (10, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            cv2.imshow(WINDOW_NAME, preview)
            wait = max(1, int(((1.0/max(1,fps)) - (time.perf_counter()-t_start)) * 1000))
            if cv2.waitKey(wait) & 0xFF == ord('q'): break

    except KeyboardInterrupt: print("\n[!] Manual Interrupt.")
    finally:
        stop_event.set()
        save_profiles()
        cv2.destroyAllWindows()
        sock.close()

if __name__ == "__main__":
    set_high_priority(); set_high_resolution_timer()
    load_profiles()
    target_ip = None # Discovery logic here or hardcode
    # ip = find_esp32() ...
    # For now, manually add your IP if find_esp32 fails
    import socket as s
    def quick_find():
        s_obj = s.socket(s.AF_INET, s.SOCK_DGRAM)
        s_obj.bind(('0.0.0.0', PORT))
        s_obj.settimeout(3.0)
        try: return s_obj.recvfrom(1024)[1][0]
        except: return None
        finally: s_obj.close()
    
    ip = quick_find()
    if ip: stream_mss_udp(ip, select_display_mss())
    reset_resolution_timer()