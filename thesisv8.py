import cv2
from paddleocr import PaddleOCR
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
import re
import sys
import warnings 
import logging
from datetime import datetime
from collections import Counter, deque
import csv 
import tkinter as tk
from tkinter import ttk, font
from PIL import Image, ImageTk
from gpiozero import OutputDevice, DistanceSensor, Buzzer, LED 
from picamera2 import Picamera2

# --- INITIALIZATION & SUPPRESSION ---
warnings.filterwarnings("ignore")
logging.getLogger("ppocr").setLevel(logging.ERROR)

# ==========================================
#             CONFIGURATION
# ==========================================
AUTH_FILE = 'Database/authorized.csv'
LOG_FILE = 'Database/access_logs.csv'
LOG_COOLDOWN = 5  
SCAN_BUFFER_LEN = 9  
CONFIDENCE_THRESHOLD = 4

ROI_SCALE_W, ROI_SCALE_H = 0.7, 0.5 
ROI_COLOR = (0, 255, 0) 

GRACE_PERIOD = 0.3      
GATE_ACTION_TIME = 3000 
ABSENCE_RESET_TIME = 2.5 

SAFETY_DISTANCE_CM = 50 
ENTRY_CONFIRM_TIME = 0.5 
SENSOR_POLL_RATE = 100    

POST_ENTRY_DELAY = 0.5   
EXIT_SCAN_COOLDOWN = 5.0 

# --- HARDWARE PINS ---
GATE_PIN_1 = 17 
GATE_PIN_2 = 27 
US_TRIG_PIN = 23 
US_ECHO_PIN = 24 
BUZZER_PIN = 22
LED_OPEN_PIN = 5   
LED_CLOSE_PIN = 6   

# ==========================================
#          HARDWARE & MODELS
# ==========================================
paddle_reader = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=False, show_log=False)
model = YOLO('./best_ncnn_model') 

try:
    gate_p1 = OutputDevice(GATE_PIN_1, active_high=True, initial_value=False)
    gate_p2 = OutputDevice(GATE_PIN_2, active_high=True, initial_value=True)
    sensor = DistanceSensor(echo=US_ECHO_PIN, trigger=US_TRIG_PIN, max_distance=1.5, queue_len=3)
    buzzer = Buzzer(BUZZER_PIN)
    led_open = LED(LED_OPEN_PIN)
    led_close = LED(LED_CLOSE_PIN)
    led_open.off()
    led_close.on()
    
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (1280, 720)})
    picam2.configure(config)
    picam2.set_controls({"AfMode": 2, "AwbMode": 3}) 
    picam2.start()
    print(f"✅ Hardware Initialized")
    
except Exception as e:
    print(f"⚠️ INITIALIZATION ERROR: {e}")
    class DummyDev: 
        def on(self): pass
        def off(self): pass
        def beep(self, on_time=0.1, off_time=0.1, n=1): pass
        def close(self): pass
        @property
        def value(self): return 0
        @property
        def distance(self): return 1.5 
    gate_p1 = gate_p2 = sensor = buzzer = led_open = led_close = DummyDev()
    picam2 = None 

# ==========================================
#               GUI SETUP (70% SCALED)
# ==========================================
root = tk.Tk()
root.title("SAVES AI Control System")
root.configure(bg="white")
# Scaled full screen simulation or fixed size (70% of 1080p is roughly 1344x756)
root.attributes('-fullscreen', True)
root.bind("<Escape>", lambda event: root.attributes("-fullscreen", False))

style = ttk.Style()
style.theme_use("clam")
# Scaled fonts (Original 10 -> 7, 9 -> 6)
style.configure("Treeview.Heading", background="#cccccc", font=("Arial", 7, "bold"))
style.configure("Treeview", font=("Arial", 6), rowheight=18)

container = tk.Frame(root, bg="white")
container.pack(fill="both", expand=True)

page_camera = tk.Frame(container, bg="white")
page_logs = tk.Frame(container, bg="white")
page_history = tk.Frame(container, bg="white")

for frame in (page_camera, page_logs, page_history):
    frame.grid(row=0, column=0, sticky="nsew")

def show_frame(frame):
    frame.tkraise()

# --- GUI HELPERS (70% SCALED) ---
def create_header(parent, title_text, sub_text=None, left_btn=None, right_btn=None):
    header_frame = tk.Frame(parent, bg="white", pady=3) # Scaled padding
    header_frame.pack(fill="x", padx=42) # Scaled 60 -> 42
    header_frame.columnconfigure(0, weight=1)
    header_frame.columnconfigure(1, weight=4)
    header_frame.columnconfigure(2, weight=1)

    if left_btn:
        btn = tk.Button(header_frame, text=left_btn['text'], command=left_btn['cmd'],
                        bg="#f0f0f0", fg=left_btn.get('color', 'black'), 
                        font=("Arial", 6, "bold"), relief="flat", padx=7) # Scaled padding/font
        btn.grid(row=0, column=0, sticky="w")

    title_container = tk.Frame(header_frame, bg="white")
    title_container.grid(row=0, column=1)
    # Scaled 18 -> 13
    tk.Label(title_container, text=title_text, font=("Helvetica", 13, "bold"), bg="white").pack()
    if sub_text:
        # Scaled 10 -> 7
        tk.Label(title_container, text=sub_text, font=("Helvetica", 7), bg="white", fg="#555").pack()

    if right_btn:
        btn = tk.Button(header_frame, text=right_btn['text'], command=right_btn['cmd'],
                        bg="#f0f0f0", fg="black", font=("Arial", 6, "bold"), relief="flat", padx=7)
        btn.grid(row=0, column=2, sticky="e")

def create_treeview(parent):
    cols = ("Time", "Plate", "Name", "Status", "Latency", "Performance Metrics")
    frame = tk.Frame(parent, bg="white")
    frame.pack(fill="both", expand=True, padx=42, pady=3) # Scaled 60/5 -> 42/3
    tree = ttk.Treeview(frame, columns=cols, show="headings")
    scrollbar = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side="right", fill="y")
    tree.pack(side="left", fill="both", expand=True)
    for col in cols:
        tree.heading(col, text=col)
        tree.column(col, anchor="center", width=63) # Scaled width 90 -> 63
    tree.tag_configure("authorized", foreground="green")
    tree.tag_configure("unauthorized", foreground="red")
    return tree

# --- PAGE ASSEMBLING ---
create_header(page_camera, "SAVES AI", None,
              left_btn={'text': "Shutdown", 'cmd': sys.exit, 'color': 'red'},
              right_btn={'text': "Session Logs ➜", 'cmd': lambda: show_frame(page_logs)})

video_label = tk.Label(page_camera, bg="black")
video_label.pack(fill="both", expand=True, padx=42, pady=3)

# Scaled font 11 -> 8
lbl_status = tk.Label(page_camera, text="SCANNING", font=("Arial", 8, "bold"), bg="white")
lbl_status.pack(side="left", padx=42)
lbl_dist = tk.Label(page_camera, text="DIST: -- cm", font=("Arial", 8), bg="white")
lbl_dist.pack(side="right", padx=42)

tree = create_treeview(page_logs)
create_header(page_logs, "SAVES AI", "current session",
              left_btn={'text': "← camera", 'cmd': lambda: show_frame(page_camera)},
              right_btn={'text': "history ➜", 'cmd': lambda: show_frame(page_history)})

tree_history = create_treeview(page_history)
create_header(page_history, "SAVES AI", "past history",
              left_btn={'text': "← camera", 'cmd': lambda: show_frame(page_camera)},
              right_btn={'text': "current ➜", 'cmd': lambda: show_frame(page_logs)})

# ==========================================
#                LOGIC
# ==========================================
logged_vehicles, scan_buffer, first_sight_times = {}, deque(maxlen=SCAN_BUFFER_LEN), {}
detection_start_time, last_plate_seen_time = None, 0
is_gate_busy, system_state = False, "SCANNING"
vehicle_entry_start_time, vehicle_confirmed = None, False

try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except:
    authorized_plates, auth_df = [], pd.DataFrame(columns=["Plate", "Name"])

def get_paddle_text(img):
    try:
        res = paddle_reader.ocr(img, cls=True)
        return "".join([line[1][0] for line in res[0]]) if res and res[0] else ""
    except: return ""

def log_to_gui_and_csv(plate, name, status, latency, det_t, ocr_t):
    ts_s = datetime.now().strftime("%H:%M:%S")
    perf = f"D:{det_t:.0f}ms|O:{ocr_t:.0f}ms"
    tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
    tree.insert("", 0, values=(ts_s, plate, name, status, f"{latency:.2f}s", perf), tags=(tag,))
    if status == "AUTHORIZED": trigger_gate_sequence()

def trigger_gate_sequence():
    global is_gate_busy; is_gate_busy = True
    lbl_status.config(text="OPENING", fg="green")
    gate_p1.on(); gate_p2.off(); led_open.on(); led_close.off()
    root.after(GATE_ACTION_TIME, smart_gate_check)

def smart_gate_check():
    global is_gate_busy, vehicle_confirmed, vehicle_entry_start_time
    try:
        dist = sensor.distance * 100
        lbl_dist.config(text=f"DIST: {dist:.1f} cm")
        if not vehicle_confirmed and dist < SAFETY_DISTANCE_CM:
            if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
            if time.time() - vehicle_entry_start_time > ENTRY_CONFIRM_TIME: vehicle_confirmed = True
        elif vehicle_confirmed and dist > SAFETY_DISTANCE_CM:
            gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
            is_gate_busy = vehicle_confirmed = False
            lbl_status.config(text="SCANNING", fg="black")
            return
    except: pass
    root.after(SENSOR_POLL_RATE, smart_gate_check)

def update_frame():
    global detection_start_time, last_plate_seen_time
    if not picam2: return
    frame = picam2.capture_array()
    h, w, _ = frame.shape
    curr_t = time.time()

    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), ROI_COLOR, 2)
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    if not is_gate_busy:
        t0 = time.perf_counter()
        results = model(roi_crop, verbose=False, conf=0.4)
        t_det = (time.perf_counter() - t0) * 1000
        detected = False

        for res in results:
            boxes = res.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected = True; last_plate_seen_time = curr_t
                if detection_start_time is None: detection_start_time = curr_t
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 2)
                    
                    if curr_t - detection_start_time > GRACE_PERIOD:
                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            t_ocr_s = time.perf_counter()
                            txt = get_paddle_text(p_crop)
                            clean = "".join([c for c in txt.upper() if c.isalnum()])
                            t_ocr_m = (time.perf_counter() - t_ocr_s) * 1000
                            
                            if len(clean) > 3:
                                scan_buffer.append(clean)
                                m_com, freq = Counter(scan_buffer).most_common(1)[0]
                                if curr_t - logged_vehicles.get(m_com, 0) > LOG_COOLDOWN:
                                    status = "AUTHORIZED" if m_com in authorized_plates else "UNAUTHORIZED"
                                    name = auth_df[auth_df['Plate']==m_com]['Name'].values[0] if status=="AUTHORIZED" else "Unknown"
                                    log_to_gui_and_csv(m_com, name, status, (t_det+t_ocr_m)/1000, t_det, t_ocr_m)
                                    logged_vehicles[m_com] = curr_t
                                    scan_buffer.clear()

        if not detected and curr_t - last_plate_seen_time > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear()

    # --- VIDEO SCALING (70%) ---
    # Original: 800x450 -> Scaled: 560x315
    win_w, win_h = 560, 315
    img = Image.fromarray(cv2.resize(frame, (win_w, win_h)))
    imgtk = ImageTk.PhotoImage(image=img)
    video_label.imgtk = imgtk; video_label.configure(image=imgtk)
    root.after(10, update_frame)

show_frame(page_camera)
root.after(500, update_frame)
root.mainloop()
