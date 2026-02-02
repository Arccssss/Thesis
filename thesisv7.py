import cv2
import easyocr
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
import re
import sys
import warnings 
from datetime import datetime
from collections import Counter, deque
import csv 
import tkinter as tk
from tkinter import ttk, font
from PIL import Image, ImageTk
from gpiozero import OutputDevice, DistanceSensor, Buzzer, LED 
from picamera2 import Picamera2

# Suppress PyTorch/Pin Memory warnings
warnings.filterwarnings("ignore")

# ==========================================
#           CONFIGURATION
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

# [NEW] Ultrasonic Stability Config
DISTANCE_BUFFER_LEN = 5 # Number of readings to filter noise
HYSTERESIS_OFFSET = 15  # Extra cm required to confirm vehicle has left

# --- HARDWARE PINS ---
GATE_PIN_1 = 17 
GATE_PIN_2 = 27 
US_TRIG_PIN = 23 
US_ECHO_PIN = 24 
BUZZER_PIN = 22
LED_OPEN_PIN = 5   
LED_CLOSE_PIN = 6   

# ==========================================
#          HARDWARE INITIALIZATION
# ==========================================
try:
    gate_p1 = OutputDevice(GATE_PIN_1, active_high=True, initial_value=False)
    gate_p2 = OutputDevice(GATE_PIN_2, active_high=True, initial_value=True)
    # [FIX] Set max_distance slightly higher to avoid 0.0 clipping on noise
    sensor = DistanceSensor(echo=US_ECHO_PIN, trigger=US_TRIG_PIN, max_distance=2.0, queue_len=5)
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
    print(f"⚠️ HARDWARE ERROR: {e}")
    class DummyDev: 
        def on(self): pass
        def off(self): pass
        def beep(self, on_time=0.1, off_time=0.1, n=1): pass
        def close(self): pass
        @property
        def value(self): return 0
        @property
        def distance(self): return 1.5 
    gate_p1 = gate_p2 = sensor = buzzer = DummyDev()
    led_open = led_close = DummyDev()
    picam2 = None 

# ==========================================
#               GUI SETUP
# ==========================================
root = tk.Tk()
root.title("SAVES AI Control System")
root.configure(bg="white")
root.attributes('-fullscreen', True)
root.bind("<Escape>", lambda event: root.attributes("-fullscreen", False))

style = ttk.Style()
style.theme_use("clam")
style.configure("Treeview.Heading", background="#cccccc", foreground="white", font=("Arial", 10, "bold"), relief="flat")
style.configure("Treeview", font=("Arial", 9), rowheight=25, background="white", fieldbackground="white")

container = tk.Frame(root, bg="white")
container.pack(fill="both", expand=True)

page_camera = tk.Frame(container, bg="white")
page_logs = tk.Frame(container, bg="white")
page_history = tk.Frame(container, bg="white")

for frame in (page_camera, page_logs, page_history):
    frame.grid(row=0, column=0, sticky="nsew")

def show_frame(frame):
    frame.tkraise()

def close_application():
    if picam2: picam2.stop()
    gate_p1.close(); gate_p2.on() 
    led_open.off(); led_close.off()
    root.destroy(); sys.exit(0)

def minimize_window(): root.iconify()

btn_minimize = tk.Button(root, text=" — ", command=minimize_window, bg="#f0f0f0", font=("Arial", 14, "bold"), relief="flat")
btn_minimize.place(relx=1.0, x=-10, y=10, anchor="ne")

def create_header(parent, title_text, sub_text=None, left_btn=None, right_btn=None):
    header_frame = tk.Frame(parent, bg="white", pady=5)
    header_frame.pack(fill="x", padx=60) 
    header_frame.columnconfigure(0, weight=1); header_frame.columnconfigure(1, weight=4); header_frame.columnconfigure(2, weight=1)
    if left_btn:
        btn = tk.Button(header_frame, text=left_btn['text'], command=left_btn['cmd'], bg="#f0f0f0", font=("Arial", 9, "bold"), relief="flat", padx=10)
        if 'color' in left_btn: btn.config(fg=left_btn['color'])
        btn.grid(row=0, column=0, sticky="w")
    title_container = tk.Frame(header_frame, bg="white")
    title_container.grid(row=0, column=1)
    tk.Label(title_container, text=title_text, font=("Helvetica", 18, "bold"), bg="white").pack()
    if sub_text: tk.Label(title_container, text=sub_text, font=("Helvetica", 10), bg="white", fg="#555").pack()
    if right_btn:
        btn = tk.Button(header_frame, text=right_btn['text'], command=right_btn['cmd'], bg="#e0e0e0", font=("Arial", 9, "bold"), relief="flat", padx=10)
        btn.grid(row=0, column=2, sticky="e")

# --- PAGE ASSEMBLING ---
create_header(page_camera, "SAVES AI", None, left_btn={'text': "Shutdown", 'cmd': close_application, 'color': 'red'}, right_btn={'text': "Current Session Logs ➜", 'cmd': lambda: show_frame(page_logs)})
video_frame_container = tk.Frame(page_camera, bg="#ff4d4d", padx=2, pady=2)
video_frame_container.pack(fill="both", expand=True, padx=60, pady=5)
video_frame_container.pack_propagate(False)
video_label = tk.Label(video_frame_container, bg="black")
video_label.pack(fill="both", expand=True)

status_frame = tk.Frame(page_camera, bg="white")
status_frame.pack(fill="x", side="bottom", pady=5)
lbl_status = tk.Label(status_frame, text="SCANNING", font=("Arial", 11, "bold"), bg="white")
lbl_status.pack(side="left", padx=60)
lbl_dist = tk.Label(status_frame, text="DIST: -- cm", font=("Arial", 11), bg="white")
lbl_dist.pack(side="right", padx=60)

cols = ("Time", "Plate", "Name", "Status", "Latency", "Performance Metrics")
def create_treeview(parent):
    frame = tk.Frame(parent, bg="white")
    frame.pack(fill="both", expand=True, padx=60, pady=5)
    tree = ttk.Treeview(frame, columns=cols, show="headings", style="Treeview")
    scrollbar = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side="right", fill="y"); tree.pack(side="left", fill="both", expand=True)
    for col in cols:
        tree.heading(col, text=col)
        tree.column(col, anchor="center", width=90)
    tree.tag_configure("authorized", foreground="green"); tree.tag_configure("unauthorized", foreground="red")
    return tree

create_header(page_logs, "SAVES AI", "current session", left_btn={'text': "← camera", 'cmd': lambda: show_frame(page_camera)}, right_btn={'text': "past history ➜", 'cmd': lambda: show_frame(page_history)})
tree = create_treeview(page_logs)
create_header(page_history, "SAVES AI", "past history", left_btn={'text': "← camera", 'cmd': lambda: show_frame(page_camera)}, right_btn={'text': "current ➜", 'cmd': lambda: show_frame(page_logs)})
tree_history = create_treeview(page_history)

# ==========================================
#                SYSTEM LOGIC
# ==========================================
logged_vehicles = {} 
scan_buffer = deque(maxlen=SCAN_BUFFER_LEN)
dist_history = deque(maxlen=DISTANCE_BUFFER_LEN) # [NEW]
first_sight_times = {} 
detection_start_time, last_plate_seen_time = None, 0 
is_gate_busy, vehicle_confirmed, vehicle_entry_start_time = False, False, None
system_state = "SCANNING"

try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except: authorized_plates, auth_df = [], pd.DataFrame(columns=["Plate", "Name", "Faculty"])

reader = easyocr.Reader(['en'], gpu=False) 
model = YOLO('./best_ncnn_model') 

def set_status(status_text, color="black"):
    global system_state
    system_state = status_text
    lbl_status.config(text=status_text, fg=color)

def log_to_gui_and_csv(plate, name, faculty, status, latency, det_time, ocr_time):
    ts_l = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ts_s = datetime.now().strftime("%H:%M:%S")
    perf_display = f"Det: {det_time:.0f}ms | OCR: {ocr_time:.0f}ms"
    tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
    tree.insert("", 0, values=(ts_s, plate, name, status, f"{latency:.2f}s", perf_display), tags=(tag,))
    
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([plate, name, faculty, status, ts_l, f"{latency:.4f}", f"{det_time:.2f}", f"{ocr_time:.2f}"])
    
    if status == "AUTHORIZED": trigger_gate_sequence()
    else:
        set_status("UNAUTHORIZED DETECTED", "red")
        buzzer.beep(on_time=0.1, off_time=0.05, n=4)
        root.after(3000, lambda: set_status("SCANNING", "black") if not is_gate_busy else None)

def trigger_gate_sequence():
    global is_gate_busy, vehicle_confirmed, vehicle_entry_start_time
    if is_gate_busy and system_state == "POST-ENTRY SCAN":
        set_status("NEXT VEHICLE", "green")
        vehicle_confirmed = False; vehicle_entry_start_time = None
        return
    if is_gate_busy: return 
    is_gate_busy = True
    set_status("AUTHORIZED: OPENING", "green")
    gate_p1.on(); gate_p2.off(); led_open.on(); led_close.off()
    root.after(GATE_ACTION_TIME, lambda: root.after(100, smart_gate_check))

# ==========================================
#           [NEW] ULTRASONIC LOGIC
# ==========================================
def get_filtered_distance():
    """Removes noise spikes using a median filter across the last 5 readings."""
    try:
        val = sensor.distance * 100
        if val <= 2.0 or val >= 180.0: return None # Discard obvious errors
        dist_history.append(val)
        if len(dist_history) < DISTANCE_BUFFER_LEN: return val
        return sorted(list(dist_history))[len(dist_history)//2] # Return Median
    except: return None

def smart_gate_check():
    global vehicle_entry_start_time, vehicle_confirmed, system_state
    if system_state in ["POST-ENTRY SCAN", "CLOSING GATE"]: return

    dist_cm = get_filtered_distance()
    
    if dist_cm is not None:
        # Display Distance
        if dist_cm >= 140: lbl_dist.config(text="DIST: CLEAR", fg="black")
        else: lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")

        if is_gate_busy:
            # Step 1: Confirm Vehicle Arrival (Consistent signal)
            if not vehicle_confirmed:
                if dist_cm < SAFETY_DISTANCE_CM:
                    if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
                    if (time.time() - vehicle_entry_start_time) >= ENTRY_CONFIRM_TIME:
                        vehicle_confirmed = True
                else: vehicle_entry_start_time = None

            # Step 2: Confirm Vehicle Departure (Hysteresis applied)
            elif vehicle_confirmed and dist_cm > (SAFETY_DISTANCE_CM + HYSTERESIS_OFFSET):
                start_post_entry_wait()
                return

    root.after(SENSOR_POLL_RATE, smart_gate_check)

def start_post_entry_wait():
    set_status("WAITING...", "orange")
    root.after(int(POST_ENTRY_DELAY * 1000), lambda: init_post_entry_scan())

def init_post_entry_scan():
    global system_state, last_plate_seen_time
    system_state = "POST-ENTRY SCAN"; last_plate_seen_time = time.time() 
    monitor_post_entry_timer()

def monitor_post_entry_timer():
    global system_state
    if system_state != "POST-ENTRY SCAN": return
    elapsed = time.time() - last_plate_seen_time
    if elapsed >= EXIT_SCAN_COOLDOWN:
        set_status("CLOSING GATE", "orange")
        gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
        root.after(GATE_ACTION_TIME, reset_gate_system)
    else:
        lbl_status.config(text=f"CLOSING IN: {max(0, EXIT_SCAN_COOLDOWN - elapsed):.1f}s")
        root.after(100, monitor_post_entry_timer)

def reset_gate_system():
    global is_gate_busy, vehicle_confirmed, vehicle_entry_start_time
    is_gate_busy = vehicle_confirmed = False
    vehicle_entry_start_time = None
    set_status("SCANNING", "black")

def update_frame():
    global detection_start_time, last_plate_seen_time
    try: frame = picam2.capture_array() if picam2 else np.zeros((540, 960, 3), dtype=np.uint8)
    except: root.after(100, update_frame); return

    h, w, _ = frame.shape
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), ROI_COLOR, 3) 
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    should_detect = (not is_gate_busy) or (system_state == "POST-ENTRY SCAN")
    if should_detect and system_state != "CLOSING GATE":
        t0 = time.perf_counter()
        results = model(roi_crop, verbose=False, conf=0.4) 
        t_detect = (time.perf_counter() - t0) * 1000
        detected_box = False; current_time = time.time()

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected_box = True; last_plate_seen_time = current_time 
                if detection_start_time is None: detection_start_time = current_time
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 3)
                    if (current_time - detection_start_time) > GRACE_PERIOD:
                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            t_ocr = time.perf_counter()
                            ocr_res = reader.readtext(cv2.cvtColor(p_crop, cv2.COLOR_RGB2GRAY), detail=0)
                            clean_text = "".join([c for c in "".join(ocr_res).upper() if c.isalnum()])
                            t_ocr_ms = (time.perf_counter() - t_ocr) * 1000
                            if len(clean_text) > 3: 
                                scan_buffer.append(clean_text)
                                most_common, freq = Counter(scan_buffer).most_common(1)[0]
                                if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                    latency = (t_detect + t_ocr_ms) / 1000.0
                                    status = "AUTHORIZED" if most_common in authorized_plates else "UNAUTHORIZED"
                                    name = auth_df[auth_df['Plate'] == most_common].iloc[0]['Name'] if status == "AUTHORIZED" else "Unknown"
                                    log_to_gui_and_csv(most_common, name, "Visitor", status, latency, t_detect, t_ocr_ms)
                                    logged_vehicles[most_common] = current_time; scan_buffer.clear()

        if not detected_box and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear()

    # Resizing
    win_w, win_h = video_frame_container.winfo_width(), video_frame_container.winfo_height()
    if win_w > 10:
        scale = min(win_w / w, win_h / h)
        img = Image.fromarray(cv2.resize(frame, (int(w * scale), int(h * scale))))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk; video_label.configure(image=imgtk)

    root.after(10, update_frame)

show_frame(page_camera)
root.after(500, smart_gate_check)
root.after(500, update_frame)
root.mainloop()
