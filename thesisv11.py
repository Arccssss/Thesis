import cv2
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
import sys
from datetime import datetime
from collections import Counter, deque
import csv
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from gpiozero import OutputDevice, DistanceSensor, Buzzer, LED
from picamera2 import Picamera2

#uses yolov11 plate detector + yolov11 plate reader

# ==========================================
#           CONFIGURATION
# ==========================================
AUTH_FILE = 'Database/authorized.csv'
LOG_FILE = 'Database/access_logs.csv'
LOG_COOLDOWN = 5
SCAN_BUFFER_LEN = 35
CONFIDENCE_THRESHOLD = 10

# ROI & Vision
ROI_SCALE_W, ROI_SCALE_H = 0.7, 0.5
ROI_COLOR = (0, 255, 0)
GRACE_PERIOD = 0.5

# Hardware Timings
GATE_ACTION_TIME = 3000   # Time for gate to physically close (ms)
GATE_SAFETY_TIMEOUT = 20.0 # Max time to wait for entry before force closing
EXIT_SCAN_COOLDOWN = 5.0  # Post-entry wait time
SENSOR_POLL_RATE = 100    # Check sensor every 100ms

# Sensor Tuning
SAFETY_DISTANCE_CM = 30
ENTRY_CONFIRM_TARGET = 0.5 # Seconds needed below 100cm to confirm vehicle
ABSENCE_RESET_TIME = 10.0

# --- HARDWARE PINS ---
GATE_PIN_1 = 17
GATE_PIN_2 = 27
US_TRIG_PIN = 23
US_ECHO_PIN = 24
BUZZER_PIN = 22
LED_OPEN_PIN = 5   # GREEN
LED_CLOSE_PIN = 6  # RED

# ==========================================
#           FSM STATE CONSTANTS
# ==========================================
STATE_IDLE = "IDLE"
STATE_OPENING = "OPENING"
STATE_WAITING_ENTRY = "WAITING_ENTRY"
STATE_POST_ENTRY = "POST_ENTRY"
STATE_CLOSING = "CLOSING"

# ==========================================
#           HARDWARE INIT
# ==========================================
try:
    gate_p1 = OutputDevice(GATE_PIN_1, active_high=True, initial_value=False)
    gate_p2 = OutputDevice(GATE_PIN_2, active_high=True, initial_value=True)
    sensor = DistanceSensor(echo=US_ECHO_PIN, trigger=US_TRIG_PIN, max_distance=1.0, queue_len=3)
    buzzer = Buzzer(BUZZER_PIN)
    
    led_green_auth = LED(LED_OPEN_PIN)
    led_red_unauth = LED(LED_CLOSE_PIN)
    
    # Initial State
    led_green_auth.off()
    led_red_unauth.on()
    
    picam2 = Picamera2()
    config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (1920, 1080)})
    picam2.configure(config)
    picam2.set_controls({"AfMode": 2, "AwbMode": 3})
    picam2.start()
    print(f"✅ Hardware Initialized")
    
except Exception as e:
    print(f"⚠️ HARDWARE ERROR: {e}")
    class DummyDev: 
        def on(self): pass
        def off(self): pass
        def beep(self, *args, **kwargs): pass
        def blink(self, *args, **kwargs): pass
        def close(self): pass
        @property
        def value(self): return 0
        @property
        def distance(self): return 1.5 
    gate_p1 = gate_p2 = sensor = buzzer = DummyDev()
    led_green_auth = led_red_unauth = DummyDev()
    picam2 = None

# ==========================================
#           GUI SETUP
# ==========================================
root = tk.Tk()
root.title("SAVES AI Control System")
root.configure(bg="white")
root.attributes('-fullscreen', True)
root.bind("<Escape>", lambda event: root.attributes("-fullscreen", True))

def close_application():
    if picam2: picam2.stop()
    gate_p1.close(); gate_p2.on()
    led_green_auth.off(); led_red_unauth.off()
    root.destroy(); sys.exit(0)

def minimize_window():
    root.iconify()

style = ttk.Style()
style.theme_use("clam")
style.configure("Treeview.Heading", background="#cccccc", foreground="white", font=("Arial", 14, "bold"), relief="flat")
style.configure("Treeview", font=("Arial", 12), rowheight=35)

container = tk.Frame(root, bg="white")
container.pack(fill="both", expand=True)

page_camera = tk.Frame(container, bg="white")
page_logs = tk.Frame(container, bg="white")
page_history = tk.Frame(container, bg="white")

for frame in (page_camera, page_logs, page_history):
    frame.grid(row=0, column=0, sticky="nsew")

def show_frame(frame):
    frame.tkraise()
    try: btn_minimize.lift()
    except: pass

def create_header(parent, title_text, sub_text=None, left_btn=None, right_btn=None):
    header_frame = tk.Frame(parent, bg="white", pady=15)
    header_frame.pack(fill="x", padx=15)
    header_frame.columnconfigure(0, weight=1)
    header_frame.columnconfigure(1, weight=4)
    header_frame.columnconfigure(2, weight=1)

    btn_font = ("Arial", 12, "bold")
    
    if left_btn:
        bg_c = "#ffcccc" if "Shutdown" in left_btn['text'] else "#e0e0e0"
        fg_c = "red" if "Shutdown" in left_btn['text'] else "#555"
        tk.Button(header_frame, text=left_btn['text'], command=left_btn['cmd'],
                  bg=bg_c, fg=fg_c, font=btn_font, relief="flat", padx=15, pady=5).grid(row=0, column=0, sticky="w")

        if "Shutdown" in left_btn['text']:
             tk.Button(header_frame, text="Reset", command=lambda: transition_to(STATE_IDLE),
                       bg="#e0e0e0", fg="#555", font=btn_font, relief="flat", padx=15, pady=5).grid(row=0, column=0, sticky="w", padx=(120, 0))

    title_c = tk.Frame(header_frame, bg="white")
    title_c.grid(row=0, column=1)
    tk.Label(title_c, text=title_text, font=("Helvetica", 24, "bold"), bg="white").pack()
    if sub_text: tk.Label(title_c, text=sub_text, font=("Helvetica", 14), bg="white", fg="#555").pack()

    if right_btn:
        tk.Button(header_frame, text=right_btn['text'], command=right_btn['cmd'],
                  bg="#e0e0e0", fg="#555", font=btn_font, relief="flat", padx=15, pady=5).grid(row=0, column=2, sticky="e")

# --- CAMERA PAGE UI ---
create_header(page_camera, "SAVES AI", None,
              left_btn={'text': "Shutdown", 'cmd': close_application},
              right_btn={'text': "Logs ➜", 'cmd': lambda: show_frame(page_logs)})

video_frame_container = tk.Frame(page_camera, bg="black")
video_frame_container.pack(fill="both", expand=True, padx=0, pady=0)
video_frame_container.pack_propagate(False)

video_label = tk.Label(video_frame_container, bg="black")
video_label.pack(fill="both", expand=True)

status_frame = tk.Frame(page_camera, bg="white")
status_frame.pack(fill="x", side="bottom", pady=15) 

lbl_status = tk.Label(status_frame, text="IDLE", font=("Arial", 16, "bold"), bg="white", fg="black")
lbl_status.pack(side="left", padx=20)

lbl_dist = tk.Label(status_frame, text="DIST: -- cm", font=("Arial", 16), bg="white", fg="black")
lbl_dist.pack(side="right", padx=20)

# --- LOGS UI ---
cols = ("Time", "Plate", "Name", "Status", "Latency", "Metrics")
def create_treeview(parent):
    f = tk.Frame(parent, bg="white")
    f.pack(fill="both", expand=True, padx=10, pady=5)
    t = ttk.Treeview(f, columns=cols, show="headings")
    sc = ttk.Scrollbar(f, orient="vertical", command=t.yview)
    t.configure(yscrollcommand=sc.set)
    sc.pack(side="right", fill="y")
    t.pack(side="left", fill="both", expand=True)
    for c in cols: t.heading(c, text=c); t.column(c, anchor="center", width=110)
    t.tag_configure("authorized", foreground="green")
    t.tag_configure("unauthorized", foreground="red")
    return t

create_header(page_logs, "SAVES AI", "current session",
              left_btn={'text': "← camera", 'cmd': lambda: show_frame(page_camera)},
              right_btn={'text': "past logs ➜", 'cmd': lambda: [load_history(), show_frame(page_history)]})
tree = create_treeview(page_logs)

create_header(page_history, "SAVES AI", "past session",
              left_btn={'text': "← camera", 'cmd': lambda: show_frame(page_camera)},
              right_btn={'text': "current logs ➜", 'cmd': lambda: show_frame(page_logs)})
tree_history = create_treeview(page_history)

btn_minimize = tk.Button(root, text=" — ", command=minimize_window, bg="#f0f0f0", font=("Arial", 14, "bold"), relief="flat")
btn_minimize.place(relx=1.0, x=-10, y=10, anchor="ne")

# ==========================================
#           LOGIC & FSM VARIABLES
# ==========================================
logged_vehicles = {}
scan_buffer = deque(maxlen=SCAN_BUFFER_LEN)
first_sight_times = {}
detection_start_time = None
last_plate_seen_time = 0

# FSM STATE VARIABLES
current_state = STATE_IDLE
state_start_time = 0
accumulated_presence = 0.0

try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except:
    authorized_plates = []
    auth_df = pd.DataFrame(columns=["Plate", "Name", "Faculty"])

# --- MODELS INITIALIZATION ---
# Model 1: Plate Detector (Kept as is)
model_plate = YOLO('./models/plate_small_boundingBox_ncnn_model') 

# Model 2: Character Recognition (New Custom YOLO)
# IMPORTANT: Update this path to your actual character model file
# If you convert this to ncnn, point to the folder instead of .pt
model_char = YOLO('./models/plate_reader_v11_ncnn_model')

# ==========================================
#           FSM TRANSITION LOGIC
# ==========================================
def transition_to(new_state):
    global current_state, state_start_time, accumulated_presence
    
    print(f"🔀 FSM TRANSITION: {current_state} -> {new_state}")
    current_state = new_state
    state_start_time = time.time()
    accumulated_presence = 0.0 
    
    lbl_status.config(text=current_state, fg="black")

    # --- STATE: IDLE ---
    if new_state == STATE_IDLE:
        gate_p1.off(); gate_p2.on() 
        led_green_auth.off()
        led_red_unauth.on()
        lbl_status.config(fg="black")
        
        scan_buffer.clear()
        first_sight_times.clear()

    # --- STATE: OPENING ---
    elif new_state == STATE_OPENING:
        gate_p1.on(); gate_p2.off() 
        led_green_auth.on()
        led_red_unauth.off()
        buzzer.beep(on_time=0.1, off_time=0.1, n=2)
        lbl_status.config(fg="green")

    # --- STATE: WAITING_ENTRY ---
    elif new_state == STATE_WAITING_ENTRY:
        led_green_auth.on()
        led_red_unauth.off()
        lbl_status.config(text="WAITING ENTRY...", fg="green")

    # --- STATE: POST_ENTRY ---
    elif new_state == STATE_POST_ENTRY:
        led_green_auth.off()
        led_red_unauth.on()
        lbl_status.config(text="POST-ENTRY SCAN", fg="orange")

    # --- STATE: CLOSING ---
    elif new_state == STATE_CLOSING:
        gate_p1.off(); gate_p2.on()
        led_green_auth.off()
        led_red_unauth.on()
        lbl_status.config(text="CLOSING...", fg="red")
        
        root.after(GATE_ACTION_TIME, lambda: transition_to(STATE_IDLE))

# ==========================================
#           FSM UPDATE LOOP
# ==========================================
def fsm_update_loop():
    global accumulated_presence
    
    # 1. Update Distance UI
    try:
        dist_cm = sensor.distance * 100
        if dist_cm >= 98: 
            lbl_dist.config(text="DIST: Clear (>1m)", fg="black")
        else:
            lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", 
                            fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")
    except: 
        dist_cm = 999

    # 2. State Specific Logic
    elapsed = time.time() - state_start_time
    
    if current_state == STATE_OPENING:
        transition_to(STATE_WAITING_ENTRY)
        
    elif current_state == STATE_WAITING_ENTRY:
        if elapsed > GATE_SAFETY_TIMEOUT:
            print("⚠️ Entry Timeout")
            transition_to(STATE_CLOSING)
            return

        if dist_cm < SAFETY_DISTANCE_CM:
            accumulated_presence += (SENSOR_POLL_RATE / 1000.0)
            if accumulated_presence >= ENTRY_CONFIRM_TARGET:
                transition_to(STATE_POST_ENTRY)
        else:
            accumulated_presence = 0.0

    elif current_state == STATE_POST_ENTRY:
        remaining = max(0, EXIT_SCAN_COOLDOWN - elapsed)
        lbl_status.config(text=f"POST-SCAN: {remaining:.1f}s")
        
        if elapsed >= EXIT_SCAN_COOLDOWN:
            transition_to(STATE_CLOSING)

    root.after(SENSOR_POLL_RATE, fsm_update_loop)

# ==========================================
#           VISION & LOGGING
# ==========================================
def log_event(plate, name, faculty, status, det, ocr, total_lat):
    # 1. Capture Time
    now = datetime.now()
    display_time = now.strftime("%H:%M:%S")      
    csv_timestamp = now.strftime("%Y-%m-%d %H:%M:%S") 

    # 2. Save to CSV
    try:
        file_exists = os.path.isfile(LOG_FILE)
        with open(LOG_FILE, 'a', newline='', encoding='utf-8') as f:
            headers = ["Plate", "Name", "Faculty", "Status", "Timestamp", "Latency", "Det", "OCR"]
            writer = csv.DictWriter(f, fieldnames=headers)
            if not file_exists: writer.writeheader()
            writer.writerow({
                "Plate": plate, 
                "Name": name, 
                "Faculty": faculty, 
                "Status": status,
                "Timestamp": csv_timestamp, 
                "Latency": f"{total_lat:.2f}",
                "Det": f"{det:.2f}", 
                "OCR": f"{ocr:.2f}"
            })
    except Exception as e: print(f"❌ Log Error: {e}")

    # 3. Update UI
    try:
        metrics = f"Det:{det:.0f} | OCR:{ocr:.0f}"
        tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
        
        tree.insert("", 0, values=(
            display_time, plate, name, status, f"{total_lat:.0f} ms", metrics
        ), tags=(tag,))

        tree_history.insert("", 0, values=(
            display_time, plate, name, status, f"{total_lat:.0f} ms", metrics
        ), tags=(tag,))
        
    except Exception as e:
        print(f"UI Update Error: {e}")

    # 4. State Machine Trigger
    if status == "AUTHORIZED":
        if current_state == STATE_IDLE or current_state == STATE_POST_ENTRY:
             transition_to(STATE_OPENING)
    else:
        if current_state == STATE_IDLE:
            print(f"⛔ Unauthorized: {plate}")
            led_red_unauth.blink(on_time=0.1, off_time=0.1, n=3, background=True)
            buzzer.beep(on_time=0.1, off_time=0.1, n=3, background=True)
            root.after(1000, lambda: led_red_unauth.on() if current_state == STATE_IDLE else None)

def trigger_authorized_event():
    if current_state == STATE_IDLE:
        transition_to(STATE_OPENING)
    elif current_state == STATE_POST_ENTRY:
        print("🚀 Next Vehicle Detected during Post-Scan!")
        transition_to(STATE_OPENING) 

def load_history():
    for item in tree_history.get_children(): tree_history.delete(item)
    if not os.path.exists(LOG_FILE): return
    try:
        with open(LOG_FILE, 'r', newline='', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reversed(list(reader)):
                full_ts = row.get("Timestamp", "")
                try: time_only = datetime.strptime(full_ts, "%Y-%m-%d %H:%M:%S").strftime("%H:%M:%S")
                except: time_only = full_ts
                
                plate = row.get("Plate", "Unknown")
                name = row.get("Name", "Unknown")
                status = row.get("Status", "---")
                
                lat_total = float(row.get("Latency", 0))
                det = float(row.get("Det", 0))
                ocr = float(row.get("OCR", 0))
                
                metrics = f"Det:{det:.0f} | OCR:{ocr:.0f}"
                tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
                
                tree_history.insert("", "end", values=(time_only, plate, name, status, f"{lat_total:.0f} ms", metrics), tags=(tag,))
    except Exception as e: print(f"Error loading history: {e}")

def update_frame():
    global detection_start_time, last_plate_seen_time
    
    if picam2 is None: return root.after(100, update_frame)

    # 1. Start the clock for TOTAL latency
    t_start_process = time.perf_counter()

    try: frame = picam2.capture_array()
    except: return root.after(100, update_frame)

    frame_rgb = frame
    h, w, _ = frame_rgb.shape
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    
    cv2.rectangle(frame_rgb, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), ROI_COLOR, 3)
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w] 

    should_detect = (current_state == STATE_IDLE) or (current_state == STATE_POST_ENTRY)
    
    if should_detect:
        # Detection Timer
        t0_det = time.perf_counter()
        results = model_plate(roi_crop, verbose=True, conf=0.6)
        t_detect = (time.perf_counter() - t0_det) * 1000 # ms
        
        current_time = time.time()
        detected = False
        
        for result in results:
            if len(result.boxes) > 0:
                detected = True
                last_plate_seen_time = current_time
                if detection_start_time is None: detection_start_time = current_time
                
                if (current_time - detection_start_time) > GRACE_PERIOD:
                    for box in result.boxes.xyxy.cpu().numpy():
                        x1, y1, x2, y2 = map(int, box)
                        cv2.rectangle(frame_rgb, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (255, 0, 0), 3)

                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            # --- CUSTOM YOLO OCR IMPLEMENTATION ---
                            t0_ocr = time.perf_counter()
                            
                            # 1. Predict Characters using the second model
                            # agnostic_nms=True prevents different classes from overlapping
                            results_char = model_char.predict(p_crop, conf=0.4, verbose=True, iou=0.5, agnostic_nms=True)
                            result_c = results_char[0]
                            
                            detected_chars = []
                            
                            # 2. Extract and Sort
                            if len(result_c.boxes) > 0:
                                for char_box in result_c.boxes:
                                    # Get X coordinate for sorting
                                    c_x1 = float(char_box.xyxy[0][0])
                                    # Get Class Name
                                    cls_id = int(char_box.cls[0])
                                    character = model_char.names[cls_id]
                                    
                                    detected_chars.append((c_x1, character))
                                
                                # CRITICAL: Sort Left to Right based on X coordinate
                                detected_chars.sort(key=lambda x: x[0])
                                
                                # Join to string
                                clean = "".join([c for _, c in detected_chars])
                            else:
                                clean = ""

                            t_ocr = (time.perf_counter() - t0_ocr) * 1000 # ms
                            
                            # 3. Check Logic
                            if len(clean) > 3:
                                scan_buffer.append(clean)
                                most_common, _ = Counter(scan_buffer).most_common(1)[0]
                                
                                if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                        # --- CALCULATE TOTAL LATENCY ---
                                        total_latency = (time.perf_counter() - t_start_process) * 1000
                                        
                                        if t_detect == 0: t_detect = 10 
                                        if t_ocr == 0: t_ocr = 10 
                                        
                                        if most_common in authorized_plates:
                                            # Authorized Logic
                                            row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                            user_faculty = row.get('Faculty', 'N/A')
                                            if pd.isna(user_faculty): user_faculty = "N/A"

                                            log_event(most_common, row['Name'], user_faculty, "AUTHORIZED", t_detect, t_ocr, total_latency)
                                        else:
                                            # Unauthorized Logic
                                            log_event(most_common, "Unknown", "N/A", "UNAUTHORIZED", t_detect, t_ocr, total_latency)
                                        
                                        logged_vehicles[most_common] = current_time
                                        scan_buffer.clear()

        if not detected and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear()

    win_w = video_frame_container.winfo_width()
    win_h = video_frame_container.winfo_height()
    if win_w > 10 and win_h > 10:
        scale = min(win_w / w, win_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        img_resized = cv2.resize(frame_rgb, (new_w, new_h))
        video_label.imgtk = ImageTk.PhotoImage(image=Image.fromarray(img_resized))
        video_label.configure(image=video_label.imgtk)

    root.after(10, update_frame)

# ==========================================
#           STARTUP
# ==========================================
show_frame(page_camera)
root.update_idletasks()
root.after(500, fsm_update_loop)
root.after(500, update_frame)

try:
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    close_application()
