import cv2
import easyocr
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
from datetime import datetime
from collections import Counter, deque
import csv 
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
from gpiozero import OutputDevice, DistanceSensor, Buzzer, LED 
from picamera2 import Picamera2
import threading

# ==========================================
#           CONFIGURATION
# ==========================================
AUTH_FILE = 'Database/authorized.csv'
LOG_FILE = 'Database/access_logs.csv'
LOG_COOLDOWN = 5  
SCAN_BUFFER_LEN = 9  
CONFIDENCE_THRESHOLD = 4

ROI_SCALE_W, ROI_SCALE_H = 0.7, 0.5 
GRACE_PERIOD = 0.3      
GATE_ACTION_TIME = 3000 
ABSENCE_RESET_TIME = 2.5 

GATE_CLOSE_GRACE_TIMEOUT = 5.0  
POST_ENTRY_SCAN_DELAY = 0.5     

SAFETY_DISTANCE_CM = 50 
ENTRY_CONFIRM_TIME = 0.5 
SENSOR_POLL_RATE = 100    

# --- HARDWARE PINS ---
GATE_PIN_1 = 17 
GATE_PIN_2 = 27 
US_TRIG_PIN = 23 
US_ECHO_PIN = 24 
BUZZER_PIN = 22
LED_OPEN_PIN = 5   
LED_CLOSE_PIN = 6  

# ==========================================
#           HARDWARE INITIALIZATION
# ==========================================
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
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (1280, 720)})
    picam2.configure(config)
    picam2.set_controls({"AfMode": 2, "AwbMode": 3}) 
    picam2.start()
    print("✅ Hardware Initialized")
except Exception as e:
    print(f"⚠️ HARDWARE ERROR: {e}")
    class DummyDev:
        def on(self): pass
        def off(self): pass
        def blink(self, *args, **kwargs): pass
        def beep(self, *args, **kwargs): pass
        def close(self): pass
        @property
        def distance(self): return 1.5
    gate_p1 = gate_p2 = sensor = buzzer = led_open = led_close = DummyDev()
    picam2 = None

# ==========================================
#           LOGIC & HELPERS
# ==========================================
logged_vehicles = {} 
scan_buffer = deque(maxlen=SCAN_BUFFER_LEN)
first_sight_times = {} 
detection_start_time = None
last_plate_seen_time = 0 
is_gate_busy = False
system_state = "SCANNING" 
vehicle_entry_start_time = None 
vehicle_confirmed = False
plate_absence_start_time = None 
scan_resume_time = 0 
current_dist_global = 150.0

try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except:
    authorized_plates = []
    auth_df = pd.DataFrame(columns=["Plate", "Name", "Faculty"])

reader = easyocr.Reader(['en'], gpu=False) 
model = YOLO('./best_ncnn_model') 

# ==========================================
#           CORE FUNCTIONS
# ==========================================

def load_history_data():
    """Reads the CSV file and populates the tree_history widget."""
    for i in tree_history.get_children(): tree_history.delete(i)
    if not os.path.isfile(LOG_FILE): return
    try:
        with open(LOG_FILE, 'r') as f:
            reader_csv = csv.DictReader(f)
            # Reversing list so newest logs are at the top
            for row in reversed(list(reader_csv)):
                ts = row.get('Timestamp', '')
                try: 
                    short_ts = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S").strftime("%H:%M:%S")
                except: 
                    short_ts = ts
                tag = "authorized" if "AUTHORIZED" in row.get('Status', '') else "unauthorized"
                tree_history.insert("", "end", values=(short_ts, row.get('Plate'), row.get('Name'), row.get('Status'), row.get('Latency') + "s"), tags=(tag,))
    except Exception as e:
        print(f"History Load Error: {e}")

def set_status(status_text, color="white"):
    global system_state
    system_state = status_text
    lbl_status.config(text=status_text, fg=color)

def authorized_feedback():
    buzzer.beep(on_time=0.1, off_time=0.05, n=2)
    led_open.blink(on_time=0.2, off_time=0.2, n=5)

# ==========================================
#           GUI SETUP
# ==========================================
root = tk.Tk()
root.title("RPi 5 Smart Access Control")
root.geometry("800x600")

notebook = ttk.Notebook(root)
notebook.pack(fill='both', expand=True)

tab_camera = tk.Frame(notebook, bg="#202020")
tab_logs = tk.Frame(notebook, bg="#f0f0f0")
tab_history = tk.Frame(notebook, bg="#e0e0e0")

notebook.add(tab_camera, text=" LIVE CAMERA ")
notebook.add(tab_logs, text=" CURRENT SESSION ")
notebook.add(tab_history, text=" PAST HISTORY ")

video_frame_container = tk.Frame(tab_camera, bg="black")
video_frame_container.pack(fill="both", expand=True, padx=5, pady=5)
video_label = tk.Label(video_frame_container, bg="black")
video_label.pack(expand=True)

control_frame = tk.Frame(tab_camera, bg="#333", height=80)
control_frame.pack(fill="x", side="bottom")

lbl_status = tk.Label(control_frame, text="SCANNING", font=("Arial", 12, "bold"), bg="#333", fg="white", width=20)
lbl_status.pack(side="left", padx=10, pady=10)

btn_manual_reset = tk.Button(control_frame, text="RESET SYSTEM", bg="#d32f2f", fg="white", font=("Arial", 10, "bold"), command=lambda: execute_close_action())
btn_manual_reset.pack(side="left", padx=10)

lbl_dist_gui = tk.Label(control_frame, text="DIST: -- cm", font=("Arial", 12), bg="#333", fg="cyan")
lbl_dist_gui.pack(side="right", padx=10)

# Session Logs Tab
cols = ("time", "plate", "owner", "status", "latency")
tree = ttk.Treeview(tab_logs, columns=cols, show="headings")
for c in cols: tree.heading(c, text=c.capitalize()); tree.column(c, width=100, anchor="center")
tree.pack(fill="both", expand=True)
tree.tag_configure("authorized", foreground="green")
tree.tag_configure("unauthorized", foreground="red")

# History Tab
tree_history = ttk.Treeview(tab_history, columns=cols, show="headings")
for c in cols: tree_history.heading(c, text=c.capitalize()); tree_history.column(c, width=100, anchor="center")
tree_history.pack(fill="both", expand=True)
tree_history.tag_configure("authorized", foreground="green")
tree_history.tag_configure("unauthorized", foreground="red")

# Manual Refresh Button in History Tab
btn_refresh_hist = tk.Button(tab_history, text="Refresh History From File", command=load_history_data)
btn_refresh_hist.pack(fill="x")

# ==========================================
#           CONTINUED CORE LOGIC
# ==========================================

def log_to_gui_and_csv(plate, name, faculty, status, latency):
    ts_l, ts_s = datetime.now().strftime("%Y-%m-%d %H:%M:%S"), datetime.now().strftime("%H:%M:%S")
    tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
    tree.insert("", 0, values=(ts_s, plate, name, status, f"{latency:.2f}s"), tags=(tag,))
    with open(LOG_FILE, 'a', newline='') as f:
        writer = csv.writer(f)
        if os.path.getsize(LOG_FILE) == 0: writer.writerow(["Plate","Name","Faculty","Status","Timestamp","Latency"])
        writer.writerow([plate, name, faculty, status, ts_l, f"{latency:.4f}"])
    
    # Auto-update history tab
    load_history_data()

    if status == "AUTHORIZED":
        threading.Thread(target=authorized_feedback, daemon=True).start()
        trigger_gate_sequence()
    else:
        set_status("UNAUTHORIZED DETECTED", "red")
        buzzer.beep(on_time=0.1, off_time=0.05, n=4)
        root.after(3000, lambda: set_status("SCANNING", "white") if not is_gate_busy else None)

def reset_gate_system():
    global is_gate_busy, vehicle_entry_start_time, vehicle_confirmed, plate_absence_start_time, scan_resume_time
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    is_gate_busy = vehicle_confirmed = False
    vehicle_entry_start_time = plate_absence_start_time = None
    scan_resume_time = 0
    set_status("SCANNING", "white")

def execute_close_action():
    set_status("CLOSING GATE", "orange")
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    root.after(GATE_ACTION_TIME, reset_gate_system)

def trigger_gate_sequence():
    global is_gate_busy, vehicle_confirmed, plate_absence_start_time, scan_resume_time
    if is_gate_busy: return 
    is_gate_busy = True
    vehicle_confirmed = False
    plate_absence_start_time = None
    scan_resume_time = 0
    set_status("OPENING GATE", "green")
    gate_p1.on(); gate_p2.off(); led_open.on(); led_close.off()

def smart_gate_check():
    global vehicle_entry_start_time, vehicle_confirmed, plate_absence_start_time, scan_resume_time, current_dist_global
    try:
        dist_cm = sensor.distance * 100
        current_dist_global = dist_cm
        lbl_dist_gui.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "cyan")

        if is_gate_busy and system_state != "CLOSING GATE":
            if not vehicle_confirmed:
                if dist_cm < SAFETY_DISTANCE_CM:
                    if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
                    if (time.time() - vehicle_entry_start_time) >= ENTRY_CONFIRM_TIME: vehicle_confirmed = True
                else: vehicle_entry_start_time = None
            elif dist_cm > SAFETY_DISTANCE_CM:
                curr_t = time.time()
                if scan_resume_time == 0: scan_resume_time = curr_t + POST_ENTRY_SCAN_DELAY
                if curr_t >= scan_resume_time:
                    if (curr_t - last_plate_seen_time) > 1.2:
                        if plate_absence_start_time is None: plate_absence_start_time = curr_t
                        if (curr_t - plate_absence_start_time) >= GATE_CLOSE_GRACE_TIMEOUT:
                            execute_close_action()
                            return
                    else: plate_absence_start_time = None 
        elif not is_gate_busy:
            if dist_cm < SAFETY_DISTANCE_CM and system_state != "UNAUTHORIZED DETECTED": set_status("DETECTING CAR", "yellow")
            elif dist_cm > SAFETY_DISTANCE_CM and system_state == "DETECTING CAR": set_status("SCANNING", "white")
    except: pass
    root.after(SENSOR_POLL_RATE, smart_gate_check)

def update_frame():
    global last_plate_seen_time, detection_start_time
    if not picam2: return
    frame = picam2.capture_array()
    h, w, _ = frame.shape
    curr_t = time.time()

    # On-screen ultrasonic CM display
    cv2.putText(frame, f"STATUS: {system_state}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    d_clr = (0, 0, 255) if current_dist_global < SAFETY_DISTANCE_CM else (0, 255, 0)
    cv2.putText(frame, f"SENSOR: {current_dist_global:.1f} cm", (30, 95), cv2.FONT_HERSHEY_SIMPLEX, 1, d_clr, 2)

    is_waiting_for_car = is_gate_busy and not vehicle_confirmed
    is_in_delay = scan_resume_time > 0 and curr_t < scan_resume_time
    if is_in_delay: cv2.putText(frame, "READYING NEXT SCAN...", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 128, 0), 2)
    elif is_waiting_for_car: cv2.putText(frame, "AWAITING PASSAGE...", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    elif plate_absence_start_time and is_gate_busy:
        rem = max(0, GATE_CLOSE_GRACE_TIMEOUT - (curr_t - plate_absence_start_time))
        cv2.putText(frame, f"CLOSING IN: {rem:.1f}s", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

    # ROI Scaling
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 0), 2)

    # Logic to only scan when not busy with confirmation or delay
    if not (system_state == "CLOSING GATE" or is_waiting_for_car or is_in_delay):
        roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        results = model(roi, verbose=False, conf=0.4)
        det = False
        for r in results:
            boxes = r.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                det = True; last_plate_seen_time = curr_t
                if detection_start_time is None: detection_start_time = curr_t
                for b in boxes:
                    x1, y1, x2, y2 = map(int, b)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 3)
                    if (curr_t - detection_start_time) > GRACE_PERIOD:
                        p_crop = roi[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            res = reader.readtext(cv2.cvtColor(p_crop, cv2.COLOR_RGB2GRAY), detail=0)
                            txt = "".join([c for c in "".join(res).upper() if c.isalnum()])
                            if len(txt) > 3:
                                if txt not in first_sight_times: first_sight_times[txt] = curr_t
                                scan_buffer.append(txt)
                                common, freq = Counter(scan_buffer).most_common(1)[0]
                                if (curr_t - logged_vehicles.get(common, 0)) > LOG_COOLDOWN:
                                    lat = curr_t - first_sight_times.get(common, curr_t)
                                    if common in authorized_plates:
                                        row = auth_df[auth_df['Plate'] == common].iloc[0]
                                        log_to_gui_and_csv(common, row['Name'], row['Faculty'], "AUTHORIZED", lat)
                                    elif freq >= CONFIDENCE_THRESHOLD:
                                        log_to_gui_and_csv(common, "Unknown", "Visitor", "UNAUTHORIZED", lat)
                                    logged_vehicles[common] = curr_t
        if not det and detection_start_time and (curr_t - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear(); first_sight_times.clear()

    # Aspect Ratio Scaler
    try:
        cw, ch = video_frame_container.winfo_width(), video_frame_container.winfo_height()
        if cw > 100 and ch > 100:
            aspect = w / h
            new_h = ch
            new_w = int(new_h * aspect)
            if new_w > cw:
                new_w = cw
                new_h = int(new_w / aspect)
            img = Image.fromarray(cv2.resize(frame, (new_w, new_h)))
            imgtk = ImageTk.PhotoImage(image=img)
            video_label.imgtk = imgtk; video_label.configure(image=imgtk)
    except: pass
    root.after(10, update_frame)

# ==========================================
#           INITIALIZATION & BOOT
# ==========================================
load_history_data() # INITIALIZED HERE
root.after(500, smart_gate_check)
root.after(500, update_frame)
root.mainloop()