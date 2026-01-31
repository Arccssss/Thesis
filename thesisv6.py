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
ROI_COLOR = (0, 255, 0) 
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
    config = picam2.create_preview_configuration(main={"format": "RGB888", "size": (1920, 1080)})
    picam2.configure(config)
    picam2.set_controls({"AfMode": 2, "AwbMode": 3}) 
    picam2.start()
    print(f"✅ Hardware Initialized")
    
except Exception as e:
    print(f"⚠️ HARDWARE ERROR: {e}")
    class DummyDev: 
        def on(self): pass
        def off(self): pass
        def blink(self, on_time=0.1, off_time=0.1, n=1): pass
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
current_dist_global = 0.0

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

video_frame_container = tk.Frame(tab_camera, bg="#202020")
video_frame_container.pack(fill="both", expand=True)
video_label = tk.Label(video_frame_container, bg="black")
video_label.pack(expand=True)

control_frame = tk.Frame(tab_camera, bg="#333")
control_frame.pack(fill="x", side="bottom")

lbl_status = tk.Label(control_frame, text="SCANNING", font=("Arial", 12, "bold"), bg="#333", fg="white", width=20)
lbl_status.pack(side="left", padx=10, pady=10)

btn_manual_reset = tk.Button(control_frame, text="RESET SYSTEM", bg="red", fg="white", font=("Arial", 10, "bold"), command=lambda: execute_close_action())
btn_manual_reset.pack(side="left", padx=10)

lbl_dist = tk.Label(control_frame, text="DIST: -- cm", font=("Arial", 12), bg="#333", fg="cyan")
lbl_dist.pack(side="right", padx=10)

cols_current = ("time", "plate", "owner", "status", "latency")
tree = ttk.Treeview(tab_logs, columns=cols_current, show="headings", height=12)
for col in cols_current: 
    tree.heading(col, text=col.capitalize())
    tree.column(col, width=100, anchor="center")
tree.pack(side="left", fill="both", expand=True)
tree.tag_configure("authorized", foreground="green")
tree.tag_configure("unauthorized", foreground="red")

tree_history = ttk.Treeview(tab_history, columns=cols_current, show="headings", height=12)
for col in cols_current: 
    tree_history.heading(col, text=col.capitalize())
    tree_history.column(col, width=100, anchor="center")
scrollbar = ttk.Scrollbar(tab_history, orient="vertical", command=tree_history.yview)
tree_history.configure(yscrollcommand=scrollbar.set)
scrollbar.pack(side="right", fill="y")
tree_history.pack(side="left", fill="both", expand=True)
tree_history.tag_configure("authorized", foreground="green")
tree_history.tag_configure("unauthorized", foreground="red")

# ==========================================
#           CORE FUNCTIONS
# ==========================================

def set_status(status_text, color="white"):
    global system_state
    system_state = status_text
    lbl_status.config(text=status_text, fg=color)

def load_history_data():
    for i in tree_history.get_children(): tree_history.delete(i)
    if not os.path.isfile(LOG_FILE): return
    try:
        with open(LOG_FILE, 'r') as f:
            reader_csv = csv.DictReader(f)
            for row in reversed(list(reader_csv)):
                ts = row.get('Timestamp', '')
                try: short_ts = datetime.strptime(ts, "%Y-%m-%d %H:%M:%S").strftime("%H:%M:%S")
                except: short_ts = ts
                tag = "authorized" if "AUTHORIZED" in row.get('Status', '') else "unauthorized"
                tree_history.insert("", "end", values=(short_ts, row.get('Plate'), row.get('Name'), row.get('Status'), row.get('Latency') + "s"), tags=(tag,))
    except: pass

def authorized_feedback():
    buzzer.beep(on_time=0.1, off_time=0.05, n=2)
    led_open.blink(on_time=0.2, off_time=0.2, n=5)

def log_to_gui_and_csv(plate, name, faculty, status, latency):
    ts_l = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ts_s = datetime.now().strftime("%H:%M:%S")
    try:
        tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
        tree.insert("", 0, values=(ts_s, plate, name, status, f"{latency:.2f}s"), tags=(tag,))
        with open(LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            if os.path.getsize(LOG_FILE) == 0: 
                writer.writerow(["Plate","Name","Faculty","Status","Timestamp","Latency"])
            writer.writerow([plate, name, faculty, status, ts_l, f"{latency:.4f}"])
        load_history_data()
        
        if status == "AUTHORIZED":
            threading.Thread(target=authorized_feedback).start()
            trigger_gate_sequence()
        else:
            set_status("UNAUTHORIZED DETECTED", "red")
            buzzer.beep(on_time=0.1, off_time=0.05, n=4)
            root.after(3000, lambda: set_status("SCANNING", "white") if not is_gate_busy else None)
    except: pass

def preprocess_plate(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def reset_gate_system():
    global is_gate_busy, vehicle_entry_start_time, vehicle_confirmed, plate_absence_start_time, scan_resume_time
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    is_gate_busy = vehicle_confirmed = False
    vehicle_entry_start_time = None
    plate_absence_start_time = None
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
    global vehicle_entry_start_time, vehicle_confirmed, system_state, plate_absence_start_time, last_plate_seen_time, scan_resume_time, current_dist_global
    try:
        dist_cm = sensor.distance * 100
        current_dist_global = dist_cm
        if dist_cm >= 148: lbl_dist.config(text="DIST: > 150 cm", fg="white")
        else: lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")

        if is_gate_busy and system_state != "CLOSING GATE":
            # 1. Car confirmation phase
            if not vehicle_confirmed:
                if dist_cm < SAFETY_DISTANCE_CM:
                    if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
                    if (time.time() - vehicle_entry_start_time) >= ENTRY_CONFIRM_TIME:
                        vehicle_confirmed = True
                        print("🚗 Vehicle confirmed. Scanning disabled until cleared.")
                else: vehicle_entry_start_time = None
            
            # 2. Car clear sensor phase
            elif dist_cm > SAFETY_DISTANCE_CM:
                curr_t = time.time()
                
                # Start the 0.5s Grace Period once car clears
                if scan_resume_time == 0:
                    scan_resume_time = curr_t + POST_ENTRY_SCAN_DELAY
                
                # Resuming scanning phase
                if curr_t >= scan_resume_time:
                    # After resume, if no plate seen for 5 seconds, close
                    if (curr_t - last_plate_seen_time) > 1.2:
                        if plate_absence_start_time is None: plate_absence_start_time = curr_t
                        if (curr_t - plate_absence_start_time) >= GATE_CLOSE_GRACE_TIMEOUT:
                            execute_close_action()
                            return
                    else:
                        plate_absence_start_time = None 

        elif not is_gate_busy:
            if dist_cm < SAFETY_DISTANCE_CM and system_state != "UNAUTHORIZED DETECTED":
                set_status("DETECTING CAR", "yellow")
            elif dist_cm > SAFETY_DISTANCE_CM and system_state == "DETECTING CAR":
                set_status("SCANNING", "white")
    except: pass
    root.after(SENSOR_POLL_RATE, smart_gate_check)

def update_frame():
    global detection_start_time, last_plate_seen_time
    try:
        frame = picam2.capture_array() if picam2 else np.zeros((540, 960, 3), dtype=np.uint8)
    except: root.after(100, update_frame); return

    h, w, _ = frame.shape
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 0), 2)
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    cv2.putText(frame, f"STATUS: {system_state}", (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    dist_color = (0, 0, 255) if current_dist_global < SAFETY_DISTANCE_CM else (0, 255, 0)
    cv2.putText(frame, f"SENSOR: {current_dist_global:.1f} cm", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, dist_color, 2)
    
    curr_t = time.time()

    # --- UPDATED SCANNING LOGIC ---
    # We skip AI scanning if:
    # 1. Gate is physically closing
    # 2. Gate is open AND car hasn't cleared the sensor yet (confirmation phase)
    # 3. We are in the 0.5s "Readying" delay
    
    is_waiting_for_car = is_gate_busy and not vehicle_confirmed
    is_in_entry_pause = scan_resume_time > 0 and curr_t < scan_resume_time
    
    skip_scan = system_state == "CLOSING GATE" or is_waiting_for_car or is_in_entry_pause

    if is_in_entry_pause:
        cv2.putText(frame, "READYING NEXT SCAN...", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 100, 0), 2)
    elif is_waiting_for_car:
        cv2.putText(frame, "GATE OPEN: AWAITING PASSAGE", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    elif plate_absence_start_time and is_gate_busy:
        rem = max(0, GATE_CLOSE_GRACE_TIMEOUT - (curr_t - plate_absence_start_time))
        cv2.putText(frame, f"GRACE: {rem:.1f}s", (30, 130), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)

    if not skip_scan:
        results = model(roi_crop, verbose=False, conf=0.4)
        detected_box = False
        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected_box = True
                last_plate_seen_time = curr_t
                if detection_start_time is None: detection_start_time = curr_t
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 3)
                    if (curr_t - detection_start_time) > GRACE_PERIOD:
                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            ocr_res = reader.readtext(preprocess_plate(p_crop), detail=0)
                            clean_text = "".join([c for c in "".join(ocr_res).upper() if c.isalnum()])
                            if len(clean_text) > 3:
                                if clean_text not in first_sight_times: first_sight_times[clean_text] = curr_t
                                scan_buffer.append(clean_text)
                                most_common, freq = Counter(scan_buffer).most_common(1)[0]
                                if (curr_t - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                    latency = curr_t - first_sight_times.get(most_common, curr_t)
                                    if most_common in authorized_plates:
                                        row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                        log_to_gui_and_csv(most_common, row['Name'], row['Faculty'], "AUTHORIZED", latency)
                                    elif freq >= CONFIDENCE_THRESHOLD:
                                        log_to_gui_and_csv(most_common, "Unknown", "Visitor", "UNAUTHORIZED", latency)
                                    logged_vehicles[most_common] = curr_t

        if not detected_box and detection_start_time and (curr_t - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear(); first_sight_times.clear()

    win_w = video_frame_container.winfo_width()
    if win_w > 10:
        new_h = int(win_w / (w/h))
        img = Image.fromarray(cv2.resize(frame, (win_w, new_h)))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk; video_label.configure(image=imgtk)
    root.after(10, update_frame)

btn_refresh = tk.Button(tab_history, text="Refresh Logs", command=load_history_data)
btn_refresh.pack(side="bottom", fill="x", pady=5)

load_history_data()
root.after(500, smart_gate_check)
root.after(500, update_frame)

try:
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    if picam2: picam2.stop()
    gate_p1.close(); gate_p2.on()