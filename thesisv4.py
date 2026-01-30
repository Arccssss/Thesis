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

# ==========================================
#         CONFIGURATION
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

SAFETY_DISTANCE_CM = 100 
ENTRY_CONFIRM_TIME = 0.5 
SENSOR_POLL_RATE = 100    

# --- HARDWARE PINS ---
GATE_PIN_1 = 17 # OPEN
GATE_PIN_2 = 27 # CLOSE
US_TRIG_PIN = 23 
US_ECHO_PIN = 24 
BUZZER_PIN = 22
LED_OPEN_PIN = 5   
LED_CLOSE_PIN = 6  

# ==========================================
#         HARDWARE INITIALIZATION
# ==========================================
try:
    gate_p1 = OutputDevice(GATE_PIN_1, active_high=True, initial_value=False)
    gate_p2 = OutputDevice(GATE_PIN_2, active_high=True, initial_value=True)
    
    # --- HARDWARE ADAPTATION START ---
    # Increased max_distance to 4.0 to utilize the 5V power boost.
    # Added queue_len=3 to provide software smoothing for the cleaner signal.
    sensor = DistanceSensor(echo=US_ECHO_PIN, trigger=US_TRIG_PIN, max_distance=4.0, queue_len=3)
    # --- HARDWARE ADAPTATION END ---
    
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
    print("✅ Hardware & Picamera2 Initialized (5V Ultrasonic Logic Applied)")
    
except Exception as e:
    print(f"⚠️ HARDWARE ERROR: {e}")
    class DummyDev: 
        def on(self): pass
        def off(self): pass
        def beep(self, on_time=0.1, off_time=0.1, n=1): pass
        @property
        def value(self): return 0
        @property
        def distance(self): return 1.5 
    gate_p1 = gate_p2 = sensor = buzzer = DummyDev()
    led_open = led_close = DummyDev()
    picam2 = None 

# ==========================================
#         GUI SETUP (Tkinter)
# ==========================================
root = tk.Tk()
root.title("RPi 5 Smart Access Control")
root.geometry("800x550") 

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

status_frame = tk.Frame(tab_camera, bg="#333")
status_frame.pack(fill="x", side="bottom")
lbl_status = tk.Label(status_frame, text="SCANNING", font=("Arial", 12, "bold"), bg="#333", fg="white", width=30)
lbl_status.pack(side="left", padx=10, pady=5)
lbl_dist = tk.Label(status_frame, text="DIST: -- cm", font=("Arial", 12), bg="#333", fg="cyan")
lbl_dist.pack(side="right", padx=10, pady=5)
lbl_debug = tk.Label(status_frame, text="Last Read: None", font=("Arial", 10), bg="#333", fg="yellow")
lbl_debug.pack(side="right", padx=10)

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

btn_refresh = tk.Button(tab_history, text="Refresh Logs", command=lambda: load_history_data())
btn_refresh.pack(side="bottom", fill="x", pady=5)

# ==========================================
#         LOGIC & HELPERS
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

try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except Exception as e: 
    print(f"⚠️ Error loading database: {e}")
    authorized_plates = []
    auth_df = pd.DataFrame(columns=["Plate", "Name", "Faculty"])

reader = easyocr.Reader(['en'], gpu=False) 
model = YOLO('./best_ncnn_model') 

def set_status(status_text, color="white"):
    global system_state
    system_state = status_text
    lbl_status.config(text=status_text, fg=color)

def load_history_data():
    for i in tree_history.get_children():
        tree_history.delete(i)
    if not os.path.isfile(LOG_FILE): return
    try:
        with open(LOG_FILE, 'r') as f:
            reader_csv = csv.DictReader(f)
            for row in reversed(list(reader_csv)):
                full_ts = row.get('Timestamp', '')
                try: short_time = datetime.strptime(full_ts, "%Y-%m-%d %H:%M:%S").strftime("%H:%M:%S")
                except: short_time = full_ts
                status_val = row.get('Status', 'UNKNOWN')
                tag = "authorized" if "AUTHORIZED" in status_val else "unauthorized"
                tree_history.insert("", "end", values=(short_time, row.get('Plate'), row.get('Name'), status_val, row.get('Latency') + "s"), tags=(tag,))
    except Exception as e: print(f"Error loading history: {e}")

def log_to_gui_and_csv(plate, name, faculty, status, latency):
    ts_long = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ts_short = datetime.now().strftime("%H:%M:%S")
    try:
        tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
        tree.insert("", 0, values=(ts_short, plate, name, status, f"{latency:.2f}s"), tags=(tag,))
        file_exists = os.path.isfile(LOG_FILE)
        with open(LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists or os.path.getsize(LOG_FILE) == 0: 
                writer.writerow(["Plate","Name","Faculty","Status","Timestamp","Latency"])
            writer.writerow([plate, name, faculty, status, ts_long, f"{latency:.4f}"])
        load_history_data()
        
        if status == "AUTHORIZED": 
            trigger_gate_sequence()
        else:
            set_status("UNAUTHORIZED DETECTED", "red")
            buzzer.beep(on_time=0.1, off_time=0.05, n=4)
            root.after(3000, lambda: set_status("SCANNING", "white") if not is_gate_busy else None)
    except Exception as e: print(f"Log Error: {e}")

def preprocess_plate(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def reset_gate_system():
    global is_gate_busy, vehicle_entry_start_time, vehicle_confirmed
    gate_p1.off()
    gate_p2.on() 
    led_open.off()
    led_close.on()
    is_gate_busy = False
    vehicle_entry_start_time = None
    vehicle_confirmed = False
    set_status("SCANNING", "white")

def execute_close_action():
    set_status("CLOSING GATE", "orange")
    gate_p1.off() 
    gate_p2.on() 
    led_open.off()
    led_close.on()
    root.after(GATE_ACTION_TIME, reset_gate_system)

def trigger_gate_sequence():
    global is_gate_busy
    if is_gate_busy: return 
    is_gate_busy = True
    set_status("OPENING GATE", "green")
    gate_p1.on()
    gate_p2.off() 
    led_open.on()
    led_close.off()
    root.after(GATE_ACTION_TIME, lambda: root.after(100, smart_gate_check))

def smart_gate_check():
    global vehicle_entry_start_time, vehicle_confirmed, system_state
    try:
        # UPDATED: Distance conversion with higher range logic
        dist_cm = sensor.distance * 100
        
        # Display logic for UI - Handles the new 4m (400cm) limit
        if dist_cm >= 395:
            lbl_dist.config(text="DIST: > 400 cm", fg="white")
        else:
            lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")

        if dist_cm < SAFETY_DISTANCE_CM:
            if not is_gate_busy and system_state != "UNAUTHORIZED DETECTED":
                set_status("DETECTING CAR", "yellow")

        if is_gate_busy and system_state != "CLOSING GATE":
            if not vehicle_confirmed:
                if dist_cm < SAFETY_DISTANCE_CM:
                    if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
                    elapsed = time.time() - vehicle_entry_start_time
                    if elapsed >= ENTRY_CONFIRM_TIME:
                        vehicle_confirmed = True
                else:
                    vehicle_entry_start_time = None
            else:
                if dist_cm > SAFETY_DISTANCE_CM:
                    execute_close_action()
                    return 
        
        elif not is_gate_busy and dist_cm > SAFETY_DISTANCE_CM:
             if system_state == "DETECTING CAR":
                 set_status("SCANNING", "white")
    except: pass
    
    if is_gate_busy or system_state == "DETECTING CAR":
        root.after(SENSOR_POLL_RATE, smart_gate_check)
    else:
        root.after(200, smart_gate_check)

def update_frame():
    global detection_start_time, last_plate_seen_time
    try:
        if picam2: frame = picam2.capture_array()
        else:
            frame = np.zeros((540, 960, 3), dtype=np.uint8) 
            cv2.putText(frame, "NO CAMERA", (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    except:
        root.after(100, update_frame)
        return

    current_dist_cm = sensor.distance * 100
    cv2.putText(frame, f"STATUS: {system_state}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
    cv2.putText(frame, f"DIST: {current_dist_cm:.1f} cm", (30, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 3)

    h, w, _ = frame.shape
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), ROI_COLOR, 3) 
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    t_detect_ms = t_extract_ms = t_compare_ms = t_log_ms = 0.0
    
    if not is_gate_busy and system_state in ["SCANNING", "DETECTING CAR", "UNAUTHORIZED DETECTED"]:
        t0 = time.perf_counter()
        results = model(roi_crop, verbose=False, conf=0.4) 
        t1 = time.perf_counter()
        t_detect_ms = (t1 - t0) * 1000

        detected_box = False
        current_time = time.time()

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected_box = True
                last_plate_seen_time = current_time 
                if detection_start_time is None: detection_start_time = current_time
                time_seen = current_time - detection_start_time

                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 3)

                    if time_seen > GRACE_PERIOD:
                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            t2 = time.perf_counter()
                            processed_plate = preprocess_plate(p_crop)
                            ocr_res = reader.readtext(processed_plate, detail=0)
                            raw_text = "".join(ocr_res).upper()
                            clean_text = "".join([c for c in raw_text if c.isalnum()])
                            t3 = time.perf_counter()
                            t_extract_ms = (t3 - t2) * 1000

                            if clean_text:
                                lbl_debug.config(text=f"Last Read: {clean_text}")
                                if len(clean_text) > 3: 
                                    t4 = time.perf_counter()
                                    if clean_text not in first_sight_times: first_sight_times[clean_text] = current_time
                                    scan_buffer.append(clean_text)
                                    log_triggered = False
                                    if len(scan_buffer) > 0:
                                        most_common, freq = Counter(scan_buffer).most_common(1)[0]
                                        cur_t = current_time
                                        last_log = logged_vehicles.get(most_common, 0)
                                        is_authorized = most_common in authorized_plates
                                        t5 = time.perf_counter()
                                        t_compare_ms = (t5 - t4) * 1000

                                        if (cur_t - last_log) > LOG_COOLDOWN:
                                            latency = cur_t - first_sight_times.get(most_common, cur_t)
                                            if is_authorized:
                                                row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                                t6 = time.perf_counter()
                                                log_to_gui_and_csv(most_common, row['Name'], row['Faculty'], "AUTHORIZED", latency)
                                                t7 = time.perf_counter()
                                                t_log_ms = (t7 - t6) * 1000
                                                logged_vehicles[most_common] = cur_t
                                                if most_common in first_sight_times: del first_sight_times[most_common]
                                                scan_buffer.clear() 
                                                log_triggered = True
                                            elif freq >= CONFIDENCE_THRESHOLD:
                                                t6 = time.perf_counter()
                                                log_to_gui_and_csv(most_common, "Unknown", "Visitor", "UNAUTHORIZED", latency)
                                                t7 = time.perf_counter()
                                                t_log_ms = (t7 - t6) * 1000
                                                logged_vehicles[most_common] = cur_t
                                                if most_common in first_sight_times: del first_sight_times[most_common]
                                                scan_buffer.clear()
                                                log_triggered = True

                                    print(f"[PERF] Plate: {clean_text} | Det: {t_detect_ms:.1f}ms | Ext: {t_extract_ms:.1f}ms")

        if not detected_box:
            if detection_start_time is not None:
                if (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
                    print(">> Resetting Session Latency.")
                    detection_start_time = None
                    scan_buffer.clear()
                    first_sight_times.clear()

    win_w, win_h = video_frame_container.winfo_width(), video_frame_container.winfo_height()
    if win_w < 10 or win_h < 10: win_w, win_h = 640, 480
    img_h, img_w, _ = frame.shape
    aspect = img_w / img_h
    if win_w / win_h > aspect:
        new_h = win_h
        new_w = int(new_h * aspect)
    else:
        new_w = win_w
        new_h = int(new_w / aspect)

    try:
        img_small = cv2.resize(frame, (new_w, new_h))
        img = Image.fromarray(img_small) 
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)
    except: pass

    root.after(10, update_frame)

load_history_data()
root.after(500, smart_gate_check)
root.after(500, update_frame)
try:
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    if picam2: picam2.stop()
    gate_p1.close()
    gate_p2.on()
    print("System Shutdown")