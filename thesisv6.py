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

# NEW: Gate Close Logic Config
GATE_CLOSE_GRACE_TIMEOUT = 5.0  # Seconds to wait after plate is gone

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

# NEW: Tracking for the 5-second close feature
plate_absence_start_time = None 

try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except: 
    authorized_plates = []
    auth_df = pd.DataFrame(columns=["Plate", "Name", "Faculty"])

reader = easyocr.Reader(['en'], gpu=False) 
model = YOLO('./best_ncnn_model') 

def set_status(status_text, color="white"):
    global system_state
    system_state = status_text
    lbl_status.config(text=status_text, fg=color)

def reset_gate_system():
    global is_gate_busy, vehicle_entry_start_time, vehicle_confirmed, plate_absence_start_time
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    is_gate_busy = vehicle_confirmed = False
    vehicle_entry_start_time = None
    plate_absence_start_time = None
    set_status("SCANNING", "white")

def execute_close_action():
    set_status("CLOSING GATE", "orange")
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    root.after(GATE_ACTION_TIME, reset_gate_system)

def trigger_gate_sequence():
    global is_gate_busy, vehicle_confirmed, plate_absence_start_time
    if is_gate_busy: return 
    is_gate_busy = True
    vehicle_confirmed = False
    plate_absence_start_time = None
    set_status("OPENING GATE", "green")
    gate_p1.on(); gate_p2.off(); led_open.on(); led_close.off()
    root.after(GATE_ACTION_TIME, lambda: smart_gate_check())

def smart_gate_check():
    global vehicle_entry_start_time, vehicle_confirmed, system_state, plate_absence_start_time, last_plate_seen_time
    try:
        dist_cm = sensor.distance * 100
        
        # UI Distance Update
        if dist_cm >= 148: lbl_dist.config(text="DIST: > 150 cm", fg="white")
        else: lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")

        # --- NEW LOGIC: GATE CLOSING CONDITION ---
        if is_gate_busy and system_state != "CLOSING GATE":
            # 1. First, confirm the vehicle actually started moving through
            if not vehicle_confirmed:
                if dist_cm < SAFETY_DISTANCE_CM:
                    if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
                    if (time.time() - vehicle_entry_start_time) >= ENTRY_CONFIRM_TIME:
                        vehicle_confirmed = True
                        print("🚗 Vehicle confirmed inside gate area.")
                else: vehicle_entry_start_time = None
            
            # 2. If confirmed and vehicle has cleared the sensor
            elif dist_cm > SAFETY_DISTANCE_CM:
                current_t = time.time()
                # Check if plate is currently visible (within ABSENCE_RESET_TIME)
                plate_is_visible = (current_t - last_plate_seen_time) < 1.0 
                
                if not plate_is_visible:
                    if plate_absence_start_time is None: 
                        plate_absence_start_time = current_t
                        print("⏱️ Plate lost. Starting 5s gate timer...")
                    
                    # If 5 seconds have passed since plate was last seen
                    if (current_t - plate_absence_start_time) >= GATE_CLOSE_GRACE_TIMEOUT:
                        execute_close_action()
                        return
                else:
                    # Reset timer if a plate reappears
                    plate_absence_start_time = None

        elif not is_gate_busy and dist_cm < SAFETY_DISTANCE_CM:
            if system_state != "UNAUTHORIZED DETECTED": set_status("DETECTING CAR", "yellow")
        elif not is_gate_busy and dist_cm > SAFETY_DISTANCE_CM:
            if system_state == "DETECTING CAR": set_status("SCANNING", "white")

    except Exception as e: print(f"Sensor Error: {e}")
    
    root.after(SENSOR_POLL_RATE, smart_gate_check)

def update_frame():
    global detection_start_time, last_plate_seen_time
    try:
        frame = picam2.capture_array() if picam2 else np.zeros((540, 960, 3), dtype=np.uint8)
    except: root.after(100, update_frame); return

    current_dist = sensor.distance * 100
    cv2.putText(frame, f"STATUS: {system_state}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)
    
    # Visual timer for the user
    if plate_absence_start_time and is_gate_busy and system_state != "CLOSING GATE":
        remaining = max(0, GATE_CLOSE_GRACE_TIMEOUT - (time.time() - plate_absence_start_time))
        cv2.putText(frame, f"CLOSING IN: {remaining:.1f}s", (30, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 165, 255), 3)

    h, w, _ = frame.shape
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), (0, 255, 0), 3) 
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    # SCANNING IS NOW ALWAYS ACTIVE (Removed 'not is_gate_busy' constraint)
    if system_state in ["SCANNING", "OPENING GATE", "DETECTING CAR", "UNAUTHORIZED DETECTED"]:
        results = model(roi_crop, verbose=False, conf=0.4) 
        current_time = time.time()
        detected_box = False

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected_box = True
                last_plate_seen_time = current_time 
                if detection_start_time is None: detection_start_time = current_time
                
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 3)
                    
                    if (current_time - detection_start_time) > GRACE_PERIOD:
                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            ocr_res = reader.readtext(preprocess_plate(p_crop), detail=0)
                            clean_text = "".join([c for c in "".join(ocr_res).upper() if c.isalnum()])
                            if len(clean_text) > 3: 
                                if clean_text not in first_sight_times: first_sight_times[clean_text] = current_time
                                scan_buffer.append(clean_text)
                                most_common, freq = Counter(scan_buffer).most_common(1)[0]
                                
                                # Process logging (only if gate isn't already open for this plate)
                                if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                    latency = current_time - first_sight_times.get(most_common, current_time)
                                    if most_common in authorized_plates:
                                        row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                        log_to_gui_and_csv(most_common, row['Name'], row['Faculty'], "AUTHORIZED", latency)
                                    elif freq >= CONFIDENCE_THRESHOLD:
                                        log_to_gui_and_csv(most_common, "Unknown", "Visitor", "UNAUTHORIZED", latency)
                                    logged_vehicles[most_common] = current_time

        if not detected_box and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear(); first_sight_times.clear()

    # GUI Update
    win_w = video_frame_container.winfo_width()
    if win_w > 10:
        new_h = int(win_w / (w/h))
        img = Image.fromarray(cv2.resize(frame, (win_w, new_h)))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk; video_label.configure(image=imgtk)
    root.after(10, update_frame)

# ==========================================
#           GUI / BOOT (REMAINDER)
# ==========================================
# [Existing Treeview/GUI code goes here - truncated for brevity]
# ... (same as your original GUI code) ...

# Ensure the rest of your original GUI code and the load_history_data function is present here.

load_history_data()
root.after(500, smart_gate_check)
root.after(500, update_frame)
root.mainloop()
