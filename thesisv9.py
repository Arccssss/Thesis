import cv2
import easyocr
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
import re
import sys 
from datetime import datetime
from collections import Counter, deque
import csv 
import tkinter as tk
from tkinter import ttk, font
from PIL import Image, ImageTk
from gpiozero import OutputDevice, DistanceSensor, Buzzer, LED 
from picamera2 import Picamera2

# ==========================================
#           CONFIGURATION
# ==========================================
AUTH_FILE = 'Database/authorized.csv'
LOG_FILE = 'Database/access_logs.csv'
LOG_COOLDOWN = 5  
SCAN_BUFFER_LEN = 5  
CONFIDENCE_THRESHOLD = 2

ROI_SCALE_W, ROI_SCALE_H = 0.7, 0.5 
ROI_COLOR = (0, 255, 0) 
GRACE_PERIOD = 0.3      
GATE_ACTION_TIME = 3000 
ABSENCE_RESET_TIME = 10.0 

GATE_SAFETY_TIMEOUT = 20.0

SAFETY_DISTANCE_CM = 100  
ENTRY_CONFIRM_TARGET = 0.5 
SENSOR_POLL_RATE = 150  

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
#          HARDWARE INITIALIZATION
# ==========================================
try:
    gate_p1 = OutputDevice(GATE_PIN_1, active_high=True, initial_value=False)
    gate_p2 = OutputDevice(GATE_PIN_2, active_high=True, initial_value=True)
    sensor = DistanceSensor(echo=US_ECHO_PIN, trigger=US_TRIG_PIN, max_distance=1.5, queue_len=3)
    buzzer = Buzzer(BUZZER_PIN)
    
    led_green_auth = LED(LED_OPEN_PIN)
    led_red_unauth = LED(LED_CLOSE_PIN)
    
    led_green_auth.off()
    led_red_unauth.on()
    
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
        def blink(self, on_time=0.1, off_time=0.1, n=1, background=True): pass
        def close(self): pass
        @property
        def value(self): return 0
        @property
        def distance(self): return 1.5 
    gate_p1 = gate_p2 = sensor = buzzer = DummyDev()
    led_green_auth = led_red_unauth = DummyDev()
    picam2 = None 

# ==========================================
#          GUI SETUP
# ==========================================
root = tk.Tk()
root.title("SAVES AI Control System")
root.configure(bg="white")
root.attributes('-fullscreen', True)
root.bind("<Escape>", lambda event: root.attributes("-fullscreen", True))

def force_fullscreen():
    root.attributes('-fullscreen', True)
root.after(500, force_fullscreen)

def close_application():
    if picam2: picam2.stop()
    gate_p1.close(); gate_p2.on() 
    led_green_auth.off(); led_red_unauth.off()
    root.destroy(); sys.exit(0)

def minimize_window(): 
    root.iconify()

style = ttk.Style()
style.theme_use("clam")
style.configure("Treeview.Heading", background="#cccccc", foreground="white", font=("Arial", 10, "bold"), relief="flat")
style.map("Treeview.Heading", background=[('active', '#b3b3b3')])
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
    try: btn_minimize.lift()
    except: pass

def create_header(parent, title_text, sub_text=None, left_btn=None, right_btn=None):
    header_frame = tk.Frame(parent, bg="white", pady=5)
    header_frame.pack(fill="x", padx=10)
    header_frame.columnconfigure(0, weight=1)
    header_frame.columnconfigure(1, weight=4)
    header_frame.columnconfigure(2, weight=1)

    if left_btn:
        bg_color = "#ffcccc" if "Shutdown" in left_btn['text'] else "#e0e0e0"
        fg_color = "red" if "Shutdown" in left_btn['text'] else "#555"
        btn = tk.Button(header_frame, text=left_btn['text'], command=left_btn['cmd'],
                        bg=bg_color, fg=fg_color, font=("Arial", 9, "bold"), 
                        relief="flat", padx=10, pady=5, bd=0)
        btn.grid(row=0, column=0, sticky="w")
        
        if "Shutdown" in left_btn['text']:
             btn_reset = tk.Button(header_frame, text="Reset", command=lambda: reset_gate_system(),
                        bg="#e0e0e0", fg="#555", font=("Arial", 9, "bold"), 
                        relief="flat", padx=10, pady=5, bd=0)
             btn_reset.grid(row=0, column=0, sticky="w", padx=(100, 0))

    title_container = tk.Frame(header_frame, bg="white")
    title_container.grid(row=0, column=1)
    tk.Label(title_container, text=title_text, font=("Helvetica", 18, "bold"), bg="white", fg="black").pack()
    if sub_text:
        tk.Label(title_container, text=sub_text, font=("Helvetica", 10), bg="white", fg="#555").pack()

    if right_btn:
        btn = tk.Button(header_frame, text=right_btn['text'], command=right_btn['cmd'],
                        bg="#e0e0e0", fg="#555", font=("Arial", 9, "bold"), 
                        relief="flat", padx=10, pady=5, bd=0)
        btn.grid(row=0, column=2, sticky="e")

# ================= PAGE 1: CAMERA =================
create_header(page_camera, "SAVES AI", None,
              left_btn={'text': "Shutdown", 'cmd': close_application},
              right_btn={'text': "Current Session Logs  ➜", 'cmd': lambda: show_frame(page_logs)})

video_frame_container = tk.Frame(page_camera, bg="#ff4d4d", padx=2, pady=2)
video_frame_container.pack(fill="both", expand=True, padx=10, pady=5)
video_frame_container.pack_propagate(False)

video_label = tk.Label(video_frame_container, bg="black")
video_label.pack(fill="both", expand=True)

status_frame = tk.Frame(page_camera, bg="white")
status_frame.pack(fill="x", side="bottom", pady=5)
lbl_status = tk.Label(status_frame, text="SCANNING", font=("Arial", 11, "bold"), bg="white", fg="black")
lbl_status.pack(side="left", padx=10)
lbl_dist = tk.Label(status_frame, text="DIST: -- cm", font=("Arial", 11), bg="white", fg="black")
lbl_dist.pack(side="right", padx=10)

# ================= LOGIC VARIABLES =================
logged_vehicles = {} 
scan_buffer = deque(maxlen=SCAN_BUFFER_LEN)
first_sight_times = {} 
detection_start_time = None
last_plate_seen_time = 0 

is_gate_busy = False
system_state = "SCANNING"  # UI ONLY
vehicle_confirmed = False
accumulated_presence = 0.0
gate_open_start_time = None 

# ================= FSM STATES =================
STATE_IDLE = "IDLE"
STATE_AUTHORIZED = "AUTHORIZED"
STATE_VEHICLE_ENTERING = "VEHICLE_ENTERING"
STATE_POST_ENTRY_SCAN = "POST_ENTRY_SCAN"
STATE_CLOSING = "CLOSING"

gate_state = STATE_IDLE  # REAL FSM STATE MACHINE

# ==========================================
# AUTH DATABASE
# ==========================================
try:
    auth_df = pd.read_csv(AUTH_FILE, dtype=str)
    auth_df['Plate'] = auth_df['Plate'].str.strip().str.upper()
    authorized_plates = auth_df['Plate'].tolist()
except: 
    authorized_plates = []
    auth_df = pd.DataFrame(columns=["Plate", "Name", "Faculty"])

reader = easyocr.Reader(['en'], gpu=False) 
model = YOLO('./best_ncnn_model') 

def set_status(status_text, color="black"):
    global system_state
    system_state = status_text
    lbl_status.config(text=status_text, fg=color)

def update_distance_ui(dist_cm):
    if dist_cm >= 148:
        lbl_dist.config(text="DIST: > 150 cm", fg="black")
    else:
        lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")

# ==========================================
# FSM RESET
# ==========================================
def reset_gate_system():
    global is_gate_busy, vehicle_confirmed, accumulated_presence
    global gate_open_start_time, detection_start_time
    global scan_buffer, first_sight_times, last_plate_seen_time, gate_state

    print("🔄 FULL SYSTEM RESET")

    gate_p1.off()
    gate_p2.on()
    led_green_auth.off()
    led_red_unauth.on()

    is_gate_busy = False
    vehicle_confirmed = False
    accumulated_presence = 0.0
    gate_open_start_time = None
    gate_state = STATE_IDLE

    detection_start_time = None
    last_plate_seen_time = 0
    scan_buffer.clear()
    first_sight_times.clear()

    set_status("SCANNING", "black")

def execute_close_action():
    global gate_state
    gate_state = STATE_CLOSING
    set_status("CLOSING GATE", "orange")
    gate_p1.off(); gate_p2.on()
    led_green_auth.off(); led_red_unauth.on()
    root.after(GATE_ACTION_TIME, reset_gate_system)

# ==========================================
# FSM GATE LOGIC
# ==========================================
def trigger_gate_sequence():
    global is_gate_busy, vehicle_confirmed, accumulated_presence, gate_open_start_time, gate_state

    if is_gate_busy and gate_state != STATE_POST_ENTRY_SCAN:
        return

    if is_gate_busy and gate_state == STATE_POST_ENTRY_SCAN:
        set_status("NEXT VEHICLE DETECTED", "green")
        led_green_auth.on()
        led_red_unauth.off()

        vehicle_confirmed = False
        accumulated_presence = 0.0
        gate_open_start_time = time.time()
        gate_state = STATE_VEHICLE_ENTERING
        root.after(100, smart_gate_check)
        return

    if is_gate_busy:
        return

    is_gate_busy = True
    accumulated_presence = 0.0
    vehicle_confirmed = False
    gate_open_start_time = time.time()

    gate_state = STATE_AUTHORIZED
    set_status("AUTHORIZED: OPENING", "green")

    led_green_auth.on()
    led_red_unauth.off()

    gate_p1.on()
    gate_p2.off()

    root.after(100, smart_gate_check)

def smart_gate_check():
    global is_gate_busy, vehicle_confirmed, accumulated_presence, gate_open_start_time, gate_state, last_plate_seen_time

    try:
        dist_cm = sensor.distance * 100
        update_distance_ui(dist_cm)

        if gate_open_start_time and (time.time() - gate_open_start_time) > GATE_SAFETY_TIMEOUT:
            execute_close_action()
            return

        # STATE: AUTHORIZED
        if gate_state == STATE_AUTHORIZED:
            if dist_cm < SAFETY_DISTANCE_CM:
                accumulated_presence += SENSOR_POLL_RATE / 1000.0
            else:
                accumulated_presence = 0.0

            if accumulated_presence >= ENTRY_CONFIRM_TARGET:
                vehicle_confirmed = True
                gate_state = STATE_VEHICLE_ENTERING
                set_status("VEHICLE ENTERING", "green")

            root.after(SENSOR_POLL_RATE, smart_gate_check)
            return

        # STATE: VEHICLE ENTERING
        if gate_state == STATE_VEHICLE_ENTERING:
            if dist_cm > SAFETY_DISTANCE_CM:
                gate_state = STATE_POST_ENTRY_SCAN
                set_status("WAITING...", "orange")
                last_plate_seen_time = time.time()

            root.after(SENSOR_POLL_RATE, smart_gate_check)
            return

        # STATE: POST ENTRY SCAN
        if gate_state == STATE_POST_ENTRY_SCAN:
            if dist_cm < SAFETY_DISTANCE_CM:
                trigger_gate_sequence()
                return

            if time.time() - last_plate_seen_time > EXIT_SCAN_COOLDOWN:
                execute_close_action()
                return

            root.after(SENSOR_POLL_RATE, smart_gate_check)
            return

    except Exception as e:
        print(f"Sensor Error: {e}")

    root.after(SENSOR_POLL_RATE, smart_gate_check)

# ==========================================
# CAMERA LOOP (UNCHANGED)
# ==========================================
def update_frame():
    global detection_start_time, last_plate_seen_time
    
    if picam2 is None:
        frame = np.random.randint(0, 256, (540, 960, 3), dtype=np.uint8)
        cv2.putText(frame, "NO CAMERA", (100, 270), cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 0, 255), 5)
    else:
        try:
            frame = picam2.capture_array()
        except Exception as e:
            print(f"📷 Camera Error: {e}")
            root.after(100, update_frame)
            return

    try:
        current_dist = sensor.distance * 100
        update_distance_ui(current_dist)
    except:
        pass

    cv2.putText(frame, f"STATUS: {system_state}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)

    h, w, _ = frame.shape
    roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
    roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
    cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), ROI_COLOR, 3) 
    roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

    should_detect = (not is_gate_busy) or (gate_state == STATE_POST_ENTRY_SCAN)

    if should_detect and gate_state != STATE_CLOSING:
        t0 = time.perf_counter()
        results = model(roi_crop, verbose=False, conf=0.4) 
        t_detect = (time.perf_counter() - t0) * 1000
        detected_box = False
        current_time = time.time()

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected_box = True
                last_plate_seen_time = current_time 
                if detection_start_time is None:
                    detection_start_time = current_time

                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (0, 255, 0), 3)

                    if (current_time - detection_start_time) > GRACE_PERIOD:
                        p_crop = roi_crop[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            ocr_res = reader.readtext(p_crop, detail=0)
                            clean_text = "".join([c for c in "".join(ocr_res).upper() if c.isalnum()])

                            if len(clean_text) > 3:
                                scan_buffer.append(clean_text)
                                most_common, freq = Counter(scan_buffer).most_common(1)[0]

                                if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                    if most_common in authorized_plates:
                                        trigger_gate_sequence()
                                    logged_vehicles[most_common] = current_time
                                    scan_buffer.clear()

        if not detected_box and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None
            scan_buffer.clear()

    win_w = video_frame_container.winfo_width()
    win_h = video_frame_container.winfo_height()

    if win_w > 10 and win_h > 10:
        scale = min(win_w / w, win_h / h)
        new_w, new_h = int(w * scale), int(h * scale)
        img = Image.fromarray(cv2.resize(frame, (new_w, new_h)))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk
        video_label.configure(image=imgtk)

    root.after(10, update_frame)

show_frame(page_camera)
root.after(500, smart_gate_check)
root.after(500, update_frame)

try:
    root.mainloop()
except KeyboardInterrupt:
    pass
finally:
    if picam2:
        picam2.stop()
    gate_p1.close()
    gate_p2.on()