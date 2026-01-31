import cv2
import easyocr
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
import re
from datetime import datetime
from collections import Counter, deque
import csv 
import tkinter as tk
from tkinter import ttk, font
from PIL import Image, ImageTk
from gpiozero import OutputDevice, DistanceSensor, Buzzer, LED 
from picamera2 import Picamera2

# ==========================================
#          CONFIGURATION
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
#          HARDWARE INITIALIZATION
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
#          GUI SETUP
# ==========================================
root = tk.Tk()
root.title("SAVES AI Control System")
root.geometry("1024x600")
root.configure(bg="white")

style = ttk.Style()
style.theme_use("clam")
style.configure("Treeview.Heading", background="#cccccc", foreground="white", font=("Arial", 11, "bold"), relief="flat")
style.map("Treeview.Heading", background=[('active', '#b3b3b3')])
style.configure("Treeview", font=("Arial", 10), rowheight=30, background="white", fieldbackground="white")

container = tk.Frame(root, bg="white")
container.pack(fill="both", expand=True)

page_camera = tk.Frame(container, bg="white")
page_logs = tk.Frame(container, bg="white")
page_history = tk.Frame(container, bg="white")

for frame in (page_camera, page_logs, page_history):
    frame.grid(row=0, column=0, sticky="nsew")

def show_frame(frame):
    frame.tkraise()

def create_header(parent, title_text, sub_text=None, left_btn=None, right_btn=None):
    header_frame = tk.Frame(parent, bg="white", pady=10)
    header_frame.pack(fill="x", padx=20)
    header_frame.columnconfigure(0, weight=1)
    header_frame.columnconfigure(1, weight=4)
    header_frame.columnconfigure(2, weight=1)

    if left_btn:
        btn = tk.Button(header_frame, text=left_btn['text'], command=left_btn['cmd'],
                        bg="#e0e0e0", fg="#555", font=("Arial", 10, "bold"), 
                        relief="flat", padx=15, pady=5, bd=0)
        if 'color' in left_btn: btn.config(fg=left_btn['color'], bg="#f0f0f0")
        btn.grid(row=0, column=0, sticky="w")

    title_container = tk.Frame(header_frame, bg="white")
    title_container.grid(row=0, column=1)
    tk.Label(title_container, text=title_text, font=("Helvetica", 24, "bold"), bg="white", fg="black").pack()
    if sub_text:
        tk.Label(title_container, text=sub_text, font=("Helvetica", 12), bg="white", fg="#555").pack()

    if right_btn:
        btn = tk.Button(header_frame, text=right_btn['text'], command=right_btn['cmd'],
                        bg="#e0e0e0", fg="#555", font=("Arial", 10, "bold"), 
                        relief="flat", padx=15, pady=5, bd=0)
        btn.grid(row=0, column=2, sticky="e")

# ================= PAGE 1: CAMERA =================
create_header(page_camera, "SAVES AI", None,
              left_btn={'text': "Reset System", 'cmd': lambda: reset_gate_system()},
              right_btn={'text': "Current Session Logs  ➜", 'cmd': lambda: show_frame(page_logs)})

video_frame_container = tk.Frame(page_camera, bg="#ff4d4d", padx=5, pady=5)
video_frame_container.pack(fill="both", expand=True, padx=40, pady=10)
video_label = tk.Label(video_frame_container, bg="black")
video_label.pack(fill="both", expand=True)

status_frame = tk.Frame(page_camera, bg="white")
status_frame.pack(fill="x", side="bottom", pady=10)
lbl_status = tk.Label(status_frame, text="SCANNING", font=("Arial", 14, "bold"), bg="white", fg="black")
lbl_status.pack(side="left", padx=40)
lbl_dist = tk.Label(status_frame, text="DIST: -- cm", font=("Arial", 12), bg="white", fg="black")
lbl_dist.pack(side="right", padx=40)

# ================= PAGE 2 & 3: LOGS =================
cols = ("Time", "Plate", "Name", "Status", "Latency", "Performance Metrics")

def create_treeview(parent):
    frame = tk.Frame(parent, bg="white")
    frame.pack(fill="both", expand=True, padx=40, pady=10)
    tree = ttk.Treeview(frame, columns=cols, show="headings", style="Treeview")
    scrollbar = ttk.Scrollbar(frame, orient="vertical", command=tree.yview)
    tree.configure(yscrollcommand=scrollbar.set)
    scrollbar.pack(side="right", fill="y")
    tree.pack(side="left", fill="both", expand=True)
    
    for col in cols:
        tree.heading(col, text=col)
        tree.column(col, anchor="center", width=120)
    tree.column("Performance Metrics", width=220)
    tree.tag_configure("authorized", foreground="green")
    tree.tag_configure("unauthorized", foreground="red")
    return tree

create_header(page_logs, "SAVES AI", "current session",
              left_btn={'text': "←  camera", 'cmd': lambda: show_frame(page_camera), 'color': '#6200ea'},
              right_btn={'text': "past session logs  ➜", 'cmd': lambda: show_frame(page_history)})
tree = create_treeview(page_logs)

create_header(page_history, "SAVES AI", "past session",
              left_btn={'text': "←  camera", 'cmd': lambda: show_frame(page_camera), 'color': '#6200ea'},
              right_btn={'text': "current session logs  ➜", 'cmd': lambda: show_frame(page_logs)})
tree_history = create_treeview(page_history)

tk.Button(page_history, text="Refresh Logs", command=lambda: load_history_data(), bg="#e0e0e0", relief="flat").pack(side="bottom", fill="x", pady=10, padx=40)

# ================= LOGIC =================
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
except: 
    authorized_plates = []
    auth_df = pd.DataFrame(columns=["Plate", "Name", "Faculty"])

reader = easyocr.Reader(['en'], gpu=False) 
model = YOLO('./best_ncnn_model') 

def set_status(status_text, color="black"):
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
                
                # UPDATED: Read from 2 separate columns 'Det' and 'Ocr'
                det = row.get('Det', '0')
                ocr = row.get('Ocr', '0')
                # Combine them for the GUI display
                perf_display = f"Det: {float(det):.0f}ms | OCR: {float(ocr):.0f}ms"
                
                tag = "authorized" if "AUTHORIZED" in row.get('Status', '') else "unauthorized"
                tree_history.insert("", "end", values=(short_ts, row.get('Plate'), row.get('Name'), row.get('Status'), row.get('Latency') + "s", perf_display), tags=(tag,))
    except Exception as e: print(f"Load Error: {e}")

def log_to_gui_and_csv(plate, name, faculty, status, latency, det_time, ocr_time):
    ts_l = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    ts_s = datetime.now().strftime("%H:%M:%S")
    
    # Create the combined string for the GUI (matches your reference image)
    perf_display = f"Det: {det_time:.0f}ms | OCR: {ocr_time:.0f}ms"
    
    try:
        tag = "authorized" if status == "AUTHORIZED" else "unauthorized"
        # Insert into Current Session Tree
        tree.insert("", 0, values=(ts_s, plate, name, status, f"{latency:.2f}s", perf_display), tags=(tag,))
        
        # Save to CSV using the 2 separate columns
        file_exists = os.path.isfile(LOG_FILE) and os.path.getsize(LOG_FILE) > 0
        with open(LOG_FILE, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists: 
                # UPDATED HEADER: Det and Ocr separate
                writer.writerow(["Plate","Name","Faculty","Status","Timestamp","Latency","Det","Ocr"])
            writer.writerow([plate, name, faculty, status, ts_l, f"{latency:.4f}", f"{det_time:.2f}", f"{ocr_time:.2f}"])
        
        load_history_data()
        
        if status == "AUTHORIZED": 
            trigger_gate_sequence()
        else:
            set_status("UNAUTHORIZED DETECTED", "red")
            buzzer.beep(on_time=0.1, off_time=0.05, n=4)
            root.after(3000, lambda: set_status("SCANNING", "black") if not is_gate_busy else None)
    except Exception as e: print(f"Log Error: {e}")

def preprocess_plate(img):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return thresh

def reset_gate_system():
    global is_gate_busy, vehicle_entry_start_time, vehicle_confirmed
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    is_gate_busy = vehicle_confirmed = False
    vehicle_entry_start_time = None
    set_status("SCANNING", "black")

def execute_close_action():
    set_status("CLOSING GATE", "orange")
    gate_p1.off(); gate_p2.on(); led_open.off(); led_close.on()
    root.after(GATE_ACTION_TIME, reset_gate_system)

# ================= GATE LOGIC =================
def trigger_gate_sequence():
    global is_gate_busy, vehicle_confirmed, vehicle_entry_start_time, system_state
    
    if is_gate_busy and system_state == "POST-ENTRY SCAN":
        set_status("NEXT VEHICLE DETECTED", "green")
        for _ in range(2):
            led_open.on(); buzzer.on(); time.sleep(0.1)
            led_open.off(); buzzer.off(); time.sleep(0.1)
        led_open.on()
        vehicle_confirmed = False
        vehicle_entry_start_time = None
        root.after(100, smart_gate_check)
        return

    if is_gate_busy: return 

    is_gate_busy = True
    set_status("AUTHORIZED: OPENING", "green")
    
    for _ in range(2):
        led_open.on(); buzzer.on(); time.sleep(0.1)
        led_open.off(); buzzer.off(); time.sleep(0.1)

    gate_p1.on(); gate_p2.off(); led_open.on(); led_close.off()
    root.after(GATE_ACTION_TIME, lambda: root.after(100, smart_gate_check))

def smart_gate_check():
    global vehicle_entry_start_time, vehicle_confirmed, system_state
    if system_state == "POST-ENTRY SCAN": return

    try:
        dist_cm = sensor.distance * 100
        if dist_cm >= 148:
            lbl_dist.config(text="DIST: > 150 cm", fg="black")
        else:
            lbl_dist.config(text=f"DIST: {dist_cm:.1f} cm", fg="red" if dist_cm < SAFETY_DISTANCE_CM else "green")

        if is_gate_busy and system_state != "CLOSING GATE":
            if not vehicle_confirmed:
                if dist_cm < SAFETY_DISTANCE_CM:
                    if vehicle_entry_start_time is None: vehicle_entry_start_time = time.time()
                    if (time.time() - vehicle_entry_start_time) >= ENTRY_CONFIRM_TIME:
                        vehicle_confirmed = True
                else:
                    vehicle_entry_start_time = None
            elif vehicle_confirmed and dist_cm > SAFETY_DISTANCE_CM:
                start_post_entry_wait()
                return

    except Exception as e: print(f"Sensor Error: {e}")
    root.after(SENSOR_POLL_RATE, smart_gate_check)

def start_post_entry_wait():
    set_status("WAITING...", "orange")
    root.after(int(POST_ENTRY_DELAY * 1000), lambda: init_post_entry_scan())

def init_post_entry_scan():
    global system_state, last_plate_seen_time
    system_state = "POST-ENTRY SCAN"
    last_plate_seen_time = time.time() 
    monitor_post_entry_timer()

def monitor_post_entry_timer():
    global system_state
    if system_state != "POST-ENTRY SCAN": return
    elapsed = time.time() - last_plate_seen_time
    countdown = max(0, EXIT_SCAN_COOLDOWN - elapsed)
    lbl_status.config(text=f"CLOSING IN: {countdown:.1f}s", fg="orange")
    if elapsed >= EXIT_SCAN_COOLDOWN:
        execute_close_action()
    else:
        root.after(100, monitor_post_entry_timer)

def update_frame():
    global detection_start_time, last_plate_seen_time
    try:
        frame = picam2.capture_array() if picam2 else np.zeros((540, 960, 3), dtype=np.uint8)
    except: root.after(100, update_frame); return

    current_dist = sensor.distance * 100
    cv2.putText(frame, f"STATUS: {system_state}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 3)

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
                            ocr_res = reader.readtext(preprocess_plate(p_crop), detail=0)
                            clean_text = "".join([c for c in "".join(ocr_res).upper() if c.isalnum()])
                            
                            # Correction: 123ABC -> ABC123
                            if len(clean_text) > 0 and clean_text[0].isdigit() and clean_text[-1].isalpha():
                                split_idx = -1
                                for i, char in enumerate(clean_text):
                                    if char.isalpha(): split_idx = i; break
                                if split_idx > 0:
                                    part_digits, part_letters = clean_text[:split_idx], clean_text[split_idx:]
                                    if part_digits.isdigit() and part_letters.isalpha():
                                        clean_text = f"{part_letters}{part_digits}"

                            t_ocr_ms = (time.perf_counter() - t_ocr) * 1000
                            
                            if len(clean_text) > 3: 
                                if clean_text not in first_sight_times: first_sight_times[clean_text] = current_time
                                scan_buffer.append(clean_text)
                                most_common, freq = Counter(scan_buffer).most_common(1)[0]
                                if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                    latency = current_time - first_sight_times.get(most_common, current_time)
                                    
                                    # Pass the raw values to the logging function
                                    if most_common in authorized_plates:
                                        row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                        log_to_gui_and_csv(most_common, row['Name'], row['Faculty'], "AUTHORIZED", latency, t_detect, t_ocr_ms)
                                    elif freq >= CONFIDENCE_THRESHOLD:
                                        log_to_gui_and_csv(most_common, "Unknown", "Visitor", "UNAUTHORIZED", latency, t_detect, t_ocr_ms)
                                    
                                    logged_vehicles[most_common] = current_time
                                    scan_buffer.clear(); first_sight_times.clear()

        if not detected_box and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear(); first_sight_times.clear()

    win_w, win_h = video_frame_container.winfo_width(), video_frame_container.winfo_height()
    if win_w > 10:
        new_w = win_w; new_h = int(win_w / (w/h))
        img = Image.fromarray(cv2.resize(frame, (new_w, new_h)))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk; video_label.configure(image=imgtk)
    root.after(10, update_frame)

show_frame(page_camera)
load_history_data()
root.after(500, smart_gate_check)
root.after(500, update_frame)
try: root.mainloop()
except KeyboardInterrupt: pass
finally:
    if picam2: picam2.stop()
    gate_p1.close(); gate_p2.on()