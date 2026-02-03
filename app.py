import cv2
import easyocr
import numpy as np
from ultralytics import YOLO
import time
import pandas as pd
import os
import sys
import threading
import csv
import json
from datetime import datetime
from collections import Counter, deque
from flask import Flask, render_template, Response, jsonify, request
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

GATE_ACTION_TIME = 3.0   # seconds
GATE_SAFETY_TIMEOUT = 20.0
EXIT_SCAN_COOLDOWN = 5.0
SENSOR_POLL_RATE = 0.1   # seconds

SAFETY_DISTANCE_CM = 100
ENTRY_CONFIRM_TARGET = 0.5
ABSENCE_RESET_TIME = 10.0

# Hardware Pins
GATE_PIN_1 = 17
GATE_PIN_2 = 27
US_TRIG_PIN = 23
US_ECHO_PIN = 24
BUZZER_PIN = 22
LED_OPEN_PIN = 5
LED_CLOSE_PIN = 6

# States
STATE_IDLE = "IDLE"
STATE_OPENING = "OPENING"
STATE_WAITING_ENTRY = "WAITING_ENTRY"
STATE_POST_ENTRY = "POST_ENTRY"
STATE_CLOSING = "CLOSING"

# ==========================================
#           GLOBAL VARIABLES
# ==========================================
app = Flask(__name__)

# Logic Globals
current_state = STATE_IDLE
state_start_time = 0
accumulated_presence = 0.0
last_distance_cm = 0.0

logged_vehicles = {}
scan_buffer = deque(maxlen=SCAN_BUFFER_LEN)
first_sight_times = {}
detection_start_time = None
last_plate_seen_time = 0

# Web Globals
output_frame = None
lock = threading.Lock()
session_logs = []  # Stores current session logs for the UI

# ==========================================
#           HARDWARE INIT
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
    print("✅ Hardware Initialized")
except Exception as e:
    print(f"⚠️ HARDWARE ERROR: {e}")
    sys.exit(1)

# Load Database & Models
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
#           HELPER FUNCTIONS
# ==========================================
def fix_reversed_plate(text):
    if len(text) < 4: return text
    if text[0].isdigit() and text[-1].isalpha():
        split_index = -1
        for i, char in enumerate(text):
            if char.isalpha(): split_index = i; break
        if split_index > 0:
            return text[split_index:] + text[:split_index]
    return text

def transition_to(new_state):
    global current_state, state_start_time, accumulated_presence
    current_state = new_state
    state_start_time = time.time()
    accumulated_presence = 0.0
    print(f"🔀 State: {new_state}")

    if new_state == STATE_IDLE:
        gate_p1.off(); gate_p2.on()
        led_green_auth.off(); led_red_unauth.on()
        scan_buffer.clear(); first_sight_times.clear()

    elif new_state == STATE_OPENING:
        gate_p1.on(); gate_p2.off()
        led_green_auth.on(); led_red_unauth.off()
        buzzer.beep(on_time=0.1, off_time=0.1, n=2)

    elif new_state == STATE_WAITING_ENTRY:
        led_green_auth.on(); led_red_unauth.off()

    elif new_state == STATE_POST_ENTRY:
        led_green_auth.off(); led_red_unauth.on()

    elif new_state == STATE_CLOSING:
        gate_p1.off(); gate_p2.on()
        led_green_auth.off(); led_red_unauth.on()
        # Non-blocking delay handled in FSM loop

def log_event(plate, name, status, det, ocr, total_lat):
    global session_logs
    
    # 1. Capture Time ONCE (So Web and CSV match exactly)
    now = datetime.now()
    display_time = now.strftime("%H:%M:%S")          # Format for Phone Screen (e.g., 14:30:05)
    csv_timestamp = now.strftime("%Y-%m-%d %H:%M:%S") # Format for CSV File (e.g., 2026-02-03 14:30:05)

    # 2. Update In-Memory List (Web UI)
    perf = f"Det:{det:.0f}ms | OCR:{ocr:.0f}ms"
    
    log_entry = {
        "time": display_time, # Uses the captured time
        "plate": plate,
        "name": name,
        "status": status,
        "latency": f"{total_lat:.0f} ms",
        "metrics": perf
    }
    session_logs.insert(0, log_entry)
    
    # 3. Save to CSV
    try:
        file_exists = os.path.isfile(LOG_FILE)
        with open(LOG_FILE, 'a', newline='', encoding='utf-8') as f:
            headers = ["Plate", "Name", "Faculty", "Status", "Timestamp", "Latency_Total_ms", "Det_ms", "OCR_ms"]
            writer = csv.DictWriter(f, fieldnames=headers)
            
            if not file_exists:
                writer.writeheader()
                
            writer.writerow({
                "Plate": plate,
                "Name": name,
                "Faculty": "N/A", 
                "Status": status,
                "Timestamp": csv_timestamp, # Uses the EXACT same captured time
                "Latency_Total_ms": f"{total_lat:.2f}",
                "Det_ms": f"{det:.2f}",
                "OCR_ms": f"{ocr:.2f}"
            })
    except Exception as e:
        print(f"❌ Log Error: {e}")

    # 4. Hardware Actions
    if status == "AUTHORIZED":
        if current_state == STATE_IDLE or current_state == STATE_POST_ENTRY:
             transition_to(STATE_OPENING)
    else:
        # Unauthorized Alert (Server Compatible)
        if current_state == STATE_IDLE:
            led_red_unauth.blink(on_time=0.1, off_time=0.1, n=3, background=True)
            buzzer.beep(on_time=0.1, off_time=0.1, n=3, background=True)
            
            # Reset LED after 0.7s
            def reset_led():
                if current_state == STATE_IDLE:
                    led_red_unauth.on()

            threading.Timer(0.7, reset_led).start()

# ==========================================
#           BACKGROUND LOOPS
# ==========================================
def fsm_loop():
    global accumulated_presence, last_distance_cm, current_state
    
    while True:
        try:
            last_distance_cm = sensor.distance * 100
        except: 
            last_distance_cm = 999.0

        elapsed = time.time() - state_start_time

        if current_state == STATE_OPENING:
            transition_to(STATE_WAITING_ENTRY)

        elif current_state == STATE_WAITING_ENTRY:
            if elapsed > GATE_SAFETY_TIMEOUT:
                transition_to(STATE_CLOSING)
            elif last_distance_cm < SAFETY_DISTANCE_CM:
                accumulated_presence += SENSOR_POLL_RATE
                if accumulated_presence >= ENTRY_CONFIRM_TARGET:
                    transition_to(STATE_POST_ENTRY)
            else:
                accumulated_presence = 0.0

        elif current_state == STATE_POST_ENTRY:
            if elapsed >= EXIT_SCAN_COOLDOWN:
                transition_to(STATE_CLOSING)

        elif current_state == STATE_CLOSING:
            # Handle the 3 second closing delay here instead of root.after
            if elapsed >= GATE_ACTION_TIME:
                transition_to(STATE_IDLE)

        time.sleep(SENSOR_POLL_RATE)

def camera_loop():
    global output_frame, detection_start_time, last_plate_seen_time, logged_vehicles
    
    while True:
        try:
            frame = picam2.capture_array()
            t_start_process = time.perf_counter()
            
            # Note: Flask serves JPEGs which are BGR, so no need to convert to RGB for display
            
            h, w, _ = frame.shape
            roi_x, roi_y = int(w*(1-ROI_SCALE_W)//2), int(h*(1-ROI_SCALE_H)//2)
            roi_w, roi_h = int(w*ROI_SCALE_W), int(h*ROI_SCALE_H)
            
            # Draw ROI on frame
            cv2.rectangle(frame, (roi_x, roi_y), (roi_x+roi_w, roi_y+roi_h), ROI_COLOR, 3)
            roi_crop = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]

            should_detect = (current_state == STATE_IDLE) or (current_state == STATE_POST_ENTRY)

            if should_detect:
                t0_det = time.perf_counter()
                results = model(roi_crop, verbose=False, conf=0.4)
                t_detect = (time.perf_counter() - t0_det) * 1000
                
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
                                cv2.rectangle(frame, (x1+roi_x, y1+roi_y), (x2+roi_x, y2+roi_y), (255, 0, 0), 3)

                                p_crop = roi_crop[y1:y2, x1:x2]
                                if p_crop.size > 0:
                                    t0_ocr = time.perf_counter()
                                    gray = cv2.cvtColor(p_crop, cv2.COLOR_BGR2GRAY)
                                    text = reader.readtext(gray, detail=0)
                                    t_ocr = (time.perf_counter() - t0_ocr) * 1000
                                    
                                    clean = "".join([c for c in "".join(text).upper() if c.isalnum()])
                                    clean = fix_reversed_plate(clean)

                                    if len(clean) > 3:
                                        scan_buffer.append(clean)
                                        most_common, _ = Counter(scan_buffer).most_common(1)[0]

                                        if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                            # Calculate Total Latency
                                            total_latency = (time.perf_counter() - t_start_process) * 1000
                                            
                                            if most_common in authorized_plates:
                                                row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                                log_event(most_common, row['Name'], "AUTHORIZED", t_detect, t_ocr, total_latency)
                                            else:
                                                log_event(most_common, "Unknown", "UNAUTHORIZED", t_detect, t_ocr, total_latency)
                                            
                                            logged_vehicles[most_common] = current_time
                                            scan_buffer.clear()

                if not detected and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
                    detection_start_time = None; scan_buffer.clear()

            # Encode for Web Streaming
            with lock:
                ret, buffer = cv2.imencode('.jpg', frame)
                if ret:
                    output_frame = buffer.tobytes()

        except Exception as e:
            print(f"Camera Loop Error: {e}")
            time.sleep(0.1)

# ==========================================
#           FLASK ROUTES
# ==========================================
@app.route('/')
def index():
    return render_template('index.html')

def gen_frames():
    while True:
        with lock:
            if output_frame is None:
                continue
            frame_data = output_frame
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/status')
def api_status():
    global last_distance_cm, current_state
    return jsonify({
        "state": current_state,
        "distance": round(last_distance_cm, 1),
        "status_color": "green" if current_state in [STATE_OPENING, STATE_WAITING_ENTRY] else "black"
    })

@app.route('/api/logs')
def api_logs():
    return jsonify(session_logs)

# Replace the existing @app.route('/api/history') with this:
@app.route('/api/history')
def api_history():
    history = []
    
    # 1. Safety Check: Does file exist?
    if not os.path.exists(LOG_FILE):
        return jsonify([])

    try:
        with open(LOG_FILE, 'r', encoding='utf-8', errors='ignore') as f:
            # 2. Check for empty file
            f.seek(0, os.SEEK_END)
            if f.tell() == 0:
                return jsonify([])
            f.seek(0)

            # 3. Read the CSV
            reader = csv.DictReader(f)
            
            # 4. Extract ONLY the columns you want
            # We use .get() so if a column is missing, it returns "N/A" instead of crashing
            raw_data = list(reader)
            
            for row in reversed(raw_data):
                # Skip empty rows
                if not row.get('Plate'): continue
                
                # Create a simple object with only 3-4 fields
                clean_entry = {
                    "Timestamp": row.get("Timestamp", ""),
                    "Plate":     row.get("Plate", "Unknown"),
                    "Status":    row.get("Status", "--"),
                    "Name":      row.get("Name", "") # Optional: Keeps the UI looking nice
                }
                history.append(clean_entry)

    except Exception as e:
        print(f"⚠️ HISTORY ERROR: {e}", flush=True)
        return jsonify([])
        
    # Return last 50 entries
    return jsonify(history[:50])

@app.route('/api/reset', methods=['POST'])
def api_reset():
    # 1. Print immediately to terminal
    print("\n🔵 WEB COMMAND RECEIVED: RESET", flush=True)
    
    # 2. Perform logic
    transition_to(STATE_IDLE)
    buzzer.beep(on_time=0.1, off_time=0.1, n=1, background=True)
    
    return jsonify({"success": True})

@app.route('/api/shutdown', methods=['POST'])
def api_shutdown():
    # 1. Print immediately to terminal
    print("\n🔴 WEB COMMAND RECEIVED: SHUTDOWN", flush=True)
    
    # 2. Define the delayed kill function
    def delayed_kill():
        time.sleep(1.0) # Wait 1s for the phone to get the "OK" message
        print("🛑 SYSTEM: Killing process now...", flush=True)
        gate_p1.off()
        gate_p2.on()
        os._exit(0)

    # 3. Start the delay in background so we can reply to the phone first
    threading.Thread(target=delayed_kill).start()
    
    return jsonify({"success": True})

# ==========================================
#           MAIN ENTRY
# ==========================================
if __name__ == '__main__':
    # Start Background Threads
    t_fsm = threading.Thread(target=fsm_loop, daemon=True)
    t_cam = threading.Thread(target=camera_loop, daemon=True)
    
    t_fsm.start()
    t_cam.start()
    
    # Run Flask Server (Host 0.0.0.0 makes it accessible on network)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)