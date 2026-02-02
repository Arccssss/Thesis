import cv2
from paddleocr import PaddleOCR  # Changed from easyocr
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

# [FIX] Suppress terminal warnings
warnings.filterwarnings("ignore")

# ==========================================
#           CONFIGURATION
# ==========================================
AUTH_FILE = 'Database/authorized.csv'
LOG_FILE = 'Database/access_logs.csv'
LOG_COOLDOWN = 5  
SCAN_BUFFER_LEN = 9  
CONFIDENCE_THRESHOLD = 4

# Scaler Crop (Digital Zoom) - center 60% of sensor
# This replaces manual ROI crop logic for better performance
ZOOM_CROP = (0.2, 0.2, 0.6, 0.6) 

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
#          INITIALIZATION
# ==========================================

# Initialize PaddleOCR (English, No GPU for Pi)
# Note: rec_batch_num=1 reduces memory usage on Pi
paddle_reader = PaddleOCR(use_angle_cls=True, lang='en', use_gpu=False, show_log=False, rec_batch_num=1)
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
    # Optimized Resolution: 720p is the sweet spot for YOLO 640x640
    config = picam2.create_preview_configuration(main={"format": "BGR888", "size": (1280, 720)})
    picam2.configure(config)
    # Apply Hardware Crop
    picam2.set_controls({"ScalerCrop": ZOOM_CROP, "AfMode": 2, "AwbMode": 3}) 
    picam2.start()
    print(f"✅ Hardware & PaddleOCR Initialized")
    
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
    gate_p1 = gate_p2 = sensor = buzzer = DummyDev()
    led_open = led_close = DummyDev()
    picam2 = None 

# ... [Keep your GUI SETUP, HEADER CREATOR, and PAGE 1-3 logic identical] ...
# ... [Keep your LOG_TO_GUI, SMART_GATE_CHECK, and RESET logic identical] ...

def get_paddle_text(img):
    """Parses PaddleOCR nested output into a clean string"""
    try:
        result = paddle_reader.ocr(img, cls=True)
        if not result or result[0] is None:
            return ""
        
        # Extract text from: [[ [box], [text, confidence] ]]
        extracted = [line[1][0] for line in result[0]]
        return "".join(extracted)
    except Exception as e:
        print(f"OCR Error: {e}")
        return ""

def update_frame():
    global detection_start_time, last_plate_seen_time
    try:
        frame = picam2.capture_array() if picam2 else np.zeros((720, 1280, 3), dtype=np.uint8)
    except: root.after(100, update_frame); return

    h, w, _ = frame.shape
    current_time = time.time()
    should_detect = (not is_gate_busy) or (system_state == "POST-ENTRY SCAN")
    
    if should_detect and system_state != "CLOSING GATE":
        t0 = time.perf_counter()
        # Pass frame directly (Hardware crop already handled ROI)
        results = model(frame, verbose=False, conf=0.4) 
        t_detect = (time.perf_counter() - t0) * 1000
        detected_box = False

        for result in results:
            boxes = result.boxes.xyxy.cpu().numpy()
            if len(boxes) > 0:
                detected_box = True; last_plate_seen_time = current_time 
                if detection_start_time is None: detection_start_time = current_time
                
                for box in boxes:
                    x1, y1, x2, y2 = map(int, box)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    
                    if (current_time - detection_start_time) > GRACE_PERIOD:
                        p_crop = frame[y1:y2, x1:x2]
                        if p_crop.size > 0:
                            t_ocr_start = time.perf_counter()
                            
                            # Using PaddleOCR instead of EasyOCR
                            raw_text = get_paddle_text(p_crop)
                            clean_text = "".join([c for c in raw_text.upper() if c.isalnum()])
                            
                            # Your specific plate reformatting logic
                            if len(clean_text) > 0 and clean_text[0].isdigit() and clean_text[-1].isalpha():
                                split_idx = -1
                                for i, char in enumerate(clean_text):
                                    if char.isalpha(): split_idx = i; break
                                if split_idx > 0:
                                    part_digits, part_letters = clean_text[:split_idx], clean_text[split_idx:]
                                    if part_digits.isdigit() and part_letters.isalpha():
                                        clean_text = f"{part_letters}{part_digits}"

                            t_ocr_ms = (time.perf_counter() - t_ocr_start) * 1000
                            
                            if len(clean_text) > 3: 
                                if clean_text not in first_sight_times: first_sight_times[clean_text] = current_time
                                scan_buffer.append(clean_text)
                                most_common, freq = Counter(scan_buffer).most_common(1)[0]
                                
                                if (current_time - logged_vehicles.get(most_common, 0)) > LOG_COOLDOWN:
                                    latency = (t_detect + t_ocr_ms) / 1000.0
                                    
                                    if most_common in authorized_plates:
                                        row = auth_df[auth_df['Plate'] == most_common].iloc[0]
                                        log_to_gui_and_csv(most_common, row['Name'], row['Faculty'], "AUTHORIZED", latency, t_detect, t_ocr_ms)
                                    elif freq >= CONFIDENCE_THRESHOLD:
                                        log_to_gui_and_csv(most_common, "Unknown", "Visitor", "UNAUTHORIZED", latency, t_detect, t_ocr_ms)
                                    
                                    logged_vehicles[most_common] = current_time
                                    scan_buffer.clear(); first_sight_times.clear()

        if not detected_box and detection_start_time and (current_time - last_plate_seen_time) > ABSENCE_RESET_TIME:
            detection_start_time = None; scan_buffer.clear(); first_sight_times.clear()

    # GUI Scaling
    win_w, win_h = video_frame_container.winfo_width(), video_frame_container.winfo_height()
    if win_w > 10 and win_h > 10:
        scale = min(win_w / w, win_h / h)
        img = Image.fromarray(cv2.resize(frame, (int(w*scale), int(h*scale))))
        imgtk = ImageTk.PhotoImage(image=img)
        video_label.imgtk = imgtk; video_label.configure(image=imgtk)

    root.after(10, update_frame)

# Start System
show_frame(page_camera)
load_history_data()
root.after(500, smart_gate_check)
root.after(500, update_frame)
root.mainloop()
