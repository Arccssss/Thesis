import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice

# --- SETUP (Use the specific pins from your diagram) ---
TRIG_PIN = 23
ECHO_PIN = 24

# We use DigitalOutputDevice/InputDevice just for safe raw pin control on Pi 5
trig = DigitalOutputDevice(TRIG_PIN)
echo = DigitalInputDevice(ECHO_PIN)

def get_distance_manual():
    # 1. TRIGGER PULSE
    # Ensure trigger is low initially
    trig.off()
    time.sleep(0.000005) # Tiny pause to let signal settle
    
    # Send 10 microsecond pulse
    trig.on()
    time.sleep(0.00001) 
    trig.off()
    
    # 2. WAIT FOR ECHO START (Timeout safety included)
    timeout = time.time() + 0.04 # 40ms timeout (max range ~6m)
    while echo.value == 0:
        pulse_start = time.perf_counter() # More precise than time.time()
        if time.time() > timeout:
            return -1 # Return -1 if sensor times out (no object)

    # 3. WAIT FOR ECHO END
    while echo.value == 1:
        pulse_end = time.perf_counter()
        if time.time() > timeout:
            return -1

    # 4. THE MANUAL MATH
    # Calculate duration of the pulse
    pulse_duration = pulse_end - pulse_start
    
    # Speed of Sound is ~34300 cm/s
    # Distance = (Time * Speed) / 2 (because sound goes there AND back)
    distance_cm = (pulse_duration * 34300) / 2
    
    return distance_cm

# --- TEST LOOP ---
try:
    print("Testing Manual Distance...")
    while True:
        dist = get_distance_manual()
        if dist == -1:
            print("Range Error / Timeout")
        else:
            print(f"Measured: {dist:.2f} cm")
        time.sleep(0.5)

except KeyboardInterrupt:
    print("Stopped")
