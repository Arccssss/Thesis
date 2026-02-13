import gpiod
import time

# ON RASPBERRY PI 5:
# The 40-pin header is usually on 'gpiochip4'.
# You can check this by running `gpiodetect` in your terminal.
CHIP_PATH = '/dev/gpiochip4'
TRIG_OFFSET = 23
ECHO_OFFSET = 24

def get_distance_gpiod():
    with gpiod.Chip(CHIP_PATH) as chip:
        # 1. SETUP LINES
        # Request TRIG as output, ECHO as input with "edge detection"
        trig_line = chip.get_line(TRIG_OFFSET)
        echo_line = chip.get_line(ECHO_OFFSET)
        
        trig_line.request(consumer="HC-SR04", type=gpiod.LINE_REQ_DIR_OUT)
        
        # We request BOTH_EDGES to catch the start (Rising) and end (Falling) of the echo
        echo_line.request(consumer="HC-SR04", type=gpiod.LINE_REQ_EV_BOTH_EDGES)

        # 2. TRIGGER PULSE
        trig_line.set_value(0)
        time.sleep(0.000005)
        trig_line.set_value(1)
        time.sleep(0.00001) # 10 microsecond pulse
        trig_line.set_value(0)

        # 3. WAIT FOR EVENTS (Kernel Timestamping)
        start_ns = 0
        end_ns = 0
        
        # We expect 2 events: Rising Edge (Start) and Falling Edge (End)
        # We loop to catch both.
        for _ in range(2):
            if echo_line.event_wait(timedelta=time.timedelta(seconds=0.1)): # Wait up to 100ms
                event = echo_line.event_read()
                
                if event.type == gpiod.LineEvent.RISING_EDGE:
                    start_ns = event.timestamp.nanoseconds
                elif event.type == gpiod.LineEvent.FALLING_EDGE:
                    end_ns = event.timestamp.nanoseconds
            else:
                return -1 # Timeout

        # 4. CALCULATE
        if end_ns > start_ns:
            # Time difference in seconds
            duration_s = (end_ns - start_ns) / 1_000_000_000.0
            
            # Speed of sound: 34300 cm/s
            distance = (duration_s * 34300) / 2
            return distance
        else:
            return -1

# --- TEST LOOP ---
try:
    print("Testing with gpiod (Kernel Timestamps)...")
    while True:
        d = get_distance_gpiod()
        if d != -1:
            print(f"Distance: {d:.2f} cm")
        else:
            print("Timeout / Error")
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
