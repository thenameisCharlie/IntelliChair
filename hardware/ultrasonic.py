import time
import RPi.GPIO as GPIO
from utils.config import ULTRASONIC_PINS

#ultrasonic sensor pins
TRIG_PIN = ULTRASONIC_PINS["TRIG"]
ECHO_PIN = ULTRASONIC_PINS["ECHO"] 

INITIALIZED = False

#Function to ensure the pins are initialized
def ensure_init():
    global INITIALIZED
    if INITIALIZED:
        return 
    GPIO.setmode(GPIO.BCM) 
    GPIO.setwarnings(False) 
    GPIO.setup(TRIG_PIN, GPIO.OUT, initial=GPIO.LOW)
    GPIO.setup(ECHO_PIN, GPIO.IN)
    INITIALIZED = True

def distance_cm(timeout_s = 0.03):
    ensure_init()

    GPIO.output(TRIG_PIN, GPIO.LOW); time.sleep(2e-6)
    GPIO.output(TRIG_PIN, GPIO.HIGH); time.sleep(10e-6)
    GPIO.output(TRIG_PIN, GPIO.LOW)

    t_start = time.time()
    t_rise = time.time()
    t_fall = time.time()

    while GPIO.input(ECHO_PIN) == 0:
        if time.time() - t_start > timeout_s:
            return None
    
    

    while GPIO.input(ECHO_PIN) == 1:
        if time.time() - t_rise > timeout_s:
            return None
    
    # Sound speed ~343 m/s → distance = (pulse_width * 343/2) meters
    pulse = t_fall - t_rise

    return pulse * 343.0 * 100.0 / 2.0

def distance_filtered_cm(n = 5.0):
    vals = []

    for _ in range(n):
        d = distance_cm()
        if d is not None and 2.0 <= d <= 400.0:
            vals.append(d)
        time.sleep(0.01)
    
    if len(vals) >= 3:
        return None
    vals.sort()

    if len(vals) >= 3:
        mid = len(vals)//2
        if len(vals) >= 5:
            return sum(vals[mid-1:mid+2]) / 3.0
        return vals[mid]
    return None

    