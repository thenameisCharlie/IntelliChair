# Purpose: Handle distance measurement using an HC-SR04-style ultrasonic sensor.
import time
import RPi.GPIO as GPIO
from utils.config import ULTRASONIC_PINS

#ultrasonic sensor pins
_TRIG_PIN = ULTRASONIC_PINS["TRIG"]
_ECHO_PIN = ULTRASONIC_PINS["ECHO"] 

_INITIALIZED = False

#Makes sure the GPIO pins for TRIG and ECHO are set up exactly once
def _ensure_init():
    global _INITIALIZED
    if _INITIALIZED:
        return 
    GPIO.setmode(GPIO.BCM) 
    GPIO.setwarnings(False) # hide re-use warnings
    GPIO.setup(_TRIG_PIN, GPIO.OUT, initial=GPIO.LOW) # TRIG idles LOW
    GPIO.setup(_ECHO_PIN, GPIO.IN) # ECHO waits for sensor signal
    # GPIO.setup(_ECHO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    _INITIALIZED = True # mark setup complete

def distance_cm(timeout_s=0.12):
    _ensure_init()

    # If ECHO is high already, wait briefly for it to drop (BCM0/1 can idle high)
    t_deadline = time.time() + timeout_s
    while GPIO.input(_ECHO_PIN) == 1:
        if time.time() > t_deadline:
            return None
        time.sleep(0.001)

    #Send a 10 µs pulse on TRIG (tell sensor to ping)
    GPIO.output(_TRIG_PIN, GPIO.LOW); time.sleep(5e-6)
    GPIO.output(_TRIG_PIN, GPIO.HIGH); time.sleep(10e-6)
    GPIO.output(_TRIG_PIN, GPIO.LOW)

    #Wait for ECHO to go HIGH (pulse leaves sensor)
    t_wait_start = time.time()
    while GPIO.input(_ECHO_PIN) == 0:
        if time.time() - t_wait_start > timeout_s:
            return None

    t_rise = time.time()  # <-- capture WHEN it actually went HIGH

    #Wait for ECHO to go LOW (pulse returned)
    while GPIO.input(_ECHO_PIN) == 1:
        if time.time() - t_rise > timeout_s:   # timeout counted from the rise now
            return None

    t_fall = time.time()  # <-- capture WHEN it actually went LOW

    #Convert pulse width to distance
    pulse = t_fall - t_rise              # seconds
    return pulse * 343.0 * 100.0 / 2.0   # cm

#Take multiple ultrasonic readings, filter noise, and return a stable value.
def distance_filtered_cm(n: int = 5) -> float | None:
    vals = []
    
    for _ in range(n):
        d = distance_cm(timeout_s=0.12)   # give BCM0/1 some slack
        if d is not None and 2.0 <= d <= 400.0:
            vals.append(d)
        time.sleep(0.01)
    
    if len(vals) < 3:
        return None
    
    vals.sort()
    mid = len(vals) // 2
    
    if len(vals) >= 5:
        return (vals[mid-1] + vals[mid] + vals[mid+1]) / 3.0
    return vals[mid]

    