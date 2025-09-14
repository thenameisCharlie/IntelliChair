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
    _INITIALIZED = True # mark setup complete

def distance_cm(timeout_s = 0.03):
    _ensure_init()

    #Send a 10 µs pulse on TRIG (tell sensor to ping).
    GPIO.output(_TRIG_PIN, GPIO.LOW); time.sleep(2e-6)
    GPIO.output(_TRIG_PIN, GPIO.HIGH); time.sleep(10e-6)
    GPIO.output(_TRIG_PIN, GPIO.LOW)

    t_start = time.time() #timestamp recorded just before waiting for the echo pulse to begin (reference point to enforce timeout)
    t_rise = time.time() # the exact time the ECHO pin first went HIGH (start of the echo pulse)
    t_fall = time.time() # the exact time the ECHO pin went LOW again (end of the echo pulse)

    #Wait for ECHO to go HIGH (pulse leaves sensor)
    while GPIO.input(_ECHO_PIN) == 0:
        if time.time() - t_start > timeout_s:
            return None
    
    
    #Wait for ECHO to go LOW (pulse returned)
    while GPIO.input(_ECHO_PIN) == 1:
        if time.time() - t_rise > timeout_s:
            return None
    
    #Convert pulse width to distance
    pulse = t_fall - t_rise # pulse width in seconds

    # speed of sound = 343 m/s. Distance = (time * 343 / 2) meters.
    return pulse * 343.0 * 100.0 / 2.0

#Take multiple ultrasonic readings, filter noise, and return a stable value.
def distance_filtered_cm(n = 5):
    vals = []

    for _ in range(n):
        d = distance_cm()
        if d is not None and 2.0 <= d <= 400.0:
            vals.append(d)
        time.sleep(0.01)
    
    if len(vals) >= 3:
        return None
    
    # Sort for median calculation
    vals.sort()


    if len(vals) >= 3:
        mid = len(vals)//2

        if len(vals) >= 5:
            return sum(vals[mid-1:mid+2]) / 3.0
        
        return vals[mid]
    
    return None

    