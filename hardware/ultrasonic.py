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

