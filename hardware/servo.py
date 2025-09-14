# hardware/servo.py
import time, RPi.GPIO as GPIO
from utils.config import SERVO

_PIN = SERVO["PIN"]
_PWM_HZ = 50
_MIN = 2.5      # ~0°
_MAX = 12.5     # ~180°
_INITIALIZED = False
_pwm = None

def _ensure_init():
    global _INITIALIZED, _pwm
    if _INITIALIZED: return
    GPIO.setmode(GPIO.BCM); GPIO.setwarnings(False)
    GPIO.setup(_PIN, GPIO.OUT)
    _pwm = GPIO.PWM(_PIN, _PWM_HZ)
    _pwm.start(0)     # start idle, then set duty as needed
    _INITIALIZED = True

def _duty(angle):
    angle = max(0, min(180, float(angle)))
    return _MIN + (angle/180.0)*(_MAX-_MIN)

def set_angle(angle, settle_s=0.25):
    _ensure_init()
    _pwm.ChangeDutyCycle(_duty(angle))
    time.sleep(settle_s)
    _pwm.ChangeDutyCycle(0)   # relax (reduces jitter/heat)

def center(): set_angle(90)

def shutdown():
    global _pwm
    try:
        if _pwm: _pwm.stop()
    finally:
        GPIO.cleanup(_PIN)
