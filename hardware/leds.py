# hardware/leds.py
import RPi.GPIO as GPIO
from utils.config import LEDS

_INITIALIZED = False

def _ensure_init():
    global _INITIALIZED
    if _INITIALIZED:
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for p in LEDS.values():
        GPIO.setup(p, GPIO.OUT, initial=GPIO.LOW)  # change to HIGH if your board is active-low
    _INITIALIZED = True

def set_rgb(r=False, g=False, b=False):
    _ensure_init()
    GPIO.output(LEDS["R"], GPIO.HIGH if r else GPIO.LOW)
    GPIO.output(LEDS["G"], GPIO.HIGH if g else GPIO.LOW)
    GPIO.output(LEDS["B"], GPIO.HIGH if b else GPIO.LOW)

def off():      set_rgb(False, False, False)
def red():      set_rgb(True,  False, False)
def green():    set_rgb(False, True,  False)
def blue():     set_rgb(False, False, True)
def yellow():   set_rgb(True,  True,  False)
def magenta():  set_rgb(True,  False, True)
def cyan():     set_rgb(False, True,  True)
