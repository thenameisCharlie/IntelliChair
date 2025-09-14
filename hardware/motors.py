# Purpose: Safe, reusable motor driver for the Yahboom G1
import time
import RPi.GPIO as GPIO
from utils.config import PINS, PWM_FREQ_HZ

class YahboomMotors:
    #constructor Initialize GPIO mode, configure pins, and start PWM at 0%
    def __init__(self, pins=PINS, pwm_freq=PWM_FREQ_HZ):
        self.p = pins

        #Use BCM numbering and suppress warnings 
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Configure the 4 direction pins as outputs, default LOW (brake)
        for k in ["IN1", "IN2", "IN3", "IN4"]:
            GPIO.setup(self.p[k], GPIO.OUT, initial=GPIO.LOW)

        #Configure the 2 PWM pins as outputs
        for k in ["ENA", "ENB"]:
            GPIO.setup(self.p[k], GPIO.OUT)

        # Create two PWM channels (left=ENA, right=ENB) and start at 0% duty
        self.pwma = GPIO.PWM(self.p["ENA"], pwm_freq)  # left tread power
        self.pwmb = GPIO.PWM(self.p["ENB"], pwm_freq)  # right tread power
        self.pwma.start(0)
        self.pwmb.start(0)
    
    #Set left tread direction via IN1/IN2
    def _left_dir(self, forward: bool):
        GPIO.output(self.p["IN1"], GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(self.p["IN2"], GPIO.LOW  if forward else GPIO.HIGH)

    #Set right tread direction via IN3/IN4
    def _right_dir(self, forward: bool):
        GPIO.output(self.p["IN3"], GPIO.HIGH if forward else GPIO.LOW)
        GPIO.output(self.p["IN4"], GPIO.LOW  if forward else GPIO.HIGH)

    #Set PWM duty cycle (0-100) on each tread
    #This is power %, not exact speed. Higher % -> more power -> faster
    def _set_speed(self, left_pct, right_pct):
        L = max(0, min(100, int(left_pct)))   # clamp to [0,100]
        R = max(0, min(100, int(right_pct)))
        self.pwma.ChangeDutyCycle(L)
        self.pwmb.ChangeDutyCycle(R)
    
    #Brake (all INx LOW) and cut PWM to 0%
    def stop(self):
        for k in ["IN1","IN2","IN3","IN4"]:
            GPIO.output(self.p[k], GPIO.LOW)
        self._set_speed(0, 0)

    #Both treads forward at the same power
    def forward(self, speed_pct=50):
        self._left_dir(True); self._right_dir(True)
        self._set_speed(speed_pct, speed_pct)

    #Both treads backward at the same power
    def backward(self, speed_pct=50):
        self._left_dir(False); self._right_dir(False)
        self._set_speed(speed_pct, speed_pct)

    #Nudge left tread while right stays idle (arc turn)
    def left(self,speed_pct=50, forward=True):
        self._left_dir(forward)
        self._set_speed(speed_pct, 0)
    
    #Nudge right tread while left stays idle (arc turn)
    def right(self, speed_pct=50, forward=True):
        self._right_dir(forward)
        self._set_speed(0, speed_pct)

    #Spin in place: left backward, right forward
    def spin_left(self, speed_pct=55):
        self._left_dir(False); self._right_dir(True)
        self._set_speed(speed_pct, speed_pct)

    #Spin in place: left forward, right backward
    def spin_right(self, speed_pct=55):
        self._left_dir(True);  self._right_dir(False)
        self._set_speed(speed_pct, speed_pct)

    #Cleanly stop motors, stop PWM, and release GPIO pins
    def shutdown(self):
        try:
            self.stop()
            self.pwma.stop(); self.pwmb.stop()
        finally:
            GPIO.cleanup()



    



    



