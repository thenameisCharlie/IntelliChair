# tests/test_motor.py
import RPi.GPIO as GPIO
import time

# TODO: move these to a config later (profiles/site1/pins.yaml)
AIN1, AIN2, PWMA = 5, 6, 12      # Left
BIN1, BIN2, PWMB = 13, 19, 18    # Right
FREQ_HZ = 100

GPIO.setmode(GPIO.BCM)
for p in (AIN1, AIN2, PWMA, BIN1, BIN2, PWMB):
    GPIO.setup(p, GPIO.OUT)

left_pwm  = GPIO.PWM(PWMA, FREQ_HZ);  left_pwm.start(0)
right_pwm = GPIO.PWM(PWMB, FREQ_HZ); right_pwm.start(0)

def drive(in1, in2, pwm, pct):
    fwd = pct >= 0
    GPIO.output(in1, GPIO.HIGH if fwd else GPIO.LOW)
    GPIO.output(in2, GPIO.LOW  if fwd else GPIO.HIGH)
    pwm.ChangeDutyCycle(min(100, abs(pct)))

try:
    print("Forward 2s @ 50%"); drive(AIN1,AIN2,left_pwm,50); drive(BIN1,BIN2,right_pwm,50); time.sleep(2)
    print("Stop 1s");         drive(AIN1,AIN2,left_pwm,0);   drive(BIN1,BIN2,right_pwm,0);   time.sleep(1)
    print("Spin L 1.5s @40%");drive(AIN1,AIN2,left_pwm,-40); drive(BIN1,BIN2,right_pwm,40);  time.sleep(1.5)
finally:
    left_pwm.stop(); right_pwm.stop(); GPIO.cleanup()
