# # tests/test_motor.py
# import RPi.GPIO as GPIO
# import time

# # TODO: move these to a config later (profiles/site1/pins.yaml)
# AIN1, AIN2, PWMA = 5, 6, 12      # Left
# BIN1, BIN2, PWMB = 13, 19, 18    # Right
# FREQ_HZ = 100

# GPIO.setmode(GPIO.BCM)
# for p in (AIN1, AIN2, PWMA, BIN1, BIN2, PWMB):
#     GPIO.setup(p, GPIO.OUT)

# left_pwm  = GPIO.PWM(PWMA, FREQ_HZ);  left_pwm.start(0)
# right_pwm = GPIO.PWM(PWMB, FREQ_HZ); right_pwm.start(0)

# def drive(in1, in2, pwm, pct):
#     fwd = pct >= 0
#     GPIO.output(in1, GPIO.HIGH if fwd else GPIO.LOW)
#     GPIO.output(in2, GPIO.LOW  if fwd else GPIO.HIGH)
#     pwm.ChangeDutyCycle(min(100, abs(pct)))

# try:
#     print("Forward 2s @ 50%"); drive(AIN1,AIN2,left_pwm,50); drive(BIN1,BIN2,right_pwm,50); time.sleep(2)
#     print("Stop 1s");         drive(AIN1,AIN2,left_pwm,0);   drive(BIN1,BIN2,right_pwm,0);   time.sleep(1)
#     print("Spin L 1.5s @40%");drive(AIN1,AIN2,left_pwm,-40); drive(BIN1,BIN2,right_pwm,40);  time.sleep(1.5)
# finally:
#     left_pwm.stop(); right_pwm.stop(); GPIO.cleanup()

# tests/test_motor.py
import RPi.GPIO as GPIO
import time

# === Yahboom G1 / BST-4WD per your manual (BCM numbering) ===
AIN1, AIN2, PWMA = 21, 20, 16   # Left track
BIN1, BIN2, PWMB = 26, 19, 13   # Right track
FREQ_HZ = 200    # PWM freq; TB6612FNG is fine at 100–20k

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for p in (AIN1, AIN2, PWMA, BIN1, BIN2, PWMB):
    GPIO.setup(p, GPIO.OUT)

L = GPIO.PWM(PWMA, FREQ_HZ); L.start(0)
R = GPIO.PWM(PWMB, FREQ_HZ); R.start(0)

def drive(in1, in2, pwm, pct):
    """pct in -100..100 (neg = reverse)"""
    fwd = pct >= 0
    GPIO.output(in1, GPIO.HIGH if fwd else GPIO.LOW)
    GPIO.output(in2, GPIO.LOW  if fwd else GPIO.HIGH)
    pwm.ChangeDutyCycle(min(100, abs(pct)))

try:
    print("Forward 2s @ 60%")
    drive(AIN1, AIN2, L, 60)
    drive(BIN1, BIN2, R, 60)
    time.sleep(2)

    print("Stop 1s")
    drive(AIN1, AIN2, L, 0)
    drive(BIN1, BIN2, R, 0)
    time.sleep(1)

    print("Spin-in-place left 1.5s @ 50%")
    drive(AIN1, AIN2, L, -50)
    drive(BIN1, BIN2, R, 50)
    time.sleep(1.5)

finally:
    L.stop(); R.stop()
    GPIO.cleanup()
