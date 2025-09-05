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
# import RPi.GPIO as GPIO
# import time

# # === Yahboom G1 / BST-4WD per your manual (BCM numbering) ===
# AIN1, AIN2, PWMA = 21, 20, 16   # Left track
# BIN1, BIN2, PWMB = 26, 19, 13   # Right track
# FREQ_HZ = 200    # PWM freq; TB6612FNG is fine at 100–20k

# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)

# for p in (AIN1, AIN2, PWMA, BIN1, BIN2, PWMB):
#     GPIO.setup(p, GPIO.OUT)

# L = GPIO.PWM(PWMA, FREQ_HZ); L.start(0)
# R = GPIO.PWM(PWMB, FREQ_HZ); R.start(0)

# def drive(in1, in2, pwm, pct):
#     """pct in -100..100 (neg = reverse)"""
#     fwd = pct >= 0
#     GPIO.output(in1, GPIO.HIGH if fwd else GPIO.LOW)
#     GPIO.output(in2, GPIO.LOW  if fwd else GPIO.HIGH)
#     pwm.ChangeDutyCycle(min(100, abs(pct)))

# try:
#     print("Forward 2s @ 60%")
#     drive(AIN1, AIN2, L, 60)
#     drive(BIN1, BIN2, R, 60)
#     time.sleep(2)

#     print("Stop 1s")
#     drive(AIN1, AIN2, L, 0)
#     drive(BIN1, BIN2, R, 0)
#     time.sleep(1)

#     print("Spin-in-place left 1.5s @ 50%")
#     drive(AIN1, AIN2, L, -50)
#     drive(BIN1, BIN2, R, 50)
#     time.sleep(1.5)

# finally:
#     L.stop(); R.stop()
#     GPIO.cleanup()
#-*- coding:UTF-8 -*-
import RPi.GPIO as GPIO
import time

#Definition of  motor pin 
IN1 = 20
IN2 = 21
IN3 = 19
IN4 = 26
ENA = 16
ENB = 13

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

#Motor pin initialization operation
def motor_init():
    global pwm_ENA
    global pwm_ENB
    GPIO.setup(ENA,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN1,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN2,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(ENB,GPIO.OUT,initial=GPIO.HIGH)
    GPIO.setup(IN3,GPIO.OUT,initial=GPIO.LOW)
    GPIO.setup(IN4,GPIO.OUT,initial=GPIO.LOW)
    #Set the PWM pin and frequency is 2000hz
    pwm_ENA = GPIO.PWM(ENA, 2000)
    pwm_ENB = GPIO.PWM(ENB, 2000)

#advance
def run():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)
    #PWM duty cycle is set to 100（0--100）
    pwm_ENA.start(50)
    pwm_ENB.start(50)

#delay 2s
time.sleep(2)

#The try/except statement is used to detect errors in the try block.
#the except statement catches the exception information and processes it.
try:
    motor_init()
    while True:
        run()
except KeyboardInterrupt:
    pass
pwm_ENA.stop()
pwm_ENB.stop()
GPIO.cleanup()
