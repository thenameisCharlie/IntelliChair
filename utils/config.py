
#BCM numbering linked to the hardware channels on Raspberry Pi's Broadcom microprocessor
PINS = {"IN1": 20, "IN2": 21, "IN3": 19, "IN4": 26, "ENA": 16, "ENB": 13} # Motor pins
ULTRASONIC_PINS = {"TRIG": 1, "ECHO": 0} # Ultrasonic pins
LEDS = {"R": 22, "G": 27, "B": 24} # LEDs
SERVO = {"PIN": 23} # Servo pin
TUNABLES = {"THRESH_CM": 40, "CRUISE_SPEED": 37, "SLOW_SPEED": 32, "SLOW_BAND": (30, 45), "SPIN_SPEED": 45, "SPIN_TIME_S": 0.30, "LOOP_DT": 0.05}

PWM_FREQ_HZ = 2000 #the rate at which the signal repeats its on/off cycle (Hz)

# RGB LED pins (BCM numbering)
LEDS = {"R": 22, "G": 27, "B": 24}

THRESH_CM    = 25   # stop range
SLOW_BAND    = 30   # slow zone above stop
CRUISE_SPEED = 35   # slower cruise = shorter stopping distance
SLOW_SPEED   = 20


