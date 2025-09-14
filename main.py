import time
from hardware.ultrasonic import distance_filtered_cm
from hardware.motors import YahboomMotors
from utils.config import TUNABLES


THRESH_CM = TUNABLES["THRESH_CM"]
CRUISE_SPEED = TUNABLES["CRUISE_SPEED"]
SLOW_SPEED = TUNABLES["SLOW_SPEED"]
SLOW_BAND = TUNABLES["SLOW_BAND"]
SPIN_SPEED = TUNABLES["SPIN_SPEED"]
SPIN_TIME_S = TUNABLES["SPIN_TIME_S"]
LOOP_DT = TUNABLES["LOOP_DT"]
STOP_CM   = THRESH_CM           # e.g. 30
RESUME_CM = STOP_CM + 8         # only resume when > 38

#
def avoid_blocked(motors: YahboomMotors, prefer_right=True)-> None:
    
    motors.stop()
    if prefer_right:
        motors.spin_right(SPIN_SPEED); time.sleep(SPIN_TIME_S)
    else:
        motors.spin_left(SPIN_SPEED);  time.sleep(SPIN_TIME_S)
    motors.stop(); time.sleep(0.05)

    d2 = distance_filtered_cm()
    if d2 is None or d2 < THRESH_CM:
        # still blocked → back up and try the other direction
        motors.backward(40); time.sleep(0.35); motors.stop()
        if prefer_right:
            motors.spin_left(SPIN_SPEED);  time.sleep(SPIN_TIME_S * 1.3)
        else:
            motors.spin_right(SPIN_SPEED); time.sleep(SPIN_TIME_S * 1.3)
        motors.stop(); time.sleep(0.05)

#
def autonomy_loop():
    motors = YahboomMotors()

    try:
        while True:
            d = distance_filtered_cm()

            if d is None:
                print("[autonomy] ultrasonic: no reading → stop")
                motors.stop()
                time.sleep(LOOP_DT)
                continue

            print(f"[autonomy] ultrasonic: {d:.1f} cm")

            # Emergency: very close
            if d < STOP_CM:
                print("[autonomy] EMERGENCY: obstacle close → avoid")
                avoid_blocked(motors)

            elif d <= RESUME_CM:
                # When it reaches 30–38 cm don’t blast forward
                print(f"[autonomy] ({STOP_CM}, {RESUME_CM}) slow {SLOW_SPEED}%")
                motors.forward(SLOW_SPEED)

            elif SLOW_BAND[0] <= d <= SLOW_BAND[1]:
                # regular slow-approach zone
                print(f"[autonomy] slow band {SLOW_BAND} forward {SLOW_SPEED}%")
                motors.forward(SLOW_SPEED)

            else:
                print(f"[autonomy] cruise forward {CRUISE_SPEED}%")
                motors.forward(CRUISE_SPEED)


            time.sleep(LOOP_DT)
    finally:
        motors.shutdown()


# def autonomy_loop():
#     motors = YahboomMotors()
#     stuck_count = 0
#     prefer_right = True

#     try:
#         while True:
#             d = distance_filtered_cm()

#             if d is None:
#                 print("[autonomy] ultrasonic: no reading → stop")
#                 motors.stop()
#                 time.sleep(LOOP_DT)
#                 continue

#             print(f"[autonomy] ultrasonic: {d:.1f} cm")

#             # Emergency: very close
#             if d < THRESH_CM:
#                 print("[autonomy] EMERGENCY: obstacle close → avoid")
#                 avoid_blocked(motors, prefer_right=prefer_right)
#                 stuck_count += 1
#                 # Flip preference if we’ve been stuck a few times
#                 if stuck_count >= 3:
#                     prefer_right = not prefer_right
#                     stuck_count = 0
#                 time.sleep(LOOP_DT)
#                 continue

#             # Clear path: reset stuck counter
#             stuck_count = 0

#             # Approach band: slow down
#             if SLOW_BAND[0] <= d <= SLOW_BAND[1]:
#                 print(f"[autonomy] slow band {SLOW_BAND} → forward {SLOW_SPEED}%")
#                 motors.forward(SLOW_SPEED)
#             else:
#                 print(f"[autonomy] cruise → forward {CRUISE_SPEED}%")
#                 motors.forward(CRUISE_SPEED)

#             time.sleep(LOOP_DT)
#     finally:
#         motors.shutdown()


if __name__ == "__main__":
    try:
        print("[autonomy] Motors + ultrasonic running together. Ctrl+C to stop.")
        autonomy_loop()
    except KeyboardInterrupt:
        print("\nExiting cleanly.")
