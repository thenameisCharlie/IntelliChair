import time
from hardware.ultrasonic import distance_filtered_cm
from hardware.motors import YahboomMotors
from hardware.servo import set_angle, center
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

#Aim servo to right/center/left, sample distances, pick the best side.
def pick_turn_direction(sample_fn, near=THRESH_CM):
    readings=[]
    for angle,label in [(45,"right"),(90,"center"),(135,"left")]:
        set_angle(angle)
        d = sample_fn()
        readings.append((label, d if d is not None else 0))
    center()
    best = max(readings, key=lambda x:x[1])
    return best[0] if best[1] >= near else "none"


def avoid_blocked(motors, prefer_right=True):
    motors.stop()
    turn = pick_turn_direction(distance_filtered_cm, near=THRESH_CM)
    if turn == "left":
        motors.spin_left(SPIN_SPEED);  time.sleep(SPIN_TIME_S)
    elif turn == "right":
        motors.spin_right(SPIN_SPEED); time.sleep(SPIN_TIME_S)
    else:
        motors.backward(40); time.sleep(0.35); motors.stop()
        if prefer_right:
            motors.spin_left(SPIN_SPEED);  time.sleep(SPIN_TIME_S*1.3)
        else:
            motors.spin_right(SPIN_SPEED); time.sleep(SPIN_TIME_S*1.3)
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


if __name__ == "__main__":
    try:
        print("[autonomy] Motors + ultrasonic running together. Ctrl+C to stop.")
        autonomy_loop()
    except KeyboardInterrupt:
        print("\nExiting cleanly.")
