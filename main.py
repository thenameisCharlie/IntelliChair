import time
from hardware.ultrasonic import distance_filtered_cm
from hardware.motors import YahboomMotors
from hardware.servo import set_angle, center
from utils.config import TUNABLES
from perception.slam import start_slam, save_map


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
    cyan()
    motors.stop()

    turn = pick_turn_direction(distance_filtered_cm, near=THRESH_CM)
    if turn == "left":
        blue()
        motors.spin_left(SPIN_SPEED);  time.sleep(SPIN_TIME_S)
    elif turn == "right":
        magenta()
        motors.spin_right(SPIN_SPEED); time.sleep(SPIN_TIME_S)
    else:
        motors.backward(40); time.sleep(0.35); motors.stop()
        if prefer_right:
            blue()
            motors.spin_left(SPIN_SPEED);  time.sleep(SPIN_TIME_S*1.3)
        else:
            magenta()
            motors.spin_right(SPIN_SPEED); time.sleep(SPIN_TIME_S*1.3)
    motors.stop(); time.sleep(0.05)
    green()


#
def autonomy_loop():
    print("[debug] autonomy_loop started")

    motors = YahboomMotors()

    try:
        while True:
            d = distance_filtered_cm()

            if d is None:
                red()
                print("[autonomy] ultrasonic: no reading → stop")
                motors.stop()
                time.sleep(LOOP_DT)
                continue

            print(f"[autonomy] ultrasonic: {d:.1f} cm")

            # Emergency: very close
            if d < STOP_CM:
                red()
                print("[autonomy] EMERGENCY: obstacle close → avoid")
                avoid_blocked(motors)
                time.sleep(LOOP_DT)
                continue

            elif d <= RESUME_CM:
                yellow()
                # When it reaches 30–38 cm don’t blast forward
                print(f"[autonomy] ({STOP_CM}, {RESUME_CM}) slow {SLOW_SPEED}%")
                motors.forward(SLOW_SPEED)

            elif SLOW_BAND[0] <= d <= SLOW_BAND[1]:
                # regular slow-approach zone
                yellow()
                print(f"[autonomy] slow band {SLOW_BAND} forward {SLOW_SPEED}%")
                motors.forward(SLOW_SPEED)

            else:
                green()
                print(f"[autonomy] cruise forward {CRUISE_SPEED}%")
                motors.forward(CRUISE_SPEED)


            time.sleep(LOOP_DT)
    finally:
        off()
        motors.shutdown()


def slam_autonomy():
    slam_proc = start_slam()
    try:
        print("[autonomy+slam] Exploring... (Ctrl+C to stop)")
        autonomy_loop()   #avoid/drive loop
    except KeyboardInterrupt:
        print("\n[autonomy+slam] Stopping SLAM + saving map.")
        save_map()
    finally:
        slam_proc.terminate()
        slam_proc.wait()

if __name__ == "__main__":
    try:
        #print("[autonomy] Motors + ultrasonic running together. Ctrl+C to stop.")
        #autonomy_loop()
        print("[slam_autonomy] Motors + ultrasonic running together. Ctrl+C to stop.")
        slam_autonomy()
    except KeyboardInterrupt:
        print("\nExiting cleanly.")
