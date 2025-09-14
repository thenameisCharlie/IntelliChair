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

def autonomy_loop():
    motors = YahboomMotors()

    try:
        while True:
            distance = distance_filtered_cm()

            if distance is None:
                if motors:  # show we're stopping because no reading
                    print("[autonomy] ultrasonic: no reading → stop")
                    motors.stop()
                time.sleep(LOOP_DT)
                continue

            print(f"[autonomy] ultrasonic: {distance:.1f} cm")

            if distance < THRESH_CM:
                print("[autonomy] EMERGENCY: obstacle close → spin-right + stop")
                motors.stop()
                motors.spin_right(SPIN_SPEED)
                time.sleep(SPIN_TIME_S)
                motors.stop()
                time.sleep(LOOP_DT)
                continue

            if SLOW_BAND[0] <= distance <= SLOW_BAND[1]:
                print(f"[autonomy] slow band ({SLOW_BAND}) → forward {SLOW_SPEED}%")
                motors.forward(SLOW_SPEED)
            else:
                print(f"[autonomy] cruise → forward {CRUISE_SPEED}%")
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





# def main():
#     # Simple read loop
#     for _ in range(20):
#         d = distance_filtered_cm()
#         if d is None:
#             print("Ultrasonic: no reading")
#         else:
#             print(f"Ultrasonic: {d:.1f} cm")
#         time.sleep(0.1)

# if __name__ == "__main__":
#     try:
#         main()
#     finally:
#         # If you only used ultrasonic, no GPIO cleanup needed here—the motor class handles its own.
#         pass