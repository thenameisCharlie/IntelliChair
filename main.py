import time
from hardware.ultrasonic import distance_filtered_cm

def main():
    # Simple read loop
    for _ in range(20):
        d = distance_filtered_cm()
        if d is None:
            print("Ultrasonic: no reading")
        else:
            print(f"Ultrasonic: {d:.1f} cm")
        time.sleep(0.1)

if __name__ == "__main__":
    try:
        main()
    finally:
        # If you only used ultrasonic, no GPIO cleanup needed here—the motor class handles its own.
        pass