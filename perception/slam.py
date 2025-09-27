"""
SLAM mapping wrapper for Yahboom G1 Tank
- Launches SLAM node (gmapping or slam_toolbox)
- Subscribes to /scan from LiDAR
- Saves map as map.pgm + map.yaml
"""

import os
import subprocess
import signal
import sys
import time
import threading
sys.path.append(os.path.expanduser("~/IntelliChair"))
from navigation import teleop



# Save maps here
MAP_NAME = "map"
slam_proc = None
teleop_proc = None

def start_slam():
    
    #Start SLAM (gmapping or slam_toolbox). 
    #Assumes ROS2 is already running and LiDAR publishes to /scan.
    
    global slam_proc
    print("[slam] Starting SLAM...")
    slam_proc = subprocess.Popen([
        "bash", "-i", "-c",
        "source /opt/ros/humble/setup.bash && "
        "source /home/robotpi/ros2_ws/install/setup.bash && "
        "ros2 run slam_toolbox sync_slam_toolbox_node"
    ])
    time.sleep(5)  # give it time to start up

def start_teleop():
    #Start teleop so user can drive with keyboard.
    global teleop_proc
    print("[slam] Starting teleop (keyboard)...")
    teleop_proc = subprocess.Popen([
        "python3",
        os.path.expanduser("~/IntelliChair/navigation/teleop.py")
    ])
    time.sleep(1)

def save_map():
    
    #Save map as map.pgm and map.yaml into ./maps
    print(f"[slam] Saving map -> {MAP_NAME}.pgm / {MAP_NAME}.yaml")
    try:
        subprocess.run([
            "bash", "-i", "-c",
            "source /opt/ros/humble/setup.bash && "
            "source /home/robotpi/ros2_ws/install/setup.bash && "
            f"ros2 run nav2_map_server map_saver_cli -f {MAP_NAME}"
        ], check=True)
        print("[slam] Map saved successfully.")
    except subprocess.CalledProcessError:
        print("[slam] ERROR: Failed to save map!")


def stop_slam():
    global slam_proc
    if slam_proc:
        print("[slam] Stopping SLAM...")
        slam_proc.send_signal(signal.SIGINT)
        slam_proc.wait()
        print("[slam] SLAM stopped.")

def stop_teleop():
    global teleop_proc
    if teleop_proc:
        print("[slam] Stopping teleop...")
        teleop_proc.send_signal(signal.SIGINT)
        teleop_proc.wait()
        print("[slam] Teleop stopped.")



def run_slam_session():
    
    #Main flow: start SLAM, run teleop, save map, stop SLAM
    try:
        start_slam()
        start_teleop()
        print("[slam] SLAM + teleop running. Drive robot to explore.")
        print("Press Ctrl+C to stop and save map.")
        while True:
            if slam_proc.poll() is not None:
                print("[slam] SLAM process ended unexpectedly.")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[slam] Stopping session...")
    finally:
        stop_teleop()
        stop_slam()
        save_map()
        print("[slam] Session complete.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        MAP_NAME = sys.argv[1]
    run_slam_session()
