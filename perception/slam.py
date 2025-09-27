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

MAP_NAME = "map"

slam_proc = None

def start_slam():
    
    #Start SLAM (gmapping or slam_toolbox). 
    #Assumes ROS2 is already running and LiDAR publishes to /scan.
    
    global slam_proc
    print("[slam] Starting SLAM...")
    slam_proc = subprocess.Popen([
        "bash", "-i", "-c",
        "source /opt/ros/humble/setup.bash && "
        "source ~/ros2_ws/install/setup.bash && "
        "ros2 run slam_toolbox sync_slam_toolbox_node"
    ])
    time.sleep(5)  # give it time to start up

#def teleop_drive():
    
    #Run teleop while SLAM is active.
    
    #print("[slam] Launching teleop. Drive robot to explore.")
    #subprocess.call(["python3", "navigation/teleop.py"])

def save_map():
    
    #Save map as map.pgm and map.yaml into ./maps
    
    print(f"[slam] Saving map -> {MAP_NAME}.pgm / {MAP_NAME}.yaml")
    try:
        subprocess.run([
            "bash", "-i", "-c",
            "source /opt/ros/humble/setup.bash && "
            "source ~/ros2_ws/install/setup.bash && "
            f"ros2 run nav2_map_server map_saver_cli -f {MAP_NAME}"
        ], check=True)
        print("[slam] Map saved successfully.")
    except subprocess.CalledProcessError:
        print("[slam] ERROR: Failed to save map!")

def stop_slam():
    #Stop SLAM process cleanly
    global slam_proc
    if slam_proc:
        print("[slam] Stopping SLAM...")
        slam_proc.send_signal(signal.SIGINT)
        slam_proc.wait()
        print("[slam] SLAM stopped.")

def run_slam_session():
    
    #Main flow: start SLAM, run teleop, save map, stop SLAM.
    
    try:
        start_slam()
        print("[slam] SLAM running. Use teleop to drive the robot.")
        print("Press Ctrl+C when finished to save map and exit.")
        while True:
            if slam_proc.poll() is not None:
                print("[slam] SLAM process ended unexpectedly.")
                break
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[slam] Stopping session...")
    finally:
        save_map()
        stop_slam()
        print("[slam] Session complete.")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        MAP_NAME = sys.argv[1]
    run_slam_session()
