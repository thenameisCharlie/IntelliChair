"""
SLAM mapping wrapper for Yahboom G1 Tank
- Launches SLAM node (gmapping or slam_toolbox)
- Subscribes to /scan from LiDAR
- Saves map as map.pgm + map.yaml
"""

import os
import subprocess
import time

MAP_DIR = os.path.join(os.getcwd(), "maps")
MAP_BASENAME = "map"

def start_slam():
    
    #Start SLAM (gmapping or slam_toolbox). 
    #Assumes ROS is already running and LiDAR publishes to /scan.
    
    print("[slam] Starting SLAM...")
    
    return subprocess.Popen(
        ["rosrun", "gmapping", "slam_gmapping", "scan:=/scan"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def teleop_drive():
    
    #Run teleop while SLAM is active.
    
    print("[slam] Launching teleop. Drive robot to explore.")
    subprocess.call(["python3", "navigation/teleop.py"])

def save_map():
    
    #Save map as map.pgm and map.yaml into ./maps
    
    os.makedirs(MAP_DIR, exist_ok=True)
    map_path = os.path.join(MAP_DIR, MAP_BASENAME)
    print(f"[slam] Saving map to {map_path}.pgm/.yaml ...")
    subprocess.call(["rosrun", "map_server", "map_saver", "-f", map_path])

def run_slam_session():
    
    #Main flow: start SLAM, run teleop, save map, stop SLAM.
    
    slam_proc = None
    try:
        slam_proc = start_slam()
        time.sleep(3)  # give SLAM a moment to initialize

        teleop_drive()  # blocking until user quits teleop

        save_map()
    finally:
        if slam_proc:
            slam_proc.terminate()
            print("[slam] SLAM stopped cleanly.")

if __name__ == "__main__":
    run_slam_session()
