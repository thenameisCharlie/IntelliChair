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

def ensure_map_dir():
    if not os.path.exists(MAP_DIR):
        os.makedirs(MAP_DIR)

def start_slam():
    #Launch slam_toolbox or gmapping as a subprocess.
    #Popen (non-blocking)
    env = os.environ.copy()
    env["PATH"] = "/opt/ros/humble/bin:" + env["PATH"]

    return subprocess.Popen(
        ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def save_map(basename=MAP_BASENAME):
    #Call map_saver to write out map.pgm + map.yaml.
    ensure_map_dir()
    map_path = os.path.join(MAP_DIR, basename)
    print(f"[slam] Saving map → {map_path}.pgm/.yaml ...")
    subprocess.run([
        "ros2", "run", "nav2_map_server", "map_saver_cli",
        "-f", map_path
    ])
    print("[slam] Map saved.")

def load_map(basename=MAP_BASENAME):
    #Launch nav2_map_server to reload a saved map.
    map_path = os.path.join(MAP_DIR, basename + ".yaml")
    if not os.path.exists(map_path):
        print(f"[slam] ERROR: Map {map_path} not found.")
        return None
    print(f"[slam] Loading map {map_path} ...")
    return subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "bringup_launch.py",
        "map:=" + map_path
    ])

if __name__ == "__main__":
    # Demo: start SLAM, wait, then save map
    slam_proc = start_slam()
    try:
        print("[slam] Running... Drive robot around with teleop or autonomy.")
        while True:
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n[slam] Ctrl+C → stopping and saving map.")
        save_map()
        slam_proc.terminate()
        slam_proc.wait()

