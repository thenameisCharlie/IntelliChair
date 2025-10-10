"""
perception/slam.py
SLAM mapping wrapper for Yahboom G1 Tank

- Launches SLAM node (slam_toolbox)
- Saves map as map.pgm + map.yaml
"""

import os
import subprocess
import time
from typing import Tuple
import threading
from perception.lidar import get_pose, lidar_thread

_lidar_thread_started = False
_lidar_thread_lock = threading.Lock()

# repo-relative map directory
MAP_DIR = os.path.join(os.getcwd(), "maps")
MAP_BASENAME = "map"

def ensure_map_dir():
    if not os.path.exists(MAP_DIR):
        os.makedirs(MAP_DIR, exist_ok=True)

def start_slam():
    #launch slam_toolbox online_async as a subprocess (non-blocking).
    #requires ROS2 environment on the Pi.
    #returns subprocess.Popen
    env = os.environ.copy()
    env["PATH"] = "/opt/ros/humble/bin:" + env.get("PATH", "")
    print("[slam] Starting SLAM (slam_toolbox online_async)...")
    return subprocess.Popen(
        ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def save_map(basename=MAP_BASENAME):
    #call nav2 map_saver_cli to write map to MAP_DIR/<basename>.pgm/.yaml
    ensure_map_dir()
    map_path = os.path.join(MAP_DIR, basename)
    print(f"[slam] Saving map → {map_path}.pgm/.yaml ...")
    try:
        subprocess.run([
            "ros2", "run", "nav2_map_server", "map_saver_cli",
            "-f", map_path
        ], check=True)
        print("[slam] Map saved.")
    except Exception as e:
        print(f"[slam] Map save failed: {e}")

def load_map(basename=MAP_BASENAME):
    #launch nav2 bringup with a saved map. Returns the subprocess or None if not found.
    map_path = os.path.join(MAP_DIR, basename + ".yaml")
    if not os.path.exists(map_path):
        print(f"[slam] ERROR: Map {map_path} not found.")
        return None
    print(f"[slam] Loading map {map_path} ...")
    env = os.environ.copy()
    env["PATH"] = "/opt/ros/humble/bin:" + env.get("PATH", "")
    return subprocess.Popen([
        "ros2", "launch", "nav2_bringup", "bringup_launch.py",
        f"map:={map_path}"
    ], env=env)

def _get_current_pose_from_lidar():
    #read current pose from perception.lidar.get_pose() and return (x, y, theta).
    #returns None if not available.
    global _lidar_thread_started

    with _lidar_thread_lock:
        if not _lidar_thread_started:
            t = threading.Thread(target=lidar_thread, daemon=True)
            t.start()
            _lidar_thread_started = True
            time.sleep(1.0)

    try:
        p = get_pose()
        if p is None:
            return None
        if isinstance(p, (list, tuple)) and len(p) >= 3:
            return float(p[0]), float(p[1]), float(p[2])
        if isinstance(p, dict):
            return float(p.get("x", 0.0)), float(p.get("y", 0.0)), float(p.get("theta", 0.0))
        if hasattr(p, "x") and hasattr(p, "y") and hasattr(p, "theta"):
            return float(p.x), float(p.y), float(p.theta)
    except Exception as e:
        print(f"[slam] Warning: error reading pose from perception.lidar: {e}")
    return None

def teach_place(name: str, aliases=None, timeout=3.0, interval=0.2):
    if aliases is None:
        aliases = []

    pose_tuple = None
    elapsed = 0.0
    while elapsed < timeout:
        pose_tuple = _get_current_pose_from_lidar()
        if pose_tuple and any(abs(v) > 1e-5 for v in pose_tuple):
            break
        time.sleep(interval)
        elapsed += interval

    if pose_tuple is None or all(abs(v) < 1e-5 for v in pose_tuple):
        print("[slam] Warning: timeout waiting for live pose. Using zeros.")
        pose_tuple = (0.0, 0.0, 0.0)

    x, y, theta = pose_tuple

    try:
        from navigation.places import PlaceManager, Pose
    except Exception as e:
        print(f"[slam] ERROR: Could not import navigation.places: {e}")
        return

    pm = PlaceManager()
    pm.add_place(name, Pose(x, y, theta), aliases=aliases)
    print(f"[slam] Learned place '{name}' at x={x:.2f}, y={y:.2f}, θ={theta:.2f}")



if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--start", action="store_true", help="Start SLAM process")
    ap.add_argument("--save", action="store_true", help="Save map and exit")
    ap.add_argument("--teach", type=str, help="Teach a named place from current SLAM pose")
    ap.add_argument("--aliases", type=str, default="", help="Comma-separated aliases for teach")
    args = ap.parse_args()

    if args.start:
        proc = start_slam()
        try:
            print("[slam] Running... Drive robot around with teleop or autonomy.")
            while True:
                time.sleep(2)
        except KeyboardInterrupt:
            print("\n[slam] Ctrl+C → stopping and saving map.")
            save_map()
        finally:
            try:
                proc.terminate()
                proc.wait()
            except Exception:
                pass
        raise SystemExit(0)

    if args.save:
        save_map()
        raise SystemExit(0)

    if args.teach:
        aliases = [a.strip() for a in args.aliases.split(",")] if args.aliases else []
        teach_place(args.teach, aliases)
        raise SystemExit(0)

    ap.print_help()
