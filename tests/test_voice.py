#!/usr/bin/env python3
"""
test_voice.py – Integrated IntelliChair voice test
Runs LiDAR (auto-restart), SLAM, and the where_am_i service together with
a simple voice/keyboard assistant for testing.
"""

import os
import sys
import time
import math
import json
import threading
from pathlib import Path
import subprocess
import numpy as np
from rplidar import RPLidar, RPLidarException

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import pyttsx3

# ----------------------------------------------------------
# PATHS / CONSTANTS
# ----------------------------------------------------------
POSE_PATH = Path("/tmp/ic_pose.json")
PORT = "/dev/ttyUSB0"
BAUDRATE = 115200

_pose_lock = threading.Lock()
_current_pose = [0.0, 0.0, 0.0]

# ----------------------------------------------------------
# STARTUP DIAGNOSTICS
# ----------------------------------------------------------
print("🔍 Starting IntelliChair voice integration test...")
print(f"Working directory: {os.getcwd()}")
print(f"Python executable: {sys.executable}")

try:
    import rclpy
    print("✅ rclpy imported successfully.")
except Exception as e:
    print(f"❌ Failed to import rclpy: {e}")
    sys.exit(1)


# ----------------------------------------------------------
# SPEECH OUTPUT
# ----------------------------------------------------------
def speak(text: str):
    """Speak text aloud using pyttsx3."""
    try:
        engine = pyttsx3.init()
        engine.say(text)
        engine.runAndWait()
    except Exception as e:
        print(f"[tts] Warning: speech output failed ({e})")


# ----------------------------------------------------------
# POSE UTILITIES
# ----------------------------------------------------------
def update_pose(x, y, theta):
    """Update the in-memory and on-disk pose."""
    with _pose_lock:
        _current_pose[:] = [float(x), float(y), float(theta)]
    try:
        tmp = POSE_PATH.with_suffix(".tmp")
        tmp.write_text(json.dumps({"x": x, "y": y, "theta": theta}))
        tmp.replace(POSE_PATH)
    except Exception:
        pass


def get_pose():
    """Return (x, y, theta) from memory or JSON file."""
    try:
        if POSE_PATH.exists():
            d = json.loads(POSE_PATH.read_text())
            return (d.get("x", 0.0), d.get("y", 0.0), d.get("theta", 0.0))
    except Exception:
        pass
    with _pose_lock:
        return tuple(_current_pose)


# ----------------------------------------------------------
# LIDAR THREAD (auto-restart)
# ----------------------------------------------------------
def lidar_thread(port=PORT, baudrate=BAUDRATE):
    """Continuously read LiDAR scans and update pose."""
    while True:
        try:
            lidar = RPLidar(port, baudrate)
            print("[lidar] ✅ Connected and scanning...")
            total_rotation = 0.0
            for scan in lidar.iter_scans():
                if not scan:
                    continue
                distances = [d for q, a, d in scan if d > 0]
                avg = sum(distances) / len(distances) if distances else 0
                total_rotation += 0.02
                x = avg * math.cos(total_rotation) / 2000.0
                y = avg * math.sin(total_rotation) / 2000.0
                update_pose(x, y, total_rotation)
                time.sleep(0.05)
        except RPLidarException as e:
            print(f"[lidar] ⚠️ {e}, restarting in 3 s...")
        except Exception as e:
            print(f"[lidar] ❌ Unexpected error: {e}, restarting in 3 s...")
        finally:
            try:
                lidar.stop()
                lidar.stop_motor()
                lidar.disconnect()
            except Exception:
                pass
            time.sleep(3)
            print("[lidar] 🔄 Reconnecting...")


# ----------------------------------------------------------
# SLAM LAUNCHER
# ----------------------------------------------------------
def start_slam():
    """Launch slam_toolbox online_async."""
    try:
        subprocess.Popen(
            ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp
        )
        print("[slam] ✅ slam_toolbox launched.")
    except Exception as e:
        print(f"[slam] ⚠️ Could not launch SLAM: {e}")


# ----------------------------------------------------------
# WHERE-AM-I SERVICE NODE
# ----------------------------------------------------------
class WhereAmINode(Node):
    def __init__(self):
        super().__init__("Where_am_i_node")
        self.srv = self.create_service(Trigger, "where_am_i", self.handle_request)
        print("[where_am_i] ✅ Service '/where_am_i' ready.")

    def handle_request(self, request, response):
        x, y, th = get_pose()
        response.success = True
        response.message = f"You are in the Living Room at ({x:.2f}, {y:.2f}), facing {math.degrees(th)%360:.1f}°."
        print(f"[where_am_i] → {response.message}")
        return response


# ----------------------------------------------------------
# SIMPLE COMMAND PARSER
# ----------------------------------------------------------
def parse_command(text):
    """Minimal intent recognizer."""
    t = text.lower()
    if "where" in t and "am" in t:
        return {"action": "where"}
    elif "status" in t:
        return {"action": "status"}
    elif "stop" in t:
        return {"action": "stop"}
    return {"action": "unknown"}


# ----------------------------------------------------------
# MAIN LOOP
# ----------------------------------------------------------
def main():
    # --- Launch LiDAR & SLAM threads ---
    threading.Thread(target=lidar_thread, daemon=True).start()
    threading.Thread(target=start_slam, daemon=True).start()

    # --- Init ROS2 and node ---
    rclpy.init()
    node = WhereAmINode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # --- Diagnostics ---
    print("\n🧠 System checks:")
    time.sleep(2)
    client = node.create_client(Trigger, "where_am_i")
    if client.wait_for_service(timeout_sec=5.0):
        print("✅ /where_am_i service available.")
    else:
        print("❌ /where_am_i service not available (check LiDAR/ROS2).")

    print("🎙️ Voice Assistant ready. Type commands like 'where am I', 'status', or 'stop'.\n")

    try:
        while True:
            text = input("You: ").strip()
            if text.lower() in {"exit", "quit"}:
                break
            intent = parse_command(text)

            if intent["action"] == "where":
                if not client.wait_for_service(timeout_sec=3.0):
                    print("⚠️ /where_am_i not available.")
                    continue
                future = client.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(node, future)
                if future.result():
                    msg = future.result().message
                    print("Robot:", msg)
                    speak(msg)
                else:
                    print("Robot: Could not get location.")
                    speak("I could not get location.")
            elif intent["action"] == "stop":
                print("Robot: Stopping for safety.")
                speak("Stopping for safety.")
            elif intent["action"] == "status":
                x, y, th = get_pose()
                msg = f"System active. Current pose ({x:.2f}, {y:.2f}), heading {math.degrees(th)%360:.1f}°."
                print("Robot:", msg)
                speak(msg)
            else:
                print("Robot: I didn’t understand that.")
                speak("I didn’t understand that.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("[main] Shutdown complete.")


# ----------------------------------------------------------
# ENTRY POINT
# ----------------------------------------------------------
if __name__ == "__main__":
    main()





