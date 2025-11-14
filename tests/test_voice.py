"""
Full integration test for IntelliChair:
Runs LiDAR pose updates, SLAM (optional), where_am_i service, and the voice assistant in one terminal.
"""

import threading
import time
import subprocess
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from voice.llm_tools import parse_command
import pyttsx3
from rplidar import RPLidar, RPLidarException
import math, json


# 1. LiDAR Thread: updates pose values continuously

_pose_lock = threading.Lock()
_current_pose = [0.0, 0.0, 0.0]

def update_pose(x, y, theta):
    with _pose_lock:
        _current_pose[:] = [x, y, theta]

def get_pose():
    with _pose_lock:
        return tuple(_current_pose)

def lidar_thread(port="/dev/ttyUSB0", baudrate=115200):
    try:
        lidar = RPLidar(port, baudrate)
        print("[lidar] Connected and scanning...")
        total_rotation = 0.0

        for scan in lidar.iter_scans():
            if not scan:
                continue
            distances = [d for q, a, d in scan if d > 0]
            avg_dist = sum(distances) / len(distances) if distances else 0.0
            total_rotation += 0.02
            x = avg_dist * math.cos(total_rotation) / 2000.0
            y = avg_dist * math.sin(total_rotation) / 2000.0
            theta = total_rotation
            update_pose(x, y, theta)
            time.sleep(0.05)
    except RPLidarException as e:
        print(f"[lidar] Error: {e}")
    finally:
        print("[lidar] Stopped.")


# 2. Optional SLAM launch (non-blocking)

def start_slam():
    try:
        print("[slam] Launching slam_toolbox (async)...")
        subprocess.Popen(
            ["ros2", "launch", "slam_toolbox", "online_async_launch.py"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setpgrp
        )
        time.sleep(3)
        print("[slam] SLAM started.")
    except Exception as e:
        print(f"[slam] Failed to start: {e}")


# 3. ROS2 Service Node (Where Am I)

def coordinates_to_room(x, y):
    if -1 < x < 2 and -1 < y < 3:
        return "Living Room"
    elif 3 < x < 6 and -1 < y < 2:
        return "Kitchen"
    elif -2 < x < 1 and 4 < y < 7:
        return "Bedroom"
    else:
        return "Unknown Area"

class WhereAmINode(Node):
    def __init__(self):
        super().__init__("Where_am_i_node")
        self.create_service(Trigger, "where_am_i", self.handle_request)
        self.get_logger().info("✅ where_am_i service ready.")

    def handle_request(self, request, response):
        x, y, theta = get_pose()
        room = coordinates_to_room(x, y)
        response.success = True
        response.message = f"You are in the {room} at ({x:.2f}, {y:.2f}), facing {math.degrees(theta):.1f}°"
        return response


#  4. Voice Assistant

def speak(text):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()

def get_location_status(node):
    client = node.create_client(Trigger, "where_am_i")
    if not client.wait_for_service(timeout_sec=3.0):
        return {"message": "Service '/where_am_i' not available."}
    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    return {"message": result.message if result else "Failed to get location."}

def voice_assistant():
    known_rooms = ["Living Room", "Kitchen", "Bedroom"]
    rclpy.init()
    node = rclpy.create_node("voice_assistant")
    print("🎙️ Voice Assistant Ready. Type a command (e.g., 'where am I' or 'status').")

    while True:
        text = input("\nYou: ").strip()
        if text.lower() in {"exit", "quit"}:
            print("Exiting assistant.")
            break
        intent = parse_command(text, known_rooms)
        action = intent.get("action")

        if action == "status":
            print("[voice] 🔄 Checking ROS2 /where_am_i service...")
            location = get_location_status(node)
            print(f"Robot: {location['message']}")
            speak(location["message"])

        elif action == "go":
            target = intent.get("target", "unknown")
            reply = f"Okay, going to the {target}."
            print(f"Robot: {reply}")
            speak(reply)

        elif action == "stop":
            print("Robot: Stopping immediately for safety.")
            speak("Stopping immediately for safety.")

        elif action == "ask":
            q = intent.get("question", "Can you clarify?")
            choices = intent.get("choices", [])
            print(f"Robot: {q}")
            if choices:
                print(f"Options: {', '.join(choices)}")
            speak(q)

        else:
            print("Robot: I didn’t understand that.")
            speak("I didn’t understand that.")
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()



#  MAIN

def main():
    # Start LiDAR thread
    threading.Thread(target=lidar_thread, daemon=True).start()

    # Optionally start SLAM
    threading.Thread(target=start_slam, daemon=True).start()

    # Initialize ROS2 once
    rclpy.init()
    node = WhereAmINode()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # Wait briefly for the service
    time.sleep(2)
    print("🎙️ Voice Assistant Ready. Type a command (e.g., 'where am I' or 'status').")

    known_rooms = ["Living Room", "Kitchen", "Bedroom"]

    try:
        while True:
            text = input("\nYou: ").strip()
            if text.lower() in {"exit", "quit"}:
                print("Exiting assistant.")
                break

            intent = parse_command(text, known_rooms)
            action = intent.get("action")

            if action == "status":
                print("[voice] 🔄 Checking ROS2 /where_am_i service...")
                client = node.create_client(Trigger, "where_am_i")
                if not client.wait_for_service(timeout_sec=3.0):
                    print("⚠️ Service '/where_am_i' not available.")
                    continue
                req = Trigger.Request()
                future = client.call_async(req)
                rclpy.spin_until_future_complete(node, future)
                if future.result():
                    print(f"Robot: {future.result().message}")
                else:
                    print("Robot: Could not get location.")

            elif action == "stop":
                print("Robot: Stopping immediately for safety.")
                speak("Stopping immediately for safety.")

            elif action == "ask":
                q = intent.get("question", "Can you clarify?")
                choices = intent.get("choices", [])
                print(f"Robot: {q}")
                if choices:
                    print(f"Options: {', '.join(choices)}")
                speak(q)

            elif action == "go":
                target = intent.get("target", "unknown")
                reply = f"Okay, going to the {target}."
                print(f"Robot: {reply}")
                speak(reply)

            else:
                print("Robot: I didn’t understand that.")
                speak("I didn’t understand that.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("[main] Shutdown complete.")



