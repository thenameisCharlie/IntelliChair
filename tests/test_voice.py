#!/usr/bin/env python3
"""
tests/test_full_intellichair.py
Full integration test: LiDAR + SLAM + where_am_i service + Voice Assistant (mic).
"""

import os
import sys
import time
import threading
import signal
import subprocess
import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

# Add root to path
repo_root = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from perception.lidar import lidar_thread
from perception.slam import start_slam
from voice.where_am_i_tool import WhereAmINode
from voice.main_voice_assistant import main as voice_main

def main():
    print("🔍 Starting full IntelliChair integration test...")

    # Start LiDAR thread
    lidar_t = threading.Thread(target=lidar_thread, daemon=True)
    lidar_t.start()
    print("✅ LiDAR thread started.")

    # Start SLAM
    slam_proc = start_slam()
    print("✅ SLAM process launched.")

    # Init ROS2 and start where_am_i node
    #rclpy.init()
    where_node = WhereAmINode()
    executor = SingleThreadedExecutor()
    executor.add_node(where_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    print("✅ where_am_i service node running.")

    # Wait for service availability
    time.sleep(3)
    client = where_node.create_client(Trigger, "where_am_i")
    if client.wait_for_service(timeout_sec=5.0):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(where_node, future)
        res = future.result()
        if res:
            print(f"[auto-test] → {res.message}")
        else:
            print("[auto-test] ❌ No response from /where_am_i")
    else:
        print("⚠️ /where_am_i service not available.")

    # Wait to allow LiDAR/SLAM data to stabilize
    print("⏳ Waiting 5 seconds for LiDAR & SLAM to stabilize...")
    time.sleep(5)

    # Launch voice assistant (mic mode)
    print("🎙️ Launching voice assistant (mic mode)...")
    try:
        voice_main()
    except KeyboardInterrupt:
        print("\n🛑 Voice Assistant interrupted.")

    # Clean shutdown
    print("🧹 Shutting down nodes and SLAM...")
    executor.shutdown()
    where_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    try:
        slam_proc.terminate()
    except Exception:
        pass
    print("[main] ✅ Integration test complete.")

if __name__ == "__main__":
    main()


