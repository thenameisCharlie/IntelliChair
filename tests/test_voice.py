# #!/usr/bin/env python3

Full IntelliChair integration test.
Runs LiDAR, SLAM, the where_am_i service, and the main voice assistant.
"""

import os
import sys
import threading
import time
import rclpy
from std_srvs.srv import Trigger

# --- Import your real project modules ---
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from perception import lidar, slam
from voice import where_am_i_tool, main_voice_assistant

print("🔍 Starting IntelliChair FULL integration test (voice enabled)...")

# -------------------------------------------------------
# STEP 1: START LIDAR & SLAM
# -------------------------------------------------------
lidar_thread = threading.Thread(target=lidar.lidar_thread, daemon=True)
lidar_thread.start()
print("✅ LiDAR thread started.")

slam_proc = slam.start_slam()
print("✅ SLAM process launched.")

# -------------------------------------------------------
# STEP 2: START THE WHERE-AM-I ROS2 NODE
# -------------------------------------------------------
rclpy.init()
where_node = where_am_i_tool.WhereAmINode()
threading.Thread(target=rclpy.spin, args=(where_node,), daemon=True).start()
print("✅ where_am_i ROS2 service node running.")

# -------------------------------------------------------
# STEP 3: TEST THE SERVICE ONCE
# -------------------------------------------------------
time.sleep(3)
client = where_node.create_client(Trigger, "where_am_i")
if client.wait_for_service(timeout_sec=5.0):
    req = Trigger.Request()
    future = client.call_async(req)
    while rclpy.ok() and not future.done():
        time.sleep(0.05)
    if future.result():
        print(f"[auto-test] → {future.result().message}")
    else:
        print("[auto-test] ❌ No response from /where_am_i")
else:
    print("⚠️ /where_am_i not available.")

# -------------------------------------------------------
# STEP 4: START MAIN VOICE ASSISTANT (with delay)
# -------------------------------------------------------
print("\n⏳ Waiting 5 seconds for LiDAR & SLAM stabilization...")
time.sleep(5)
print("🎙️ Launching voice assistant...")
try:
    main_voice_assistant.main()
except KeyboardInterrupt:
    print("\n🛑 Shutting down IntelliChair...")

# -------------------------------------------------------
# STEP 5: CLEAN SHUTDOWN
# -------------------------------------------------------
print("\n🧹 Cleaning up ROS2 nodes and SLAM...")
where_node.destroy_node()
if rclpy.ok():
    rclpy.shutdown()

print("[main] ✅ IntelliChair system shutdown complete.")









