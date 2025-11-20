#!/usr/bin/env python3

import time
import threading
import rclpy
from std_srvs.srv import Trigger

# Import your module
from voice import where_am_i_tool


def main():
    print("\n=== TEST: where_am_i_tool ===")

    # -----------------------------------------------
    # 1. Mock pose so we know expected output
    # -----------------------------------------------
    def fake_pose():
        return (1.0, 2.0, 0.0)   # Should be Living Room
    where_am_i_tool.get_pose_with_fallback = fake_pose

    # -----------------------------------------------
    # 2. Start service node
    # -----------------------------------------------
    rclpy.init()
    node = where_am_i_tool.WhereAmINode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    time.sleep(0.4)

    # -----------------------------------------------
    # 3. Call the service manually
    # -----------------------------------------------
    client = node.create_client(Trigger, "where_am_i")
    if not client.wait_for_service(timeout_sec=3.0):
        print("❌ Service not available")
        return

    req = Trigger.Request()
    future = client.call_async(req)

    # wait for result
    while rclpy.ok() and not future.done():
        time.sleep(0.05)

    result = future.result()
    print(f"Service Response → {result.message}")

    # -----------------------------------------------
    # 4. Shutdown
    # -----------------------------------------------
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print("=== where_am_i_tool test complete ===\n")


if __name__ == "__main__":
    main()

