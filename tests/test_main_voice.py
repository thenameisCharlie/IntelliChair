import time
import threading
import rclpy
from std_srvs.srv import Trigger

# Import your modules
from voice import main_voice_assistant
from voice import where_am_i_tool


def test_voice_assistant_calls_where_am_i(monkeypatch):
    """
    Test that the voice assistant successfully calls the where_am_i service
    and returns the correct location message.
    """

    # ------------------------------
    # 1. Mock pose so output is predictable
    # ------------------------------
    monkeypatch.setattr(
        where_am_i_tool,
        "get_pose_with_fallback",
        lambda: (1.0, 2.0, 0.0)   # This is in the Living Room region
    )

    # ------------------------------
    # 2. Start the ROS2 /where_am_i service node
    # ------------------------------
    rclpy.init()
    node = where_am_i_tool.WhereAmINode()

    # spin in background
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # allow server to start
    time.sleep(0.4)

    # ------------------------------
    # 3. Call voice assistant's get_location_status()
    # ------------------------------
    result = main_voice_assistant.get_location_status()
    assert "Living Room" in result["message"]
    assert "(1.00, 2.00)" in result["message"]

    # ------------------------------
    # 4. Clean shutdown
    # ------------------------------
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

