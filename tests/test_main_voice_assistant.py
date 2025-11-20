#!/usr/bin/env python3

"""
One-shot voice test:
- Listens to the mic once
- Parse command
- If user asked "Where am I?" → calls where_am_i service
"""

import time
import threading
import rclpy
from std_srvs.srv import Trigger

from voice import main_voice_assistant
from voice import where_am_i_tool
from voice.llm_tools import parse_command


def start_where_am_i_service():
    """Start the ROS2 service in a background thread."""
    rclpy.init()
    node = where_am_i_tool.WhereAmINode()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    time.sleep(0.4)
    return node


def main():
    print("\n=== 🎤 One-Shot Voice Command Test ===")
    print("Speak after the beep...")
    time.sleep(1)
    print("Beep!\n")

    # ------------------------------------------------------
    # 1. Listen ONCE
    # ------------------------------------------------------
    spoken_text = main_voice_assistant.listen()
    if not spoken_text:
        print("❌ No speech detected.")
        return

    print(f"🗣 You said: {spoken_text!r}")

    # ------------------------------------------------------
    # 2. Parse intent
    # ------------------------------------------------------
    known_rooms = ["Living Room", "Kitchen", "Bedroom"]
    intent = parse_command(spoken_text, known_rooms)

    print(f"🔍 Parsed Intent: {intent}")

    # ------------------------------------------------------
    # 3. Start where_am_i node
    # ------------------------------------------------------
    where_node = start_where_am_i_service()

    # ------------------------------------------------------
    # 4. If status/where → call the service
    # ------------------------------------------------------
    if intent.get("action") == "status":
        print("📡 Calling /where_am_i ...")
        response = main_voice_assistant.get_location_status()
        print("📍 Location:", response.get("message"))

    else:
        print("ℹ️ No status action detected. Nothing to test.")

    # ------------------------------------------------------
    # 5. Shutdown
    # ------------------------------------------------------
    where_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

    print("\n=== ✔ One-Shot Voice Test Complete ===\n")


if __name__ == "__main__":
    main()

