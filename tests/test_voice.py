import rclpy
from std_srvs.srv import Trigger
from voice.llm_tools import parse_command
import pyttsx3

def test_tts_output():
    """Verify that pyttsx3 TTS engine initializes and speaks."""
    try:
        engine = pyttsx3.init()
        engine.say("Testing voice output.")
        engine.runAndWait()
        print("[voice] ✅ TTS engine initialized successfully.")
    except Exception as e:
        print(f"[voice] ❌ TTS initialization failed: {e}")

def test_llm_parser():
    """Test a few typical voice commands for expected LLM actions."""
    known_rooms = ["Living Room", "Kitchen", "Bedroom"]

    test_inputs = {
        "where am i": "status",
        "status": "status",
        "stop": "stop",
        "go to the kitchen": "go",
        "bring me to the bedroom": "go",
        "what room am i in": "status",
    }

    print("[voice] 🧠 Testing LLM parser...")
    for text, expected_action in test_inputs.items():
        intent = parse_command(text, known_rooms)
        action = intent.get("action")
        assert action == expected_action or action == "ask", \
            f"Expected '{expected_action}', got '{action}' for input '{text}'"
        print(f"   '{text}' → {action}")
    print("[voice] ✅ LLM parser responses OK.")

def test_where_am_i_service():
    """Test communication with the ROS2 where_am_i node."""
    print("[voice] 🔄 Testing ROS2 /where_am_i service...")
    rclpy.init()
    node = rclpy.create_node("test_voice_client")
    client = node.create_client(Trigger, "where_am_i")

    if not client.wait_for_service(timeout_sec=3.0):
        print("[voice] ⚠️ Service '/where_am_i' not available.")
        rclpy.shutdown()
        return

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    if result:
        print(f"[voice] ✅ Service response: {result.message}")
    else:
        print("[voice] ❌ No response from /where_am_i service.")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    print("\n=== 🧩 IntelliChair Voice System Test ===\n")
    test_tts_output()
    test_llm_parser()
    test_where_am_i_service()
    print("\n✅ Voice subsystem tests complete.\n")
