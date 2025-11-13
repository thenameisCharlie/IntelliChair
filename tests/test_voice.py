import rclpy
from std_srvs.srv import Trigger
from voice.llm_tools import parse_command
import pyttsx3
import speech_recognition as sr
import time



# 1. Text-to-Speech (TTS) Test

def test_tts_output():
    """Verify that pyttsx3 TTS engine initializes and speaks."""
    try:
        engine = pyttsx3.init()
        engine.say("Testing voice output from IntelliChair system.")
        engine.runAndWait()
        print("[voice] ✅ TTS engine initialized successfully.")
    except Exception as e:
        print(f"[voice] ❌ TTS initialization failed: {e}")



# 2. Command Parsing (LLM Tools) Test

def test_llm_parser():
    """Check if LLM parser correctly interprets text commands."""
    known_rooms = ["Living Room", "Kitchen", "Bedroom"]

    test_inputs = {
        "where am i": "status",
        "status": "status",
        "stop": "stop",
        "go to the kitchen": "go",
        "take me to the bedroom": "go",
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



# 3. ROS2 where_am_i Service Test

def test_where_am_i_service():
    """Query the where_am_i ROS2 service for location."""
    print("[voice] 🔄 Checking ROS2 /where_am_i service...")
    rclpy.init()
    node = rclpy.create_node("test_voice_client")
    client = node.create_client(Trigger, "where_am_i")

    if not client.wait_for_service(timeout_sec=3.0):
        print("[voice] ⚠️ Service '/where_am_i' not available.")
        rclpy.shutdown()
        return None

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()

    node.destroy_node()
    rclpy.shutdown()

    if result:
        print(f"[voice] ✅ Service response: {result.message}")
        return result.message
    else:
        print("[voice] ❌ No response from /where_am_i service.")
        return None



# 4 Live Microphone Command Test

def test_live_voice():
    """Test real microphone input for speech recognition and command handling."""
    print("\n🎤 Speak a command when prompted (say 'exit' to quit).")
    print("   Try: 'where am I', 'status', 'go to kitchen', or 'stop'.\n")

    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    tts_engine = pyttsx3.init()
    known_rooms = ["Living Room", "Kitchen", "Bedroom", "Bathroom"]

    while True:
        try:
            with mic as source:
                recognizer.adjust_for_ambient_noise(source)
                print("Mic: Listening...")
                audio = recognizer.listen(source)

            command = recognizer.recognize_google(audio)
            print(f"You said: {command}")

            if command.lower() in {"exit", "quit"}:
                print("[voice] 👋 Exiting live test.")
                tts_engine.say("Goodbye.")
                tts_engine.runAndWait()
                break

            # Parse and respond
            intent = parse_command(command, known_rooms)
            action = intent.get("action")
            target = intent.get("target")
            print(f"[intent] {intent}")

            if action == "status":
                msg = test_where_am_i_service() or "Could not get location."
                print(f"Robot: {msg}")
                tts_engine.say(msg)

            elif action == "go" and target:
                msg = f"Okay, going to the {target}."
                print(f"Robot: {msg}")
                tts_engine.say(msg)

            elif action == "stop":
                msg = "Stopping immediately."
                print(f"Robot: {msg}")
                tts_engine.say(msg)

            elif action == "ask":
                q = intent.get("question", "Can you clarify?")
                opts = intent.get("choices", [])
                msg = q + (" Options are: " + ", ".join(opts) if opts else "")
                print(f"Robot: {msg}")
                tts_engine.say(q)

            else:
                msg = "I didn't understand that command."
                print(f"Robot: {msg}")
                tts_engine.say(msg)

            tts_engine.runAndWait()

        except sr.UnknownValueError:
            print("[voice] ❌ Could not understand audio.")
            continue
        except Exception as e:
            print(f"[voice] ⚠️ Error: {e}")
            time.sleep(1)


# Main runner
if __name__ == "__main__":
    print("\n=== 🧩 IntelliChair Voice System Live Test ===\n")
    test_tts_output()
    test_llm_parser()
    msg = test_where_am_i_service()
    if msg:
        print(f"Current location: {msg}\n")
    test_live_voice()
    print("\n✅ Voice subsystem live test complete.\n")

