# main_voice_assistant.py
# connects the LLM tool (for understanding user speech)
# with ROS2 where_am_i service (for robot location reporting).

from voice.llm_tools import parse_command
from std_srvs.srv import Trigger
import rclpy
import pyttsx3
import time
import speech_recognition as sr   # mic new import

#ROS2 SERVICE CALLER
def get_location_status():
    """Call your ROS2 /where_am_i service."""
    rclpy.init()
    node = rclpy.create_node("where_am_i_client")
    client = node.create_client(Trigger, "where_am_i")

    if not client.wait_for_service(timeout_sec=3.0):
        node.get_logger().error("Service not available.")
        rclpy.shutdown()
        return {"error": "service not available"}

    req = Trigger.Request()
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    result = future.result()
    node.destroy_node()
    rclpy.shutdown()
    return {"message": result.message}


#SPEECH OUTPUT
def speak(text: str):
    """Speak text aloud."""
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()


#SPEECH INPUT
def listen():
    """Listen to the user's voice and return recognized text."""
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        print("\nMic: Listening... (say 'exit' to quit)")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        text = recognizer.recognize_google(audio)
        print(f"You said: {text}")
        return text
    except sr.UnknownValueError:
        print("Sorry, I didn’t catch that.")
        return ""
    except sr.RequestError:
        print("Speech recognition unavailable.")
        return ""


# --- MAIN LOOP ---
def main():
    known_rooms = ["Living Room", "Kitchen", "Bedroom"]
    print("Voice Assistant Ready. Say a command (e.g., 'Where am I?').")

    while True:
        #Get voice input
        user_text = listen().strip()
        if not user_text:
            continue

        if user_text.lower() in {"exit", "quit", "stop listening"}:
            print("Robot: Exiting assistant.")
            speak("Goodbye!")
            break

        # Interpret the command
        intent = parse_command(user_text, known_rooms)
        action = intent.get("action")
        target = intent.get("target")
        reason = intent.get("reason", "")

        print(f"[intent] action={action}, target={target}, reason={reason}")

        # Connect to your ROS2 node
        if action == "status":
            location = get_location_status()
            message = location.get("message", "Could not get your location.")
            print(f"Robot: {message}")
            speak(message)

        elif action == "go" and target:
            reply = f"Okay, going to the {target}."
            print(f"Robot: {reply}")
            speak(reply)
            # TODO: send navigation command

        elif action == "stop":
            reply = "Stopping immediately for safety."
            print(f"Robot: {reply}")
            speak(reply)
            # TODO: stop motors

        elif action == "ask":
            q = intent.get("question", "Can you clarify?")
            choices = intent.get("choices", [])
            print(f"Robot: {q}")
            if choices:
                print(f"Options: {', '.join(choices)}")
            speak(q)

        else:
            reply = "I didn't understand that command."
            print(f"Robot: {reply}")
            speak(reply)

        time.sleep(0.5)


if __name__ == "__main__":
    main()
