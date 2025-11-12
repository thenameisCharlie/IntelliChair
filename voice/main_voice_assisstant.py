# main_voice_assistant.py
# connects the LLM tool (for understanding user speech)
# with ROS2 where_am_i service (for robot location reporting).

from voice.llm_tools import parse_command   #  llm_tools file
from std_srvs.srv import Trigger
import rclpy
import time
import pyttsx3  # optional for voice output

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


#SPEECH FUNCTION optional
def speak(text: str):
    engine = pyttsx3.init()
    engine.say(text)
    engine.runAndWait()


#main loop
def main():
    known_rooms = ["Living Room", "Kitchen", "Bedroom"]

    print("Voice Assistant Ready. Type a command or say 'exit' to quit.")
    while True:
        user_text = input("\nYou: ").strip()
        if user_text.lower() in {"exit", "quit"}:
            print("Exiting assistant.")
            break

        #Use LLM tools to interpret the user’s message
        intent = parse_command(user_text, known_rooms)
        action = intent.get("action")
        target = intent.get("target")
        reason = intent.get("reason", "")

        print(f"[intent] action={action}, target={target}, reason={reason}")

        #Connect to where_am_i ROS2 node
        if action == "status":
            location = get_location_status()
            message = location.get("message", "Could not get your location.")
            print(f"Robot: {message}")
            speak(message)

        elif action == "go" and target:
            reply = f"Okay, going to the {target}."
            print(f"Robot: {reply}")
            speak(reply)
            # TODO: send navigation command here

        elif action == "stop":
            reply = "Stopping immediately for safety."
            print(f"Robot: {reply}")
            speak(reply)
            # TODO: send stop command here

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
