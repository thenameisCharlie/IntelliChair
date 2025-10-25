import time
import os
from voice.command_listener import CommandListener
from voice.tts import speak
from navigation.navigator import Navigator

# Default model path; user can override via ENV VOSK_MODEL_PATH
MODEL_PATH = os.environ.get("VOSK_MODEL_PATH", "/home/robotpi/models/vosk-en")

def run():
    nav = Navigator()
    listener = CommandListener(MODEL_PATH)
    listener.start()
    speak("Voice control ready.")

    last_cmd = None
    last_ts = 0.0

    try:
        for cmd in listener.listen():
            now = time.time()
            if cmd == last_cmd and (now - last_ts) < 1.0:
                continue
            last_cmd, last_ts = cmd, now

            if cmd == "stop":
                nav.emergency_stop()
                speak("Stopped.")
                continue

            if cmd == "go to kitchen":
                speak("Going to the kitchen.")
                try:
                    nav.go_to("kitchen")
                except Exception as e:
                    speak(f"Navigation failed. {e}")
                continue

            if cmd == "status":
                room = nav.current_room()
                # Placeholder battery check; integrate with your BMS if available
                speak(f"I am in the {room}. Battery OK.")
                print(f"[status] room={room} battery=OK")
                continue
    finally:
        listener.stop()
        speak("Voice control shutting down.")

if __name__ == "__main__":
    run()
