import os
import threading
import time

from voice.command_listener import CommandListener
from voice.tts import speak
from navigation.navigator import Navigator

MODEL_PATH = os.environ.get("VOSK_MODEL_PATH", "/home/robotpi/models/vosk-en")

def run():
    nav = Navigator()

    listener = CommandListener(MODEL_PATH)
    listener.start()
    speak("Voice control ready.")
    print("[voice] Ready.")

    # Background nav thread handle + lock
    nav_thread = None
    nav_lock = threading.Lock()
    navigating = False
    last_cmd_time = 0.0
    DEBOUNCE_SEC = 0.5

    def is_busy():
        nonlocal nav_thread
        return nav_thread is not None and nav_thread.is_alive()

    def start_nav(destination: str):
        """Start navigation in a background thread."""
        nonlocal nav_thread, navigating
        def _worker():
            try:
                nav.go_to(destination)
            finally:
                # mark idle when finished or errored
                with nav_lock:
                    navigating = False
        with nav_lock:
            navigating = True
        nav_thread = threading.Thread(target=_worker, daemon=True)
        nav_thread.start()

    try:
        while True:
            for cmd in listener.listen():
                now = time.time()
                if now - last_cmd_time < DEBOUNCE_SEC:
                    continue
                last_cmd_time = now

                print(f"[voice] recognized: {cmd}")

                # EMERGENCY STOP (always available)
                if cmd == "stop":
                    try:
                        nav.emergency_stop()
                    except Exception:
                        pass
                    speak("Stopped.")
                    # if a nav thread is running, let it unwind
                    continue

                # STATUS
                if cmd == "status":
                    room = nav.current_room()
                    speak(f"I am in the {room}. Battery OK.")
                    continue

                # GO TO KITCHEN (accept synonym 'kitchen')
                if cmd in ("go to kitchen", "kitchen"):
                    if is_busy():
                        speak("Already navigating. Say stop to halt.")
                        continue
                    speak("Going to the kitchen.")
                    start_nav("kitchen")
                    continue

                # Unknown (shouldn't happen with fixed grammar, but safe)
                print(f"[voice] Unhandled command: {cmd}")

    except KeyboardInterrupt:
        print("[voice] Ctrl+C received. Shutting down.")
    finally:
        try:
            listener.stop()
        except Exception:
            pass
        try:
            import RPi.GPIO as GPIO
            GPIO.cleanup()
        except Exception:
            pass
        speak("Voice control shutting down.")

if __name__ == "__main__":
    run()
