import os
import threading
import time

from voice.command_listener import CommandListener
from voice.tts import speak
from navigation.navigator import Navigator
import json
from voice.llm_tools import parse_command

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

    def _known_rooms():
        try:
            with open("navigation/places.json", "r") as f:
                data = json.load(f)
            return sorted(list(data.keys()))
        except Exception:
            return []
        
    # def _known_rooms():
    #     try:
    #         import json, os
    #         from pathlib import Path
    #         base = Path(__file__).resolve().parents[1]  # repo root (…/IntelliChair)
    #         places_path = base / "navigation" / "places.json"
    #         with open(places_path, "r") as f:
    #             data = json.load(f)
    #         return sorted(list(data.keys()))
    #     except Exception:
    #         return []

    def _disambiguate(choices):
        # speak choices and capture a short reply, pick a contained room name
        from voice.tts import speak as _speak  # reuse your TTS
        _speak("Which room? " + ", ".join(choices))
        reply = listener.listen_freeform_once(seconds=3.5)
        if not reply:
            return None
        reply = reply.lower()
        for c in choices:
            if c in reply:
                return c
        # light fallback: first word match
        for c in choices:
            if c.split()[0] in reply:
                return c
        return None

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

                # # Unknown (shouldn't happen with fixed grammar, but safe)
                # print(f"[voice] Unhandled command: {cmd}")
                
                # Unknown with fixed grammar → switch to free-form AI mode
                print(f"[voice] Unhandled fixed command: {cmd}")
                from voice.tts import speak as _speak
                # Use the transcript we already have; if it's too short, ask once.
                # Use what we got, unless it was an unknown token or too short
                utter = None if cmd in ("[unk]", "__FREEFORM__", "<unk>", "unk") else cmd

                if not utter or len(utter.split()) < 2:
                    _speak("Tell me what you need.")
                    utter = listener.listen_freeform_once(seconds=4.0)
                    if not utter:
                        _speak("I did not catch that.")
                        continue

                rooms = _known_rooms()

                if not rooms:
                    _speak("I don't have any saved rooms yet. Please save places first.")
                    continue

                intent = parse_command(utter, rooms)
                print(f"[ai] utter='{utter}' -> intent={intent}")

                if intent["action"] == "stop":
                    try:
                        nav.emergency_stop()
                    except Exception:
                        pass
                    _speak("Stopped.")
                    continue

                if intent["action"] == "status":
                    room = nav.current_room()
                    _speak(f"I am in the {room}.")
                    continue

                if intent["action"] == "ask":
                    choice = _disambiguate(intent.get("choices") or rooms)
                    if not choice:
                        _speak("Sorry, I still did not understand.")
                        continue
                    if is_busy():
                        _speak("Already navigating. Say stop to halt.")
                        continue
                    _speak(f"Heading to the {choice}.")
                    start_nav(choice)
                    continue

                if intent["action"] == "go":
                    target = intent.get("target")
                    if not target:
                        target = _disambiguate(rooms)
                        if not target:
                            _speak("Sorry, I did not understand.")
                            continue
                    if is_busy():
                        _speak("Already navigating. Say stop to halt.")
                        continue
                    _speak(f"On my way to the {target}.")
                    start_nav(target)
                    continue

                _speak("Sorry, I didn't understand.")
                continue


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
