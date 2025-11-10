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
    awaiting_until = 0.0             # NEW: suppress keywords while we wait for an answer
    CLARIFY_COOLDOWN = 4.0           # NEW: ignore keyword hits for this long

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
        
    # Makes the prompts visually appealing
    def _say(msg: str):
        # Mute mic while speaking to avoid hearing our own TTS
        listener.pause()                 # NEW
        speak(msg)
        print(f"🗣  {msg}")
        time.sleep(0.20)                 # tiny settle
        listener.resume()                # NEW

    def _choices_to_english(choices):
        if not choices: 
            return ""
        return choices[0] if len(choices)==1 else ", ".join(choices[:-1]) + f", or {choices[-1]}"

    def _log_ai(utter, intent):
        lines = [
            "🤖 AI decision:",
            f"  heard   : {utter!r}",
            f"  action  : {intent.get('action')}",
            f"  target  : {intent.get('target')}",
            f"  reason  : {intent.get('reason','')}",
        ]
        if intent.get("question"):
            lines.append(f"  question: {intent['question']}")
        if intent.get("choices"):
            lines.append("  choices : " + ", ".join(intent["choices"]))
        print("\n".join(lines))
        
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

                # If we're in a clarification window, ignore keyword emissions
                if now < awaiting_until:
                    continue

                last_cmd_time = now

                print(f"[voice] recognized: {cmd}")

                # EMERGENCY STOP (always available)
                if cmd == "stop":
                    try:
                        nav.emergency_stop()
                    except Exception:
                        pass
                    _say("Stopping now.")
                    continue

                # STATUS
                if cmd == "status":
                    room = nav.current_room()
                    _say(f"I'm in the {room}. Battery OK.")
                    continue

                # GO TO KITCHEN (accept synonym 'kitchen')
                if cmd in ("go to kitchen", "kitchen"):
                    if is_busy():
                        _say("Already navigating. Say 'stop' to halt.")
                        continue
                    _say("Heading to the kitchen.")
                    start_nav("kitchen")
                    continue

                
                # Unknown with fixed grammar → switch to free-form AI mode
                # print(f"[voice] Unhandled fixed command: {cmd}")

                
                # -------- Unknown / AI-handled path --------
                utter = (cmd or "").strip()

                # Case 1: listener told us there was speech but FINAL was empty,
                # or user said a dangling "go to" → prompt for room and capture free-form.
                # if cmd == "__FREEFORM__" or cmd == "go to":
                #     _say("Which room would you like to go to?")
                #     awaiting_until = time.time() + CLARIFY_COOLDOWN
                #     time.sleep(0.80)
                #     utter = listener.listen_freeform_once(seconds=8.0)
                #     awaiting_until = 0.0
                #     if not utter:
                #         _say("Sorry, I didn't catch that.")
                #         continue

                if cmd == "__FREEFORM__" or cmd == "go to":
                    _say("Which room would you like to go to?")
                    awaiting_until = time.time() + CLARIFY_COOLDOWN

                    # ↓ NEW: clear any TTS echo / old audio and give a short arm window
                    listener.flush_queue()
                    time.sleep(0.25)

                    utter = listener.listen_freeform_once(seconds=8.0)
                    awaiting_until = 0.0
                    if not utter:
                        _say("Sorry, I didn't catch that.")
                        continue

                # Case 2: otherwise, treat whatever we heard as the utterance and let AI parse it
                rooms = _known_rooms()
                if not rooms:
                    _say("I don't have any saved rooms yet. Please save places first.")
                    continue

                intent = parse_command(utter, rooms)
                _log_ai(utter, intent)

                if intent["action"] == "stop":
                    try: nav.emergency_stop()
                    except Exception: pass
                    _say("Stopping now.")
                    continue

                if intent["action"] == "status":
                    room = nav.current_room()
                    _say(f"I'm in the {room}.")
                    continue

                if intent["action"] == "ask":
                    choices = intent.get("choices") or rooms
                    prompt  = intent.get("question") or "Which room would you like?"
                    _say(f"{prompt} {_choices_to_english(choices)}.")
                    awaiting_until = time.time() + CLARIFY_COOLDOWN
                    time.sleep(0.80)
                    choice = _disambiguate(choices)
                    awaiting_until = 0.0
                    if not choice:
                        _say("Sorry, I didn't catch that.")
                        continue
                    if is_busy():
                        _say("Already navigating. Say 'stop' to halt.")
                        continue
                    _say(f"Okay—going to the {choice}.")
                    start_nav(choice)
                    continue

                if intent["action"] == "go":
                    target = intent.get("target")
                    if not target:
                        _say("Sorry, I didn't understand.")
                        continue
                    if is_busy():
                        _say("Already navigating. Say 'stop' to halt.")
                        continue
                    _say(f"On my way to the {target}. If you need to stop, say 'stop'.")
                    start_nav(target)
                    continue

                # Fallback
                _say("Sorry, I didn't understand.")
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
