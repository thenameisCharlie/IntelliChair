"""
Voice demo entrypoint:
- "go to kitchen" → time-based drive towards waypoint (placeholder)
- "stop"          → immediate stop
- "status"        → speaks location or battery (placeholder)
Run:
    python -m voice.voice_demo
"""
import threading
import time
from typing import Dict

from hardware.motors import YahboomMotors
from hardware.ultrasonic import distance_filtered_cm
from utils.config import TUNABLES
from voice.asr_keywords import KeywordListener
from voice.text_to_speech import Speaker

# Simple location + waypoints (replace with SLAM/odometry when ready)
WAYPOINTS: Dict[str, float] = {
    # naive "seconds to drive" stand-ins for each room
    "kitchen": 5.0,
    "living room": 0.0,  # assume starting here
}

THRESH_CM = TUNABLES["THRESH_CM"]
CRUISE_SPEED = TUNABLES["CRUISE_SPEED"]
SLOW_SPEED = TUNABLES["SLOW_SPEED"]
SLOW_BAND = TUNABLES["SLOW_BAND"]

class VoiceController:
    def __init__(self, vosk_model_path: str):
        self.speaker = Speaker()
        self.listener = KeywordListener(vosk_model_path)
        self.listener.on_go_to = self._cmd_go_to
        self.listener.on_stop = self._cmd_stop
        self.listener.on_status = self._cmd_status

        self.motors = YahboomMotors()
        self._nav_lock = threading.Lock()
        self._nav_thread: threading.Thread | None = None
        self._stop_nav = threading.Event()
        self._current_room = "living room"

    # ---- Commands ----
    def _cmd_go_to(self, room: str):
        # map aliases
        aliases = {
            "kitchen": "kitchen",
            "the kitchen": "kitchen",
            "living room": "living room",
            "the living room": "living room",
            "livingroom": "living room",
        }
        target = aliases.get(room, room)
        if target not in WAYPOINTS:
            self.speaker.say(f"I don't know a place called {room}.")
            return
        self.speaker.say(f"Going to {target}.")
        self._start_navigation(target)

    def _cmd_stop(self):
        self.speaker.say("Stopping.")
        self._stop_nav.set()
        try:
            self.motors.stop()
        except Exception:
            pass

    def _cmd_status(self):
        # Placeholder: battery always OK; room = current
        self.speaker.say(f"I am in the {self._current_room}. Battery OK.")

    # ---- Navigation ----
    def _start_navigation(self, target_room: str):
        with self._nav_lock:
            # stop any existing nav
            self._stop_nav.set()
            if self._nav_thread and self._nav_thread.is_alive():
                self._nav_thread.join(timeout=0.5)
            self._stop_nav.clear()
            self._nav_thread = threading.Thread(
                target=self._navigate_time_based, args=(target_room,), daemon=True
            )
            self._nav_thread.start()

    def _navigate_time_based(self, target_room: str):
        # Naive: drive forward for N seconds unless obstacle within THRESH, then slow/stop.
        secs = WAYPOINTS.get(target_room, 3.0)
        t0 = time.time()
        try:
            while not self._stop_nav.is_set() and (time.time() - t0) < secs:
                d = distance_filtered_cm(samples=3)
                if d is not None and d < THRESH_CM + SLOW_BAND:
                    # slow or stop
                    if d < THRESH_CM:
                        self.motors.stop()
                        time.sleep(0.1)
                        continue
                    else:
                        self.motors.forward(SLOW_SPEED)
                else:
                    self.motors.forward(CRUISE_SPEED)
                time.sleep(0.05)
            # stop at end and set location
            self.motors.stop()
            if not self._stop_nav.is_set():
                self._current_room = target_room
                self.speaker.say(f"Arrived at {target_room}.")
        finally:
            try:
                self.motors.stop()
            except Exception:
                pass

    def run(self):
        self.speaker.say("Voice control ready. Say 'go to kitchen', 'stop', or 'status'.")
        self.listener.start()
        try:
            while True:
                time.sleep(0.2)
        except KeyboardInterrupt:
            pass
        finally:
            self.listener.close()
            try:
                self.motors.shutdown()
            except Exception:
                pass

def main():
    # Expect a Vosk model unpacked at ./models/vosk (e.g., vosk-model-small-en-us-0.15)
    model_path = "models/vosk"
    vc = VoiceController(model_path)
    vc.run()

if __name__ == "__main__":
    main()
