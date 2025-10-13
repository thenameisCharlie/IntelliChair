"""
Simple text-to-speech wrapper using pyttsx3 (offline).
"""
import threading
import pyttsx3

class Speaker:
    def __init__(self, rate: int = 170):
        self.engine = pyttsx3.init()
        self.engine.setProperty("rate", rate)
        self._lock = threading.Lock()

    def say(self, text: str):
        with self._lock:
            self.engine.say(text)
            self.engine.runAndWait()
