
import pyttsx3

_engine = None

def speak(text: str):
    global _engine
    if _engine is None:
        _engine = pyttsx3.init()
        try:
            rate = _engine.getProperty('rate')
            _engine.setProperty('rate', int(rate * 1.05))
            _engine.setProperty('volume', 1.0)
        except Exception:
            pass
    _engine.say(text)
    _engine.runAndWait()