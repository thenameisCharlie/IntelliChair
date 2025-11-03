"""
Offline keyword listener using Vosk (no LLM). 
Recognizes:
- "go to <room>"
- "stop"
- "status"
"""
import json
import queue
import re
import sys
import threading
from typing import Callable, Dict, Optional

try:
    import sounddevice as sd  # lightweight mic capture
except Exception:
    sd = None

from vosk import Model, KaldiRecognizer


_GO_TO_RE = re.compile(r"\b(go\s*to)\s+(?P<room>[a-zA-Z ]+)\b")
_STATUS_RE = re.compile(r"\b(status|where\s+am\s+i|battery)\b")
_STOP_RE = re.compile(r"\b(stop|halt|abort)\b")

def _norm(text: str) -> str:
    return " ".join(text.lower().strip().split())

class KeywordListener:
    def __init__(
        self, 
        model_path: str, 
        samplerate: int = 16000,
        device: Optional[int] = None
    ) -> None:
        if sd is None:
            raise RuntimeError("sounddevice is not available; please `pip install sounddevice`")
        self.model = Model(model_path)
        self.rec = KaldiRecognizer(self.model, samplerate)
        self.samplerate = samplerate
        self.device = device
        self._stop = threading.Event()
        self._q: "queue.Queue[bytes]" = queue.Queue()

        self.on_go_to: Optional[Callable[[str], None]] = None
        self.on_stop: Optional[Callable[[], None]] = None
        self.on_status: Optional[Callable[[], None]] = None

    def _audio_cb(self, indata, frames, time, status):
        if status:
            # print(status, file=sys.stderr)
            pass
        self._q.put(bytes(indata))

    def start(self):
        self._stop.clear()
        self._thr = threading.Thread(target=self._run, daemon=True)
        self._thr.start()

    def close(self):
        self._stop.set()
        if hasattr(self, "_thr"):
            self._thr.join(timeout=1.0)

    def _run(self):
        with sd.RawInputStream(
            samplerate=self.samplerate,
            blocksize=8000,
            dtype="int16",
            channels=1,
            callback=self._audio_cb,
            device=self.device
        ):
            while not self._stop.is_set():
                try:
                    data = self._q.get(timeout=0.1)
                except queue.Empty:
                    continue
                if self.rec.AcceptWaveform(data):
                    try:
                        result = json.loads(self.rec.Result())
                    except json.JSONDecodeError:
                        continue
                    text = _norm(result.get("text", ""))
                    if not text:
                        continue
                    # print(f"[heard] {text}")
                    self._dispatch(text)
                else:
                    # partial = json.loads(self.rec.PartialResult()).get("partial", "")
                    pass

    def _dispatch(self, text: str):
        # stop first (high priority safety)
        if _STOP_RE.search(text):
            if self.on_stop: self.on_stop()
            return
        # go to room
        m = _GO_TO_RE.search(text)
        if m and self.on_go_to:
            room = _norm(m.group("room"))
            self.on_go_to(room)
            return
        # status last
        if _STATUS_RE.search(text):
            if self.on_status: self.on_status()
            return
        # else: ignore unrecognized
