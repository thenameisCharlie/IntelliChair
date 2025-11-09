# voice/command_listener.py
import os
import json
import queue
import time
import threading
from typing import Generator, Optional

import sounddevice as sd
from vosk import Model, KaldiRecognizer

# Fixed phrases; expand as needed
PHRASES = ["go to kitchen", "kitchen", "stop", "status"]

# Debug flags (set VOICE_DEBUG=1 in env to enable)
DEBUG = os.environ.get("VOICE_DEBUG", "0") == "1"
# Enable reacting to partial results (helps with very short words like "stop")
PARTIAL_ENABLE = os.environ.get("VOICE_PARTIAL", "1") == "1"

class CommandListener:
    """
    Offline keyword listener using Vosk.
    - Auto-detects device samplerate (e.g., 44100/48000).
    - Small blocksize for low latency.
    - Reacts to FINAL results; optionally also to PARTIAL for short words like "stop".
    - Debounces identical triggers.
    """
    def __init__(self, model_path: str, sample_rate_fallback: int = 16000):
        self.model = Model(model_path)
        self.sample_rate_fallback = int(sample_rate_fallback)
        self.q: "queue.Queue[bytes]" = queue.Queue()
        self.stop_flag = threading.Event()
        self.stream: Optional[sd.RawInputStream] = None

        grammar = json.dumps(PHRASES)
        self.rec = KaldiRecognizer(self.model, self.sample_rate_fallback, grammar)

        # Debounce tracking
        self._last_emit: dict[str, float] = {}
        self._debounce_sec = 0.75  # suppress duplicate emits within this window

    def _audio_cb(self, indata, frames, time_info, status):
        if status:
            # buffer over/underrun warnings are okay
            pass
        self.q.put(bytes(indata))

    def _should_emit(self, phrase: str) -> bool:
        now = time.time()
        last = self._last_emit.get(phrase, 0.0)
        if now - last >= self._debounce_sec:
            self._last_emit[phrase] = now
            return True
        return False

    def start(self):
        self.stop_flag.clear()

        device_env = os.environ.get("MIC_DEVICE", "").strip()
        device = int(device_env) if device_env.isdigit() else None

        # Detect samplerate
        try:
            info = sd.query_devices(device)
            sr = int(info.get("default_samplerate") or self.sample_rate_fallback)
            self.current_sr = sr
        except Exception:
            sr = self.sample_rate_fallback

        # Rebuild recognizer for this samplerate
        grammar = json.dumps(PHRASES)
        self.rec = KaldiRecognizer(self.model, sr, grammar)

        # Small blocks for fast turnaround: ~60–125 ms
        # Prefer ~sr/16 (≈ 62.5ms @ 44100), but keep a sane minimum
        blocksize = max(1024, int(sr / 16))

        self.stream = sd.RawInputStream(
            samplerate=sr,
            blocksize=blocksize,
            dtype="int16",
            channels=1,
            callback=self._audio_cb,
            device=device,
        )
        self.stream.start()
        print(f"[voice] Listening on device={device if device is not None else 'default'} "
              f"@ {sr} Hz (blocksize={blocksize})")

    def stop(self):
        self.stop_flag.set()
        try:
            if self.stream is not None:
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass

    def listen(self) -> Generator[str, None, None]:
        """
        Yield matched phrases. Reacts to FINAL results; optionally to PARTIAL 'stop'
        for snappy emergency halts.
        """
        while not self.stop_flag.is_set():
            # data = self.q.get()
            
            try:
                data = self.q.get(timeout=0.2)
            except queue.Empty:
                continue

            if self.rec.AcceptWaveform(data):
                # FINAL result
                try:
                    res = json.loads(self.rec.Result())
                except Exception:
                    continue
                text = (res.get("text") or "").strip()
                if DEBUG:
                    print(f"[voice] FINAL: '{text}'")
                if not text:
                    continue

                for p in PHRASES:
                    if text == p and self._should_emit(p):
                        yield p
                        break
                else:
                    # Not one of the fixed phrases.
                    if text and text != "[unk]" and self._should_emit(text):
                        yield text
                    elif text == "[unk]":
                        # Force the controller to switch to free-form capture
                        yield "__FREEFORM__"
                            
            else:
                # PARTIAL result (fires often)
                if not PARTIAL_ENABLE:
                    continue
                try:
                    pres = json.loads(self.rec.PartialResult())
                except Exception:
                    continue
                partial = (pres.get("partial") or "").strip()
                if DEBUG and partial:
                    print(f"[voice] PARTIAL: '{partial}'")
                if not partial:
                    continue
                # If we clearly heard "stop" in partials, emit immediately
                # (handles short utterance not producing a final result)
                if partial == "stop" or partial.endswith(" stop"):
                    if self._should_emit("stop"):
                        yield "stop"

    #function just added
    def listen_freeform_once(self, seconds: float = 3.5):
        """
        Capture ~N seconds of free-form speech (no grammar) and return one text string.
        Uses the same audio stream/queue; temporary recognizer without grammar.
        """
        import json as _json
        if self.stream is None:
            return None

        # Use the current samplerate detected in start(); fall back if missing.
        sr = getattr(self, "current_sr", self.sample_rate_fallback)
        temp_rec = KaldiRecognizer(self.model, sr)

        deadline = time.time() + seconds
        final_text = ""

        while time.time() < deadline:
            try:
                data = self.q.get(timeout=0.2)
            except Exception:
                continue
            if temp_rec.AcceptWaveform(data):
                try:
                    res = _json.loads(temp_rec.Result())
                    final_text = (res.get("text") or "").strip()
                except Exception:
                    pass

        # flush any remainder
        try:
            res = _json.loads(temp_rec.FinalResult())
            if not final_text:
                final_text = (res.get("text") or "").strip()
        except Exception:
            pass

        return final_text or None


