# voice/command_listener.py
import os
import json
import queue
import time
import threading
from pathlib import Path
from typing import Generator, Optional

import sounddevice as sd
from vosk import Model, KaldiRecognizer

# Fixed phrases; expand as needed
PHRASES = ["stop", "status", "kitchen", "go to kitchen", "go to"] 

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

        # grammar = json.dumps(PHRASES)
        # self.rec = KaldiRecognizer(self.model, self.sample_rate_fallback, grammar)

        grammar = json.dumps(self._build_vocab())
        self.rec = KaldiRecognizer(self.model, self.sample_rate_fallback, grammar)
        self.rec.SetWords(True)
        self.rec.SetMaxAlternatives(3)

        # Debounce tracking
        self._last_emit: dict[str, float] = {}
        self._debounce_sec = 1.2  # suppress duplicate emits within this window
        self.paused = False       

    def _audio_cb(self, indata, frames, time_info, status):
        if self.paused:
            return
        if status:
            pass
        self.q.put(bytes(indata))

    def _should_emit(self, phrase: str) -> bool:
        now = time.time()
        last = self._last_emit.get(phrase, 0.0)
        if now - last >= self._debounce_sec:
            self._last_emit[phrase] = now
            return True
        return False
    
    def pause(self):
        """Temporarily ignore mic input."""
        self.paused = True

    def resume(self):
        """Resume mic and clear any old buffered audio."""
        self.paused = False
        self.flush_queue()
    
    def _build_vocab(self):
        """
        Build a small word-level vocabulary for Vosk grammar:
        hotwords + room tokens from navigation/places.json.
        Allows phrases like "go to living room" to be recognized word-by-word.
        """
        vocab = {"go", "to", "stop", "status", "room", "the", "take", "me", "please", "bring", "move"}

        # Load rooms from places.json
        try:
            repo_root = Path(__file__).resolve().parents[1]  # .../IntelliChair
            places_path = repo_root / "navigation" / "places.json"
            with open(places_path, "r") as f:
                data = json.load(f)  # expects {"kitchen": {...}, "living room": {...}, ...}
            room_names = list(data.keys())
        except Exception:
            room_names = []

        # Split multi-word rooms into tokens ("living room" -> "living","room")
        for r in room_names:
            for tok in r.lower().split():
                if tok:
                    vocab.add(tok)

        # Return a sorted list for stable output
        return sorted(vocab)

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
        
        self.current_sr = sr

        # Rebuild recognizer for this samplerate
        # grammar = json.dumps(PHRASES)
        # self.rec = KaldiRecognizer(self.model, sr, grammar)

        grammar = json.dumps(self._build_vocab())
        self.rec = KaldiRecognizer(self.model, sr, grammar)
        self.rec.SetWords(True)
        self.rec.SetMaxAlternatives(3)

        # Small blocks for fast turnaround: ~60–125 ms
        # Prefer ~sr/16 (≈ 62.5ms @ 44100), but keep a sane minimum
        blocksize = max(1024, int(sr / 32)) # was int(sr / 16)

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

                # If Vosk finalized but gave us nothing (common with "go to room"),
                # force the controller into clarify mode.
                if not text:
                    if self._should_emit("__FREEFORM__"):
                        yield "__FREEFORM__"
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

            # else:
            #     # PARTIAL result (fires often)
            #     if not PARTIAL_ENABLE:
            #         continue
            #     try:
            #         pres = json.loads(self.rec.PartialResult())
            #     except Exception:
            #         continue

            #     partial = (pres.get("partial") or "").strip()
            #     if DEBUG and partial:
            #         print(f"[voice] PARTIAL: '{partial}'")
            #     if not partial:
            #         continue

            #     # 1) Highest priority: "stop" reacts immediately
            #     if partial == "stop" or partial.endswith(" stop"):
            #         if self._should_emit("stop"):
            #             yield "stop"
            #         continue

            #     # 2) If we clearly have a multi-word phrase (not dangling "go"/"go to"),
            #     #    surface it so the controller can use it (AI or direct).
            #     if " " in partial and not partial.endswith((" go", " go to")):
            #         if self._should_emit(partial):
            #             yield partial
            #         continue

            #     # 3) PROMOTE COMMON HANGS:
            #     #    Vosk often leaves "go to room" as partials "go", "to", "room" and then FINAL="".
            #     #    When we hear "go to" or just "room" alone, force the controller into clarify mode.
            #     if partial in {"go to", "room"}:
            #         if self._should_emit(f"promote:{partial}"):
            #             # Yield "go to" so voice_control treats it as an incomplete command
            #             yield "go to"
            #         continue

            #     # 4) Treat one-word hotwords as triggers (e.g., "kitchen")
            #     #    Only if it's in your fixed list.
            #     if partial in PHRASES and self._should_emit(partial):
            #         yield partial           
            # else:
            #     # PARTIAL result (fires often)
            #     if not PARTIAL_ENABLE:
            #         continue
            #     try:
            #         pres = json.loads(self.rec.PartialResult())
            #     except Exception:
            #         continue
            #     partial = (pres.get("partial") or "").strip()
            #     if DEBUG and partial:
            #         print(f"[voice] PARTIAL: '{partial}'")
            #     if not partial:
            #         continue
            #     # If we clearly heard "stop" in partials, emit immediately
            #     # (handles short utterance not producing a final result)
            #     if partial == "stop" or partial.endswith(" stop"):
            #         if self._should_emit("stop"):
            #             yield "stop"

    #function just added
    def listen_freeform_once(self, seconds: float = 6.0):
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
    
    def flush_queue(self):
        """Empty pending audio so the next listen starts fresh."""
        try:
            while True:
                self.q.get_nowait()
        except queue.Empty:
            pass


