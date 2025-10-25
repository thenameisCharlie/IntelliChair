import os
import queue, json, threading
import sounddevice as sd
from vosk import Model, KaldiRecognizer

PHRASES = ["go to kitchen", "stop", "status"]

class CommandListener:
    def __init__(self, model_path: str, sample_rate: int = 16000):
        self.model = Model(model_path)
        grammar = json.dumps(PHRASES + ["[unk]"])
        self.rec = KaldiRecognizer(self.model, sample_rate, grammar)
        self.sample_rate = sample_rate
        self.q = queue.Queue()
        self.stop_flag = threading.Event()
        self.stream = None

    def _audio_cb(self, indata, frames, time, status):
        if status:
            # underrun/overrun etc. — keep going
            pass
        self.q.put(bytes(indata))

    def start(self):
        self.stop_flag.clear()
        # Allow overriding input device via env var
        device_env = os.environ.get("MIC_DEVICE", "").strip()
        device = int(device_env) if device_env.isdigit() else None

        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self._audio_cb,
            device=device,  # None uses default
        )
        self.stream.start()
        print(f"[voice] Listening on device={device if device is not None else 'default'} @ {self.sample_rate} Hz")

    def stop(self):
        self.stop_flag.set()
        try:
            if self.stream is not None:
                self.stream.stop()
                self.stream.close()
        except Exception:
            pass

    def listen(self):
        while not self.stop_flag.is_set():
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                try:
                    res = json.loads(self.rec.Result())
                except Exception:
                    continue
                text = (res.get("text") or "").strip()
                if not text:
                    continue
                for p in PHRASES:
                    if text == p:
                        yield p
                        break
