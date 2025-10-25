
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
            pass
        self.q.put(bytes(indata))

    def start(self):
        self.stop_flag.clear()
        self.stream = sd.RawInputStream(
            samplerate=self.sample_rate,
            blocksize=8000,
            dtype='int16',
            channels=1,
            callback=self._audio_cb
        )
        self.stream.start()

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