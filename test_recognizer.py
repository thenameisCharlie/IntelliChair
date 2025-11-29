# import pytest
# pytest.skip("Skipping Vosk recognizer test — requires microphone and cannot run under pytest", allow_module_level=True)

import os, json, queue, sounddevice as sd
from vosk import Model, KaldiRecognizer

GRAMMAR = ["go to kitchen", "stop", "status", "[unk]"]
model = Model(os.environ.get("VOSK_MODEL_PATH", "/home/robotpi/models/vosk-en"))
rec = KaldiRecognizer(model, 16000, json.dumps(GRAMMAR))
q = queue.Queue()

def cb(indata, frames, time, status):
    q.put(bytes(indata))

dev = os.environ.get("MIC_DEVICE")
dev = int(dev) if (dev and dev.isdigit()) else None
print(f"[test] device={dev if dev is not None else 'default'}")
with sd.RawInputStream(samplerate=16000, channels=1, dtype='int16', blocksize=8000, callback=cb, device=dev):
    print("Listening… say: 'go to kitchen' / 'stop' / 'status' (Ctrl+C to quit)")
    while True:
        if rec.AcceptWaveform(q.get()):
            print("FINAL:", rec.Result())
