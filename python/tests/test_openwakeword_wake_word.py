"""Standalone openWakeWord test / threshold-tuning harness.

Streams the default microphone and prints the live score for each loaded wake
word model, announcing a detection when a score crosses its threshold. Use it to
verify models and to tune their thresholds before wiring them into main.py.

Usage:
    # test a bundled pretrained sample (default: hey_jarvis)
    python test_openwakeword_wake_word.py
    python test_openwakeword_wake_word.py alexa
    python test_openwakeword_wake_word.py hey_mycroft

    # test your own trained models (name resolves in python/models, or pass a path)
    python test_openwakeword_wake_word.py hey_orange stop_now

Names are resolved (in order) as: an existing file path, then a *.onnx in
python/models/, then a bundled openWakeWord model in resources/models/. The base
feature models must be present once:
    python -c "import openwakeword; openwakeword.utils.download_models()"
"""
import os
import sys
import glob
import numpy as np
import pyaudio
import openwakeword
from openwakeword.model import Model

THRESHOLD = 0.5

models_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "models"))
oww_resources = os.path.join(os.path.dirname(openwakeword.__file__), "resources", "models")


def resolve_model(name):
    """Resolve a model name/path to an .onnx file path."""
    if os.path.isfile(name):
        return os.path.abspath(name)
    stem = os.path.splitext(os.path.basename(name))[0]
    # look in the project's models dir, then openWakeWord's bundled resources
    for search_dir in (models_dir, oww_resources):
        exact = os.path.join(search_dir, stem + ".onnx")
        if os.path.isfile(exact):
            return exact
        # allow short names like "hey_jarvis" -> "hey_jarvis_v0.1.onnx"
        matches = sorted(glob.glob(os.path.join(search_dir, stem + "*.onnx")))
        if matches:
            return matches[0]
    raise FileNotFoundError(
        "Could not resolve wake-word model '{}'. Looked for a file path, "
        "{}\\{}.onnx, and a bundled openWakeWord model.".format(name, models_dir, stem))


requested = sys.argv[1:]
if not requested:
    print("usage: {} <wake-word-model> [<wake-word-model> ...]".format(sys.argv[0]))
    sys.exit(1)
    
model_paths = [resolve_model(n) for n in requested]
print("Using models:")
for p in model_paths:
    print("   ", p)

# Windows openWakeWord only supports the onnx inference framework.
oww = Model(wakeword_models=model_paths, inference_framework="onnx")
model_keys = list(oww.models.keys())

# openWakeWord expects 16 kHz, 16-bit mono audio; it prefers 1280-sample (80 ms) frames.
SAMPLE_RATE = 16000
FRAME_LENGTH = 1280

pa = pyaudio.PyAudio()
stream = pa.open(format=pyaudio.paInt16,
                 channels=1,
                 rate=SAMPLE_RATE,
                 input=True,
                 frames_per_buffer=FRAME_LENGTH)

print("Loaded model keys:", model_keys)
print("Listening for wake words at threshold {}... (Ctrl+C to stop)".format(THRESHOLD))
try:
    while True:
        pcm = stream.read(FRAME_LENGTH, exception_on_overflow=False)
        audio = np.frombuffer(pcm, dtype=np.int16)
        preds = oww.predict(audio)
        scores = {k: float(preds[k]) for k in model_keys}  # type: ignore  # predict() returns {name: score}
        line = "  ".join("{}={:.2f}".format(k, scores[k]) for k in model_keys)
        print(line, end="\r", flush=True)
        for k in model_keys:
            if scores[k] >= THRESHOLD:
                print("\nDetected '{}' (score {:.2f})!".format(k, scores[k]))
                oww.reset()  # clear streaming buffers so we don't immediately re-trigger
                break
except KeyboardInterrupt:
    pass
finally:
    stream.stop_stream()
    stream.close()
    pa.terminate()
