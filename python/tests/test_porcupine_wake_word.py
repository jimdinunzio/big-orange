import pvporcupine
import pyaudio
import os

# AccessKey obtained from Picovoice Console (https://console.picovoice.ai/)
access_key = os.getenv("PORCUPINE_ACCESS_KEY")

hey_orange_keyword_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../models", "Hey-Orange_en_windows_v3_0_0.ppn"))

porcupine = pvporcupine.create(
    access_key=access_key,
    keyword_paths=[hey_orange_keyword_path, pvporcupine.KEYWORD_PATHS['grapefruit']],  # None for built-in, path for custom
    sensitivities=[0.25, 0.5]  # Sensitivity for each keyword
)

pa = pyaudio.PyAudio()
stream = pa.open(format=pyaudio.paInt16,
                 channels=1,
                 rate=porcupine.sample_rate,
                 input=True,
                 frames_per_buffer=porcupine.frame_length)

try:
    print("Listening for wake words...")
    import struct
    while True:
        pcm = stream.read(porcupine.frame_length, exception_on_overflow=False)
        pcm = struct.unpack_from("h" * porcupine.frame_length, pcm)
        result = porcupine.process(pcm)
        if result >= 0:
            print("Wake word, #{} detected!".format(result))
except KeyboardInterrupt:
    pass
finally:
    stream.stop_stream()
    stream.close()
    porcupine.delete()