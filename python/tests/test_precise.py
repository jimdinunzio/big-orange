import speech_recognition as sr
import time
import collections
import math
import os
import pyaudio
import speech_recognition as sr
from speaker_pixel_ring import SpeakerPixelRing

def test():
    SAMPLING_RATE = 16000
    # create a recognizer object
    r = sr.Recognizer()

    def on_prediction(conf):
        #print('!' if conf > 0.7 else '.', end='', flush=True)
        print("%1.2f" % conf)
        pixel_ring.setPrediction(conf)

    r = sr.Recognizer()
    pConfig = r.PreciseListener.Config(model_file_path='models/hey-orange.pb', on_prediction=on_prediction)
    # create a microphone object
    mic = sr.Microphone(sample_rate=SAMPLING_RATE)
    pixel_ring = SpeakerPixelRing()
    p = pyaudio.PyAudio()

    try:
        with mic as source:
            pl = r.PreciseListener(source, pConfig)
            stream = p.open(format=source.format,
                            channels=1,
                            rate=source.SAMPLE_RATE,
                            output=True)
            try:
                while True:
                    buffer, delta_time = pl.wait_for_hot_word(source)
                    print(f"delta_time = {delta_time}")
                    # Play samples from the buffer (3)
                    #stream.write(buffer)
            except Exception as e:
                print(e)
    except Exception as e:
        print(e)

    # Close stream (4)
    stream.close()
    # Release PortAudio system resources (5)
    p.terminate()


if __name__ == '__main__':
    test()

