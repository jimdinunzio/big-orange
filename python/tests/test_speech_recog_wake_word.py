import sys
import os

# allow importing from parent directory
sys.path.append(os.path.abspath(os.path.join('..')))

os.chdir(os.path.abspath(os.path.join('..')))

import speech_recognition as sr
import pyaudio
from speaker_pixel_ring import SpeakerPixelRing
from pvporcupine import KEYWORD_PATHS
import time

def test():
    HEY_ORANGE_KEYWORD_IDX = 0
    STOP_NOW_KEYWORD_IDX = 1
    SAMPLING_RATE = 16000
    CHUNK_SIZE = 512
    # create a recognizer object
    r = sr.Recognizer()
    stop_flag = False

    def on_detection(index):
        if index == HEY_ORANGE_KEYWORD_IDX:
            for i in range(1, 12):
                pixel_ring.setColoredVolume(i)
                time.sleep(0.005)
        elif index == STOP_NOW_KEYWORD_IDX:
            pixel_ring.setRedVolume()
            nonlocal stop_flag
            stop_flag = True
        
    def on_listen_timeout(index):
        if index == HEY_ORANGE_KEYWORD_IDX:
            for i in range(11, -1, -1):
                pixel_ring.setColoredVolume(i)
                time.sleep(0.005)

    r = sr.Recognizer()
    keyword_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "../models", "Hey-Orange_en_windows_v3_0_0.ppn"))
    access_key = os.getenv("PORCUPINE_ACCESS_KEY")
    assert access_key is not None, "No Porcupine access key set. Hot word detection test cannot be done."
    
    pConfig = r.PorcupineListener.Config(access_key=access_key,
                                         keyword_paths=[keyword_path, KEYWORD_PATHS['grapefruit']],
                                         keyword_types=[r.PorcupineListener.KeywordType.LISTEN, r.PorcupineListener.KeywordType.IMMEDIATE],
                                         sensitivities=[0.25, 0.5],
                                         on_detection=on_detection,
                                         on_det_timeout=on_listen_timeout)
    # create a microphone object
    mic = sr.Microphone(sample_rate=SAMPLING_RATE, chunk_size=CHUNK_SIZE)


    pixel_ring = SpeakerPixelRing()
    p = pyaudio.PyAudio()

    try:
        with mic as source:
            print("Say something!")
            assert source.format == pyaudio.paInt16, "Microphone must be in 16bit format."
            pl = r.PorcupineListener(source, pConfig)
            stream = p.open(format=source.format,
                            channels=1,
                            rate=source.SAMPLE_RATE,
                            frames_per_buffer=source.CHUNK,
                            input=True)
            try:
                while True:
                    result, keyword_type = pl.wait_for_hot_word()
                    if result == HEY_ORANGE_KEYWORD_IDX:
                        print("Hot word detected!")
                    elif result == STOP_NOW_KEYWORD_IDX:
                        print("Stop now detected!")
                    print("Keyword type: ", keyword_type)
                    if stop_flag:
                        break
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

