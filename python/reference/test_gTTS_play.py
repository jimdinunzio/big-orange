import io
import pygame
from gtts import *

# To play audio text-to-speech during execution
def speak(my_text):
    with io.BytesIO() as f:
        try:
            gTTS(text='',
                 lang='en',slow=False).write_to_fp(f)
        except Exception as e:
            print(e)
            if e != "No text to speak":
                _internet = False
            return
        f.seek(0)
        pygame.mixer.init()
        pygame.mixer.music.load(f)
        pygame.mixer.music.play()
        while pygame.mixer.music.get_busy():
            continue
        
speak("Hello, my name is orange.")
