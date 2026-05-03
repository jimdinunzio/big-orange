# -*- coding: utf-8 -*-
"""
Created on Sat Oct 17 23:46:32 2020

@author: LattePanda
"""

import io
from IPython.display import Audio
from gtts import gTTS

# To play audio text-to-speech during execution
def speak(my_text):
    with io.BytesIO() as f:
        gTTS(text=my_text, lang='en').write_to_fp(f)
        f.seek(0)
        return Audio(f.read(), autoplay=True)

