#!/usr/bin/env python3

# NOTE: this example requires PyAudio because it uses the Microphone class

import speech_recognition as sr
import usb.core
from mic_array_tuning import Tuning
import json

class MicArray(object):
    def __init__(self):
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if not self.dev:
            raise RuntimeError("Error, could not initialize mic array.")
        self.tuning = Tuning(self.dev)

    def getDoa(self):
        return self.tuning.direction
    
    def getIsSpeech(self):
        return self.tuning.is_speech()

    def doa2YawDelta(self, doa):
        yawDelta = doa - 90
        if yawDelta >= 180:
            yawDelta = yawDelta - 360
        return yawDelta

def test():
    m = MicArray()
    # obtain audio from the microphone
    r = sr.Recognizer()
    #r.energy_threshold = 100
    #r.dynamic_energy_threshold = False

    while True:
        with sr.Microphone() as source:
            print("Say something!")
            try:
                r.adjust_for_ambient_noise(source, is_speech_cb=m.getIsSpeech)
                print("threshold = ", r.energy_threshold)
                audio = r.listen(source, phrase_time_limit=7, is_speech_cb=m.getIsSpeech)
                #phrase = r.recognize_google(audio)
                result = r.recognize_vosk(audio, arg2='[ "orange", "[unk]" ]', alts=1)
                print(result)
                phrase = json.loads(result)
                phrase = phrase["alternatives"][0]["text"]
                print("Vosk recognizer thinks you said \"", phrase, "\"")
            except sr.UnknownValueError:
                print("Vosk recognizer could not understand audio")
            except sr.RequestError as e:
                print("Vosk recognizer error; {0}".format(e))
            except sr.WaitTimeoutError:
                print("listening timed out")
            except KeyboardInterrupt:
                break

# # recognize speech using Google Cloud Speech
# GOOGLE_CLOUD_SPEECH_CREDENTIALS = r"""INSERT THE CONTENTS OF THE GOOGLE CLOUD SPEECH JSON CREDENTIALS FILE HERE"""
# try:
#     print("Google Cloud Speech thinks you said " + r.recognize_google_cloud(audio, credentials_json=GOOGLE_CLOUD_SPEECH_CREDENTIALS))
# except sr.UnknownValueError:
#     print("Google Cloud Speech could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Google Cloud Speech service; {0}".format(e))

# # recognize speech using Wit.ai
# WIT_AI_KEY = "INSERT WIT.AI API KEY HERE"  # Wit.ai keys are 32-character uppercase alphanumeric strings
# try:
#     print("Wit.ai thinks you said " + r.recognize_wit(audio, key=WIT_AI_KEY))
# except sr.UnknownValueError:
#     print("Wit.ai could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Wit.ai service; {0}".format(e))

# # recognize speech using Microsoft Bing Voice Recognition
# BING_KEY = "INSERT BING API KEY HERE"  # Microsoft Bing Voice Recognition API keys 32-character lowercase hexadecimal strings
# try:
#     print("Microsoft Bing Voice Recognition thinks you said " + r.recognize_bing(audio, key=BING_KEY))
# except sr.UnknownValueError:
#     print("Microsoft Bing Voice Recognition could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Microsoft Bing Voice Recognition service; {0}".format(e))

# # recognize speech using Microsoft Azure Speech
# AZURE_SPEECH_KEY = "INSERT AZURE SPEECH API KEY HERE"  # Microsoft Speech API keys 32-character lowercase hexadecimal strings
# try:
#     print("Microsoft Azure Speech thinks you said " + r.recognize_azure(audio, key=AZURE_SPEECH_KEY))
# except sr.UnknownValueError:
#     print("Microsoft Azure Speech could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Microsoft Azure Speech service; {0}".format(e))

# # recognize speech using Houndify
# HOUNDIFY_CLIENT_ID = "INSERT HOUNDIFY CLIENT ID HERE"  # Houndify client IDs are Base64-encoded strings
# HOUNDIFY_CLIENT_KEY = "INSERT HOUNDIFY CLIENT KEY HERE"  # Houndify client keys are Base64-encoded strings
# try:
#     print("Houndify thinks you said " + r.recognize_houndify(audio, client_id=HOUNDIFY_CLIENT_ID, client_key=HOUNDIFY_CLIENT_KEY))
# except sr.UnknownValueError:
#     print("Houndify could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from Houndify service; {0}".format(e))

# # recognize speech using IBM Speech to Text
# IBM_USERNAME = "INSERT IBM SPEECH TO TEXT USERNAME HERE"  # IBM Speech to Text usernames are strings of the form XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX
# IBM_PASSWORD = "INSERT IBM SPEECH TO TEXT PASSWORD HERE"  # IBM Speech to Text passwords are mixed-case alphanumeric strings
# try:
#     print("IBM Speech to Text thinks you said " + r.recognize_ibm(audio, username=IBM_USERNAME, password=IBM_PASSWORD))
# except sr.UnknownValueError:
#     print("IBM Speech to Text could not understand audio")
# except sr.RequestError as e:
#     print("Could not request results from IBM Speech to Text service; {0}".format(e))
