import tts.sapi
import tts.flags
import time
import threading

def initialize_speech():
    global _voice
    _voice = tts.sapi.Sapi()
    _voice.set_voice("Mark") # David, Mark, Eva, or Zira. 
    _voice.voice.Volume = 100
    _voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds


def wait_until_done():
    print("waiting until speech done")
    while _voice.voice.WaitUntilDone(100) == False:
        pass
    print("speech done")

def cancel_speaking():
    print("cancelling speaking")
    _voice.voice.Speak("", tts.flags.SpeechVoiceSpeakFlags.PurgeBeforeSpeak.value)

def speak_async(phrase):
    speak(phrase, tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value, add_to_memory=False)

def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value, add_to_memory=True):
    try:
        #print("SPEAKING: ", phrase)
        _voice.say(phrase, flag)
        # add robot response to memory
    except Exception:
        print("Speak has timed out.")
        pass


initialize_speech()

threading.Thread(target=lambda: (time.sleep(3), cancel_speaking())).start()

speak_async("This is an asynchronous speech test. You should hear this message immediately.")
wait_until_done()



speak_async("testing queued speech. If you can hear me, you heard the second phrase")
speak_async("again testing queued speech. If you can hear me, you heard the third phrase")

# time.sleep(3)
# cancel_speaking()

