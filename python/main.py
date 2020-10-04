# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 17:39:44 2020

@author: LattePanda
"""

_locations = { "kitchen" : (9.157, -3.269), "living room" : (0.061, -3.775), 
              "dining room" : (3.329, -2.849),  "office" : (0,0)}
import tts.sapi
import time
import ctypes
from ctypes import cdll
import os
import sys

def speak(phrase):
    global _last_phrase, voice

    voice = tts.sapi.Sapi()
    print(phrase)
    #voice.set_voice("Andy")
    voice.voice.Volume = 80
    voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds

    try:
        voice.say(phrase)
    except Exception:
        print("Speak has timed out.")
        pass

    if phrase != "I said":
        _last_phrase = phrase
        
def monitorMoveAction():
    while (1):
        status = slamtec.getMoveActionStatus()
        print("status = ", status)
        if status == ActionStatus.Finished.value:
            print(slamtec.getMoveActionError())
            break;
        elif status == ActionStatus.Error.value:
            print(slamtec.getMoveActionError())
            break
        elif status == ActionStatus.Stopped.value:
            break;
        time.sleep(1)

from enum import Enum
class ActionStatus(Enum):
    def __init__(self, number):
        self._as_parameter__ = number
        
    # The action is created but not started
    WaitingForStart = 0
    # The action is running
    Running = 1
    # The action is finished successfully
    Finished = 2
    # The action is paused
    Paused = 3
    # The action is stopped
    Stopped = 4
    # The action is under error
    Error = 5        

def gotoLocation(location):
    goal = _locations.get(location);
    if goal:
        print("unknown location")
        return;
    slamtec.moveToFloat(goal[0], goal[1])

def speak(phrase):
    import tts.sapi
    global _last_phrase, voice

    voice = tts.sapi.Sapi()
    print(phrase)
    #voice.set_voice("Andy")
    voice.voice.Volume = 80
    voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds

    try:
        voice.say(phrase)
    except Exception:
        print("Speak has timed out.")
        pass

    if phrase != "I said":
        _last_phrase = phrase

        
if __name__ == '__main__':
    DllsPath = os.getcwd() + r"\..\DLLs\\"
    
    cdll.LoadLibrary(DllsPath + "libeay32.dll")
    cdll.LoadLibrary(DllsPath + "ssleay32.dll")
    
    slamtec = cdll.LoadLibrary(DllsPath + "SlamtecDll.dll")
    
    # Set up C function input and output types
    slamtec.connectSlamtec.argtypes = ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p
    slamtec.SlamtecLocation.restype = ctypes.c_char_p
    
    slamtec.loadSlamtecMap.argtypes = ctypes.c_char_p,
    slamtec.loadSlamtecMap.restype = ctypes.c_int
    
    slamtec.movetoFloat.argtypes = ctypes.c_float, ctypes.c_float
    slamtec.movetoFloat.restype = None
    slamtec.home.restype = None
    slamtec.disconnect.restype = None
    slamtec.getMoveActionStatus.restype = ctypes.c_int
    
    #speak("hello")

    errStr = ctypes.create_string_buffer(255)
    res = slamtec.connectSlamtec(b"192.168.11.1", 1445, errStr, 255);
    if res == 1 :
        print("Could not connect to SlamTec, time out: ", errStr.value)
        sys.exit()
    elif res == 2:
        print("Could not connect to SlamTec, connection fail: ", errStr.value)
        sys.exit()
    
    slamtec.wakeup()
    
    #print("Loading map")
    #res = slamtec.loadSlamtecMap(b'HCR-MyHouse.stcm')
    #print("Done loading map")
    
    #time.sleep(5)
    
    # Ready to take commands
    
    print("Moving to location")
    
    slamtec.movetoFloat(0.0, 0.0)
    #gotoLocation("dining room")
    monitorMoveAction()
    locString = slamtec.SlamtecLocation()
    print(locString)   
    
    #time.sleep(10)
    
    #slamtec.home()
    #monitorMoveAction()
        
    #locString = slamtec.SlamtecLocation()
    #print(locString)   
    
    slamtec.disconnect()
