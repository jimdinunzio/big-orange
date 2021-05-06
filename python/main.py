# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 17:39:44 2020

@author: LattePanda
"""

# Constants
_hotword = "orange"
_privacy_mode = True
_map_filename_str = b'MyHouse.stcm'
_sdp_ip_address = b"192.168.11.1"
_sdp_port = 1445
_execute = True # False for debugging, must be True to run as: >python main.py
_run_flag = True # setting this to false kills all threads for shut down
_eyes_flag = False # should eyes be displayed or not
_person = "Jim"
_new_person_flag = False
_people = {"Evi":5, "Jim":8, "stranger":1, "nobody":0}
_mood = "happy"
_moods = {"happy":50, "bored":20, "hungry":10}
_locations = { "kitchen" : (11.357, -4.094), "kitchen nook": (8.764, -4.696), 
              "living room" : (0.875, -2.435), "back slider": (0.53, -2.90), 
              "dining area" : (6.3, -3.457), "office" : (1.934, 0.163), 
              "front door" : (11.722, -1.355), "end of hall" : (6.932, 0.011), 
              "home" : (0.0, 0.0), "custom" : (float("NaN"), float("NaN")) }

_HOUSE_RECT = {"left":-0.225,"bottom":-5.757, "width":12.962, "height":7.6}
_OFFICE_RECT = {"left":-0.225,"bottom":-0.3, "width":4.34, "height":2.144}
_INIT_RECT =  {"left":-0.5,"bottom":-0.5, "width":1.0, "height":1.0}
_STARTUP_ROOM = _OFFICE_RECT

# Globals
_goal = ""
_goal_queue = []
_time = "morning"
_times = ["morning", "noon", "afternoon", "evening", "night"]
_last_phrase = "nothing"
_listen_flag = True
_action = "" # this will be set to whatever specific action is being attempted
_action_flag = False # True means some action is in progress
_interrupt_action = False # True when interrupting a previously started action
_request = "" # comes from someone telling the robot to do something
_thought = "" # comes from the robot thinking that it wants to do something
_do_something_flag = False
_motion_flag = False  # set by monitor_motion thread, looking for humans in motion
_robot_is_moving = False # set by robotMotion thread: True when robot is in motion
_last_motion_time = 0 # the time.time() motion was last detected
_ser6 = ""
_internet = False # True when connected to the internet

import parse
import tts.sapi
import time
from ctypes import *
import os
import sys
import threading
from threading import Thread
from playsound import playsound
from word2number import w2n
import math
import io
#import pygame
#from gtts import gTTS
from my_sdp_client import MyClient
from my_sdp_server import *
import cv2
import ai_vision.detect as detect
import ai_vision.classify as classify
import winspeech
from enum import Enum

class HandleResponseResult(Enum):
    """Enumerated type for result of handling response of command"""
    def __init__(self, number):
        self._as_parameter__ = number
        
    # The response was not handled because the request was unknown
    NotHandledUnknown = -1
    # The response was not handle because no hot word was given
    NotHandledNoHotWord = 0
    # The response was handled
    Handled = 1

###############################################################
# Text input to Google Assistant for web based queries

import logging
import json
import click
import google.auth.transport.grpc
import google.auth.transport.requests
import google.oauth2.credentials

from google.assistant.embedded.v1alpha2 import (
    embedded_assistant_pb2,
    embedded_assistant_pb2_grpc
)


###############################################################
# Movement releated
    
def getMoveActionStatus():
    global _sdp
    status = _sdp.getMoveActionStatus()
    if status == ActionStatus.Finished:
        print("move action finished.")
    elif status == ActionStatus.Error:
        errStr = _sdp.getMoveActionError()
        print("move action error: ", errStr)
    elif status == ActionStatus.Stopped:
        print("action has been cancelled.")
    return status

################################################################
def turn(degrees):
    global _sdp, _action_flag, _action, _robot_is_moving
    if degrees == 0:
        return
    # if already in action, ignore this
    if _action_flag:
        return

    print("rotating ", degrees, "degrees")
    _action_flag = True # The robot is being commanded to move
    _sdp.rotate(math.radians(degrees))

    result = _sdp.waitUntilMoveActionDone()
    if result == ActionStatus.Error:
        speak("Something is wrong. I could not turn.")
        
    _action_flag = False # the robot's done turning
    _action = ""

################################################################
# function to calculate the distance between 2 points (XA YA) and (XB YB)
def distance_A_to_B(XA, YA, XB, YB):
    dist = math.sqrt((XB - XA)**2 + (YB - YA)**2)
    return dist


#x == _sdp.getX() and
#y == _sdp.getY()
################################################################

def nearest_location(x, y):
    global _locations
    
    location = ""
    distance = 1000000
    for loc, coord in _locations.items():
        dist = distance_A_to_B(x, y, coord[0], coord[1])
        if ( dist < distance):
            location = loc
            distance = dist
    return location, distance
 

################################################################
# if the robt is within half a meter of the goal, then success:
# return the closest location and the distance to it and if it is close enough
def where_am_i():
    global _sdp
    
    try:
        pose = _sdp.pose()
    except:
        return "unknown", 0, False
    location, distance = nearest_location(pose.x, pose.y)
    if (distance <= 100):
        closeEnough = True
    else:
        closeEnough = False
    return location, distance, closeEnough

################################################################
# This cancels an ongoing action - which may be a goto or something else.


def cancelAction(interrupt = False):
    global _sdp, _action_flag, _action, _interrupt_action
    if interrupt:
        _interrupt_action = True
    for attempt in range(3):
        try:
            _sdp.cancelMoveAction()
            _action_flag = False
            _action = ""
        except:            
            print("An error occurred canceling the action, trying again.")
            time.sleep(0.1)
        else:
            break
    return

def startrun():
    global _run_flag
    _run_flag = True


def stoprun():
    global _run_flag
    _run_flag = False


def testgoto(str):
    global _goal
    _goal = str

def batteryMonitor():
    global _sdp
    try:
        batteryPercent = _sdp.battery()
        if batteryPercent < 50:
            speak(_person + ", F.Y.I., my battery is about half full.")
        elif batteryPercent < 25:
            speak(_person + ", my battery is getting low. I'll have to charge up soon.")
        elif batteryPercent < 10:
            speak(_person + ", I need to recharge my battery now or I will have to shut down.")
        elif batteryPercent < 5:
            speak(_person + ", my battery is exhausted, and I am shutting down now.") 
            _run_flag = False
            os.system("shutdown /s /t 10")
    except:
        None
    time.sleep(10)
    
################################################################
# This is where goto actions are initiated and get carried out.
# If the robot is in the process of going to a  location, but
# another goto action request is receved, the second request will
# replace the earlier request

def handleGotoLocation():
    global _run_flag, _goal, _action_flag, _sdp, _interrupt_action
    while _run_flag:
        if _goal == "" or _action_flag:
            # no goal or some action is currently in progress, so sleep.
            time.sleep(0.5)
            continue

        print("I'm free and A new goal arrived: ", _goal)
        if _goal == "home":
            speak("I'm going home")
            _sdp.home()
        else:
            coords = _locations.get(_goal)
            if coords is None:
                speak("Sorry, I don't know how to get there.")
                print("unknown location")
                _goal = ""
                if len(_goal_queue) > 0:
                    _goal = _goal_queue.pop(0)
                continue
            if _goal != "custom":
                location, distance, closeEnough = where_am_i()
                if _goal == location and distance < 50:
                    speak("I'm already at the " + _goal)
                    _goal = ""
                    if len(_goal_queue) > 0:
                        _goal = _goal_queue.pop(0)
                    continue
                speak("I'm going to the " + _goal)
            else:
                speak("OK.")
            _action_flag = True
            _sdp.moveToFloat(coords[0], coords[1])

        _interrupt_action = False
        while(_interrupt_action == False):
            maStatus = getMoveActionStatus()
            if maStatus == ActionStatus.Stopped or \
                maStatus == ActionStatus.Error or \
                maStatus == ActionStatus.Finished:
                break
            time.sleep(0.5)

        if _interrupt_action == True:
            _interrupt_action = False
            continue
        # reaching this point, the robot first moved, then stopped - so check where it is now
        location, distance, closeEnough = where_am_i()

        # and now check to see if it reached the goal
        if (location == _goal and closeEnough):
            speak("I've arrived!")
        elif _goal != "custom":
            speak("Sorry, I didn't make it to the " + _goal)
        else:
            speak("Sorry, I didn't make it to where you wanted.")
        
        # finally clear the _action and _action_flag
        _action_flag = False # you've arrived somewhere, so no further action
        _action = ""            
        _goal = ""
        if len(_goal_queue) > 0:
            _goal = _goal_queue.pop(0)
        time.sleep(0.5)
        
################################################################
# Pretty print all currently active robot threads
def pretty_print_threads():
    i = 1
    for item in threading.enumerate():
        print(i,":",item)
        i += 1

################################################################
#ts = time.localtime()
#print(time.strftime("%H", ts)) # a 24 hour clock hour only
# This runs in it's own thread updating the time every ten seconds
def time_update():
    global _run_flag, _time
    while _run_flag:
        hour = int(time.strftime("%H", time.localtime()))
        if hour >= 6 and hour < 12:
            _time = "morning"
        elif hour == 12:
            _time = "noon"
        elif hour > 12 and hour < 17:
            _time = "afternoon"
        elif hour >= 17 and hour < 23:
            _time = "evening"
        else:
            _time = "night"
        time.sleep(10)

###############################################################
# Speech Related
            
def speak(phrase):
    global _last_phrase, _voice

    try:
        print(phrase)
        _voice.say(phrase)
    except Exception:
        print("Speak has timed out.")
        pass

    if phrase != "I said":
        _last_phrase = phrase

# To play audio text-to-speech during execution
# def google_speak(my_text):
#     global _internet
#     with io.BytesIO() as f:
#         try:
#             print(my_text)
#             gTTS(text=my_text,
# #                lang='en',slow=False).write_to_fp(f)
#         except Exception as e:
#             print(e)
#             if e != "No text to speak":
#                 _internet = False
#                 print("Cannot use google speak, may have lost internet.")
#             return;
#         _internet = True
#         f.seek(0)
#         pygame.mixer.init()
#         pygame.mixer.music.load(f)
#         pygame.mixer.music.play()
#         while pygame.mixer.music.get_busy():
#             continue

#def speak(my_text):
#    google_speak(my_text)
#    if (_internet == False):
#        local_speak(my_text)

###############################################################
# Miscellaneous

def loadMap():
    global _sdp
    _sdp.wakeup()
    print("Loading map")
    res = _sdp.loadSlamtecMap(_map_filename_str)
    print("Done loading map")
    if (res == 0):
        None
        # speak("Now let me get my bearings.")
        # result = recoverLocalization(_INIT_RECT)
        # if result == False:
        #    speak("I don't appear to be at the map starting location.")
    else:
        speak("Something is wrong. I could not load the map.")
    return res

def saveMap():
    global _sdp
    print("saving map")
    res = _sdp.saveSlamtecMap(b'HCR-MyHouse.stcm')
    if res != 0:
        speak("Something is wrong. I could not save the map.")
    return res
        
def show_picture_mat(mat):
    cv2.imshow("Snapshot", mat)
    cv2.waitKey(10000)
    cv2.destroyWindow("Snapshot")
    
def show_picture(filepath):
    img = cv2.imread(filepath)
    cv2.imshow("Snapshot", img)
    cv2.waitKey(10000)
    cv2.destroyWindow("Snapshot")
    
def take_picture(filename):
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    time.sleep(2.0)  # If you don't wait, the image will be dark
    return_value, image = camera.read()
    playsound("sounds/camera-shutter.wav", block=True)
    cv2.imwrite("pictures_taken/" + filename, image)
    del(camera)  # so that others can use the camera as soon as possible
    return image
    
def statusReport():
    global _person, _mood, _sdp
    
    speak("This is my current status.")
    location, distance, closeEnough = where_am_i()
    if not closeEnough:
        answer = "I'm currently closest to the " + location
        speak(answer)
    else:
        answer = "I'm at the " + location + " location."
        speak(answer)
        answer = "I'm with " + _person
        speak(answer)
    answer = "Battery is at "
    answer = answer + str(_sdp.battery()) + " percent"
    speak(answer)
    answer = "And I'm feeling " + _mood
    speak(answer)
    time.sleep(5)

def handle_response(phrase, check_hot_word = True):
    global _run_flag, _goal, _listen_flag, _last_phrase, _motion_flag
    global _thought, _person, _new_person_flag, _mood, _time
    global _request, _action, _action_flag
    global _eyes_flag, _sdp, _hotword, _privacy_mode

    # convert phrase to lower case for comparison
    phrase = phrase.lower()
    if "stop" in phrase:
        cancelAction()
        speak("Okay.")
        location, distance, closeEnough = where_am_i() 
        if not closeEnough:
            speak("I'm closest to the " + location)
        else:
            ans = "I am now near the " + location + " location."
            speak(ans)
        return HandleResponseResult.Handled
    
    if (phrase == "what's your name" or
        phrase == "what is your name" or
        phrase == "who are you"):
        speak("My name is Orange. Easy to remember, right?")
        return HandleResponseResult.Handled

    # check if the first word is the wake up word, otherwise ignore speech
    if check_hot_word:
        try:
            (firstWord, phrase) = phrase.split(maxsplit=1)
        except:
            return HandleResponseResult.NotHandledNoHotWord
        if (firstWord != _hotword):
            return HandleResponseResult.NotHandledNoHotWord
        
    # some verbal commands are handled inside the listen thread
    
    if (phrase == "resume listening" or
        phrase == "start listening" or
        phrase == "start listening again"):
        _listen_flag = True
        speak("Okay, I'm listening.")
        return HandleResponseResult.Handled
    
    if _listen_flag == False:
        return HandleResponseResult.Handled
    
    if (phrase == "pause listening" or
        phrase == "stop listening"):
        _listen_flag = False
        print("Okay, pause listening.")
        return HandleResponseResult.Handled
    
    if (phrase == "what" or
        phrase == "what did you say" or
        phrase == "please repeat what you just said"):
        speak("I said")
        speak(_last_phrase)
        return HandleResponseResult.Handled
        
    if "on privacy" in phrase:
        _privacy_mode = True
        speak("Ok. Privacy is on.")
        return HandleResponseResult.Handled

    if "off privacy" in phrase:
        _privacy_mode = False
        speak("Ok. Privacy is off.")
        return HandleResponseResult.Handled

    if phrase == "this is evelyn":
        if _person != "Evelyn":
            _new_person_flag = True
        _person = "Evelyn"
        return HandleResponseResult.Handled
    
    if phrase == "this is jim":
        if _person != "Jim":
            _new_person_flag = True
        _person = "Jim"
        return HandleResponseResult.Handled

    if phrase == "goodbye":
        answer = "See you later, " + _person
        speak(answer)
        if _person != "nobody":
            _new_person_flag = True
        _person = "nobody"
        return HandleResponseResult.Handled

    if phrase == "who are you with":
        answer = "I'm with " + _person
        speak(answer)
        return HandleResponseResult.Handled

    if (phrase == "how are you" or
        phrase == "how are you feeling"):
        answer = "I'm feeling " + _mood
        speak(answer)
        return HandleResponseResult.Handled
    
    if phrase == "where are you":
        location, distance, closeEnough = where_am_i()
        if not closeEnough:
            answer = "I'm closest to the " + location
            speak(answer)
            return HandleResponseResult.Handled
        answer = "I'm at the " + location + " location."
        speak(answer)
        return HandleResponseResult.Handled
                
    if phrase == "what time is it":
        day = time.strftime("%A", time.localtime())
        month = time.strftime("%B", time.localtime())
        date = str(int(time.strftime("%d", time.localtime())))
        hour = str(int(time.strftime("%I", time.localtime())))
        minutes = time.strftime("%M", time.localtime())
        ampm = time.strftime("%p", time.localtime())
        if ampm == "AM":
            ampm = "a.m."
        else:
            ampm = "p.m."
        answer = "It is " + day + " " + month + " " + date + " at " + hour + " " + minutes + " " + ampm
        speak(answer)
        answer = "It's " + _time + "."
        speak(answer)
        return HandleResponseResult.Handled
    
    if "status" in phrase:
        statusReport()
        return HandleResponseResult.Handled
                        
    if phrase == "challenge phase one":
        speak("Ok. I'm doing the challenge phase one.")
        _sdp.wakeup()
        time.sleep(6)
        lps = _sdp.getLaserScan()
        longest_dist = 0
        longest_angle = 0
        for i in range(0, lps.size):
            if lps.distance[i] > longest_dist:
                longest_dist = lps.distance[i]
                longest_angle = lps.angle[i]

        longest_dist -= 0.75
        print("other side of room: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
        pose = _sdp.pose()
        xt = pose.x + longest_dist * math.cos(math.radians(pose.yaw) + longest_angle)
        yt = pose.y + longest_dist * math.sin(math.radians(pose.yaw) + longest_angle)
        _locations["custom"] = (xt, yt)
        _locations["origin"] = (pose.x, pose.y)
        _goal_queue.append("origin")
        _goal = "custom"
        return HandleResponseResult.Handled

    if phrase == "go home" or phrase == "go recharge" or phrase == "go to dock":
        cancelAction(interrupt = True)
        _goal = "home"
        return HandleResponseResult.Handled

    class GoDir:
        Forward = 0
        Backward = -180
        Right = -90
        Left = 90
        
    phi = None
    # replace dash if present after gto with space => go <direction> 
    if phrase.startswith("go-"):
        phrase = phrase[:3].replace('-',' ') + phrase[3:]
    if phrase.startswith("go forward"):
        phi = GoDir.Forward
    elif phrase.startswith("go backward"):
        phi = GoDir.Backward
    elif phrase.startswith("go right"):
        phi = GoDir.Right
    elif phrase.startswith("go left"):
        phi = GoDir.Left

    if phi != None:
        # if already in action, ignore this
        if _action_flag:
            return HandleResponseResult.Handled
        unit = None
        dist = None
        phrase_split = phrase.split() # split string into individual words
        split_len = len(phrase_split)
        if split_len < 3:
            return HandleResponseResult.Handled
        n = 2
        # parse optional "at n degrees"
        if split_len >= 7 and (phi == GoDir.Forward or phi == GoDir.Backward) and \
            phrase_split[n] == "at" and phrase_split[n+2] == "degrees":
            try:
                phi += float(w2n.word_to_num(phrase_split[n+1]))
            except:
                try:
                    phi += float(phrase_split[n+1])
                except:
                    return HandleResponseResult.Handled
            n += 3
        # parse distance
        try:
            dist = float(w2n.word_to_num(phrase_split[n]))
        except:
            try:
                dist = float(phrase_split[n])
            except:
                if phrase_split[n] == "to":
                    dist = 2
                elif phrase_split[n] == "for":
                    dist = 4
                else:
                    # this handles when distance is combined with unit. e.g. 3M, 3cm
                    fs = "{:n}{0}"
                    parsed = parse.parse(fs, phrase_split[n])
                    if parsed != None:
                        dist = parsed[0]
                        unit = parsed[1]

        if dist == None:
            return HandleResponseResult.NotHandledUnknown

        # parse unit of distance
        if split_len > n+1:
            unit = phrase_split[n+1]

        if unit == None or unit.startswith("m"):
            None
        elif unit == "cm" or unit == "centimeters":
            dist /= 100
        elif unit == "in" or unit == "inches":
            dist *= 0.0254
        elif unit == "yard" or unit == "yards":
            dist /= 1.094
        else:
            print("unknown unit")
            return HandleResponseResult.NotHandledUnknown
        
        #if unit not mentioned assume meters
        pose = _sdp.pose()
        xt = pose.x + dist * math.cos(math.radians(pose.yaw + phi))
        yt = pose.y + dist * math.sin(math.radians(pose.yaw + phi))
        _locations["custom"] = (xt, yt)
        _goal = "custom"
        return HandleResponseResult.Handled
        
    if phrase == "go to sleep":
        speak("Okay. I'm going to take a nap.")
        os.system("rundll32.exe powrprof.dll,SetSuspendState 0,1,0")
        return HandleResponseResult.Handled

    if "go to" in phrase:
        words = phrase.split()
        try:
            words.remove("the")
        except:
            None
        if len(words) > 2:
            cancelAction(interrupt = True)
            _goal = " ".join(words[2:]).lower()
        return HandleResponseResult.Handled
            
    if phrase == "recover localization":
        speak("I will search the whole map to locate myself.")
        result = recoverLocalization(_HOUSE_RECT)
        if result == False:
            speak("I could not confirm my location.")

    if phrase == "list your threads":
        print()
        pretty_print_threads()
        print()
        return HandleResponseResult.Handled
    
    # if phrase == "open your eyes":
    #     if _eyes_flag == True:
    #         return HandleResponseResult.Handled
    #     _eyes_flag = True
    #     Thread(target = eyes, name = "Display eyes").start()
    #     return HandleResponseResult.Handled
    
    if phrase == "close your eyes":
        _eyes_flag = False
        return HandleResponseResult.Handled
    
    if (phrase == "initiate shut down" or
        phrase == "initiate shutdown" or
        phrase == "begin shut down" or
        phrase == "begin shutdown"):
        _eyes_flag = False
        speak("Okay, I'm shutting down.")
        #playsound("C:/Users/bjwei/wav/R2D2d.wav")
        # List all currently running threads
        print("\nClosing these active threads:")
        pretty_print_threads()
        # Have them terminate and close
        _run_flag = False
        return HandleResponseResult.Handled        

    if phrase == "shutdown system":
        speak("Okay, I'm shutting down the system.")
        _run_flag = False
        os.system("shutdown /s /t 10")
        return HandleResponseResult.Handled
            
    if "battery" in phrase or "voltage" in phrase:
        ans = "My battery is currently at "
        ans = ans + str(_sdp.battery()) + " percent"
        speak(ans)
        return HandleResponseResult.Handled
        
    if "motion" in phrase:
        if _motion_flag:
            speak("I am detecting motion.")
        else:
            speak("I'm not detecting any motion.")
        return HandleResponseResult.Handled
 
    if "load map" in phrase:
        speak("Ok. I will load my map.")
        loadMap()
        return HandleResponseResult.Handled
    
    if "save map" in phrase:
        speak("Ok. I will save my map.")
        saveMap()
        return HandleResponseResult.Handled
    
    if "clear map" in phrase:
        speak("Ok. I will clear my map.")
        _sdp.clearSlamtecMap()
        return HandleResponseResult.Handled
    
    if "map updating" in phrase:
        if "enable" in phrase:
            enable = True
            speak("Ok. I will enable map updating.")
        elif "disable" in phrase:
            enable = False
            speak("Ok. I will disable map updating.")
        else:
            return HandleResponseResult.Handled                
        _sdp.setUpdate(enable)
        return HandleResponseResult.Handled
    
    if "take a picture" in phrase:
        speak("Ok. Say Cheeze...")
        pic_filename = "capture_" + time.ctime().replace(' ', '-', -1).replace(":","-",-1) +".jpg"
        mat = take_picture(pic_filename)
        time.sleep(0.5)
        speak("Ok. Here is the picture I took.")
        show_picture_mat(mat)
        return HandleResponseResult.Handled

    if "set speed" in phrase:
        if "low" in phrase:
            speed = 1
        elif "medium" in phrase:
            speed = 2
        elif "high" in phrase:
            speed = 3
        else:
            return HandleResponseResult.Handled
        speak("Ok. I'm setting the speed.")
        if speed != _sdp.setSpeed(speed):
            speak("Sorry, I could not change my speed this time.")
        return HandleResponseResult.Handled
    
    if "you see" in phrase:
        results = detect.detect_objects(top_count=3)
        print(results)
        res_count = len(results) 
        if res_count > 0 and results[0].percent >= 40:
            reply_str = "I see a " + results[0].label
            if res_count == 2 and results[1].percent >= 40:
                reply_str += " and a " + results[1].label
            elif res_count > 2:
                for i in range(1, res_count - 1):
                    if results[i].percent >= 40:
                        reply_str += ", a " +results[i].label
                if results[res_count - 1].percent >= 40:
                    reply_str += " and a " + results[res_count - 1].label
        else:
            reply_str = "I don't see anything I recognize."
        speak(reply_str)
        return HandleResponseResult.Handled
    
    if "identify this" in phrase:
        model = classify.ModelType.General
        if "bird" in phrase:
            model = classify.ModelType.Birds
        elif "insect" in phrase:
            model = classify.ModelType.Insects
        elif "plant" in phrase:
            model = classify.ModelType.Plants
        results = classify.classify(model)
        print(results)
        if len(results) > 0 and results[0].percent > 40 and "background" not in results[0]:
            speak("it looks like a " + results[0].label)
        else:
            speak("Sorry, I do not know what it is.")
        return HandleResponseResult.Handled
    
    if "clear windows" in phrase:
        cv2.destroyAllWindows()
        return HandleResponseResult.Handled
    
    deg = 0
    temp = phrase # special case requiring parsing
    temp = temp.replace('\xb0', ' degrees') # convert '90°' to '90 degrees'
    phrase_split = temp.split() # split string into individual words
    split_len = len(phrase_split)
    if split_len >= 2 and ((phrase_split[0] == "turn" and 
                           phrase_split[1] != "on" and 
                           phrase_split[1] != "off") or phrase_split[0] == "rotate"):
        try:
            deg = int(w2n.word_to_num(phrase_split[1]))
            if split_len >= 4 and phrase_split[3] == "clockwise":
                deg = -deg
        except:
            if phrase_split[1] == "around":
                deg = 180
            else:
                None
        turn(deg)
        return HandleResponseResult.Handled
    
    return HandleResponseResult.NotHandledUnknown # not handled

###############################################################
# Speech recognition using Google over internet connection
def listen():
    import speech_recognition as sr
    global _run_flag, _goal
    global _internet, _hotword, _privacy_mode
    
    ###########################################################
    # Text input to Google Assistant for web based queries
    ASSISTANT_API_ENDPOINT = 'embeddedassistant.googleapis.com'
    DEFAULT_GRPC_DEADLINE = 60 * 3 + 5
    PLAYING = embedded_assistant_pb2.ScreenOutConfig.PLAYING

    api_endpoint = ASSISTANT_API_ENDPOINT
    credentials = os.path.join(click.get_app_dir('google-oauthlib-tool'), 'credentials.json')
    device_model_id = "orbital-clarity-197305-marvin-hcmekv"
    device_id = "orbital-clarity-197305"
    lang = 'en-US'
    display = False
    verbose = False
    grpc_deadline = DEFAULT_GRPC_DEADLINE
    
    class TextAssistant(object):
        """Text Assistant that supports text based conversations.
        Args:
          language_code: language for the conversation.
          device_model_id: identifier of the device model.
          device_id: identifier of the registered device instance.
          display: enable visual display of assistant response.
          channel: authorized gRPC channel for connection to the
            Google Assistant API.
          deadline_sec: gRPC deadline in seconds for Google Assistant API call.
        """
        
        def __init__(self, language_code, device_model_id, device_id,
                     display, channel, deadline_sec):
            self.language_code = language_code
            self.device_model_id = device_model_id
            self.device_id = device_id
            self.conversation_state = None
            # Force reset of first conversation.
            self.is_new_conversation = True
            self.display = display
            self.assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(channel)
            self.deadline = deadline_sec
        
        def __enter__(self):
            return self
        
        def __exit__(self, etype, e, traceback):
            if e:
                return False
        
        def assist(self, text_query):
            """Send a text request to the Assistant and playback the response.
            """
            def iter_assist_requests():
                config = embedded_assistant_pb2.AssistConfig(
                    audio_out_config=embedded_assistant_pb2.AudioOutConfig(
                        encoding='LINEAR16',
                        sample_rate_hertz=16000,
                        volume_percentage=0,
                    ),
                    dialog_state_in=embedded_assistant_pb2.DialogStateIn(
                        language_code=self.language_code,
                        conversation_state=self.conversation_state,
                        is_new_conversation=self.is_new_conversation,
                    ),
                    device_config=embedded_assistant_pb2.DeviceConfig(
                        device_id=self.device_id,
                        device_model_id=self.device_model_id,
                    ),
                    text_query=text_query,
                )
                # Continue current conversation with later requests.
                self.is_new_conversation = False
                if self.display:
                    config.screen_out_config.screen_mode = PLAYING
                req = embedded_assistant_pb2.AssistRequest(config=config)
                #assistant_helpers.log_assist_request_without_audio(req)
                yield req

            text_response = None
            html_response = None
            
            try:
                for resp in self.assistant.Assist(iter_assist_requests(),
                                                  self.deadline):
                    #assistant_helpers.log_assist_response_without_audio(resp)
                    if resp.screen_out.data:
                        html_response = resp.screen_out.data
                    if resp.dialog_state_out.conversation_state:
                        conversation_state = resp.dialog_state_out.conversation_state
                        self.conversation_state = conversation_state
                    if resp.dialog_state_out.supplemental_display_text:
                        text_response = resp.dialog_state_out.supplemental_display_text
            except Exception as e:
                print("got error from assistant: "+ str(e))
            return text_response, html_response
    
    
    logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO)
    
    # Load OAuth 2.0 credentials.
    try:
        with open(credentials, 'r') as f:
            credentials = google.oauth2.credentials.Credentials(token=None, **json.load(f))
            http_request = google.auth.transport.requests.Request()
            credentials.refresh(http_request)
    except Exception as e:
        logging.error('Error loading credentials: %s', e)
        logging.error('Run google-oauthlib-tool to initialize '
                        'new OAuth 2.0 credentials.')
    
    # Create an authorized gRPC channel.
    grpc_channel = google.auth.transport.grpc.secure_authorized_channel(credentials, http_request, api_endpoint)
    speak("I'm connected to Google Assistant.")
    #logging.info('Connecting to %s', api_endpoint)

    # create a recognizer object
    r = sr.Recognizer()
        
    speak("Hello, My name is Orange. Pleased to be at your service.")
        
    def local_hotword_recog_cb(phrase, listener, hotword):
        global _pause_listening
        print("I heard: %s" % phrase)
        if "stop" in phrase or "what's your name" in phrase or "what is your name" in phrase \
            or "who are you" in phrase:
            handle_response(phrase)
            return
        if hotword in phrase:
            _pause_listening = True
            listener.stop_listening()
            listenFromGoogleSpeechRecog()
            handled_result = handle_response(phrase, False)
        if handled_result == HandleResponseResult.NotHandledUnknown:            
            sendToGoogleAssistant(phrase)
        _pause_listening = False

    def local_speech_recog_cb(phrase, listener, hotword):
        global _privacy_mode
        print("I heard: %s" % phrase)
        handled_result = handle_response(phrase)
        if _privacy_mode == False:
            listener.stop_listening()
            return
        if handled_result == HandleResponseResult.NotHandledUnknown:
            print("I am not sure how to help with that.")
            speak("I am not sure how to help with that.")
        elif handled_result == HandleResponseResult.NotHandledNoHotWord:
            print("No hot word, ignoring.")

    while _run_flag:
        use_local_speech = (not _internet) or _privacy_mode

        if use_local_speech:
            print("local listener")
            # if no internet access or privacy mode is active, use WSR / SAPI to recognize a command subset
            local_listener = winspeech.listen_for(None, "speech.xml", "RobotCommands", 
            lambda phrase, listener, hotword=_hotword: local_speech_recog_cb(phrase, listener, hotword))
        else: # use winspeech to detect hotword or stop and then invoke google cloud speech
            print("detecting hotword")
            local_listener = winspeech.listen_for(None, "hotword.xml", "RobotHotword", 
            lambda phrase, listener, hotword=_hotword: local_hotword_recog_cb(phrase, listener, hotword))

        while local_listener.is_listening() or _pause_listening:
            time.sleep(0.1)


def listenFromGoogleSpeechRecog():
    # obtain audio from the microphone
    try:
        with sr.Microphone() as source:
            # adjust microphone for ambient noise:
            # first, turn off dynamic thresholding,
            # eg: r.adjust_for_ambient_noise(source)
            # then, set a volume threshold at 500 where
            # 0 = it hears everything. 4000 = it hears nothing.
            r.dynamic_energy_threshold = False
            r.energy_threshold = 2000 if _goal != "" else 500 
            print("Say something!")
            audio = r.listen(source, phrase_time_limit= 2 if _goal != "" else None)
    except:
        return
    
    # recognize speech using Google Speech Recognition
    try:
        phrase = r.recognize_google(audio)
        _internet = True
        print("I heard: " + phrase)
    except sr.UnknownValueError:
        phrase = ""
        _internet = True
        print("What?")
    except sr.RequestError:
        phrase = ""
        _internet = False
        print("I lost my internet connection.")
    except:
        print("Unknown speech recognition error.")
    

def sendToGoogleAssistant(phrase):
    global _internet
    if _internet:
        print("Don't know about that, sending question to google assistant")
        with TextAssistant(lang, device_model_id, device_id, display,
                grpc_channel, grpc_deadline) as assistant:
            response_text, response_html = assistant.assist(text_query = phrase)
            if response_text:
                speak(response_text)
    else: 
        print("I am not sure how to help with that.")
        speak("I am not sure how to help with that.")

def recoverLocalization(rect):
    result = _sdp.recoverLocalization(rect["left"], 
                                      rect["bottom"],
                                      rect["width"],
                                      rect["height"])
    print("Recovering localization result = ", result)
    if result == ActionStatus.Finished:
        location, distance, closeEnough = where_am_i()
        if closeEnough:
            speak("I appear to be at the " + location + " location.")
        else:
            speak("I appear to be near the "+ location + " location.")
        return True
    return False

def initialize_speech():
    global _voice
    _voice = tts.sapi.Sapi()
    #_voice.set_voice("Andy")
    _voice.voice.Volume = 80
    _voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds

################################################################   
# This is where data gets initialized from information stored on disk
# and threads get started
def initialize_robot():
    global _moods, _people, _ser6, _internet, _eyes_flag
    
    # speak("Initializing data from memory.")
    
    # with open('moods_data_file') as json_file:  
    #     _moods = json.load(json_file)

    # with open('people_data_file') as json_file:  
    #     _people = json.load(json_file)
        
    # _ser6 = serial.Serial('COM6', 9600, timeout=1.0)
    
    _internet = True
    
    
    Thread(target = listen, name = "Listen").start()
    
    Thread(target = time_update, name = "Time").start()
        
#    Thread(target = monitor_motion, name = "People Motion Monitor").start()
    
#    Thread(target = behaviors, name = "Behaviors").start()
    
    Thread(target = handleGotoLocation, name = "Handle Goto Location").start()
#    Thread(target = actions, name = "Actions").start()
    
#    Thread(target = robotMoving, name = "Is Robot Moving").start()
    
#    Thread(target = megaCOM7, name = "megaCOM7").start()
    
    _eyes_flag = True # this is needed to start the display and to keep it open
#    Thread(target = eyes, name = "Display eyes").start()
    
    #Thread(target = display, name = "Display").start()
    #Thread(target = batteryMonitor, name = "Battery monitor").start()
    
################################################################
# This is where data gets saved to disk
# and by setting _run_flag to False, threads are told to terminate
def shutdown_robot():
    global _run_flag, _moods, _people, _ser6, _sdp
    
    # # Save "memories" to disk    
    # with open('moods_data_file', 'w') as outfile:  
    #     json.dump(_moods, outfile)
    
    # with open('people_data_file', 'w') as outfile:  
    #     json.dump(_people, outfile)
    
    _run_flag = False
    
    # TBD join threads
    
    # Close all open threads
    #while (threading.active_count() > 1):
    #    print("\nClosing... ")
    #    pretty_print_threads()
    #    time.sleep(1)

    # Close the serial port
#    _ser6.close()
   
#    playsound("C:/Users/bjwei/wav/R2D2.wav")
    _sdp.disconnect()
    del _sdp
    print("\nDone!")
   
################################################################
# This initializes and runs the robot
def robot():
    global _run_flag, _ser6
    
    initialize_robot()
    
    while _run_flag:
        time.sleep(1)
    
    shutdown_robot()

def connectToSdp():
    errStr = create_string_buffer(255)
    res = _sdp.connectSlamtec(_sdp_ip_address, _sdp_port, errStr.raw, 255)
    if res == 1 :
        print("Could not connect to SlamTec, time out.")#, errStr.value)
    elif res == 2:
        print("Could not connect to SlamTec, connection fail.")#, errStr.value)
    return res

def init_local_speech_rec():
    # Start an in-process edge recognizer using SAPI.
    winspeech.initialize_recognizer(winspeech.INPROC_RECOGNIZER)
    
def run():
    global _sdp
    # Start 32 bit bridge server
    _sdp = MyClient()

    if _execute:
        init_local_speech_rec()
        initialize_speech()
        speak("I'm starting up.")

        res = connectToSdp()

        if (res == 0):
            print("I'm now connected to Slamtec. Now loading the map.")
            loadMap()
            # set to update map by default.
            _sdp.setUpdate(True)
            None
        else:
            speak("Something is wrong. I could not connect to Slamtec.")
        robot()
    else:
        print("Program is in DEBUG mode.")
    
if __name__ == '__main__':
    run()
