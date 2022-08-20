# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 17:39:44 2020

@author: LattePanda
"""
import math
import random
from threading import Lock
import sdp_comm
import subprocess

# Constants
_show_rgb_window = False
_show_depth_window = False
_default_map_name = 'my house'
_hotword = "orange"
_google_mode = False
_execute = True # False for debugging, must be True to run as: >python main.py
_run_flag = True # setting this to false kills all threads for shut down
_eyes_flag = True # should eyes be displayed or not
_moods = {"happy":50, "bored":20, "hungry":10}
_HOUSE_RECT = {"left":-0.225,"bottom":-5.757, "width":12.962, "height":7.6}
_OFFICE_RECT = {"left":0.405,"bottom":-0.128, "width":3.6, "height":1}
_KITCHEN_RECT = {"left":10.3,"bottom":-4.9, "width":1.8, "height":1.6}
_DINING_AREA_RECT =  {"left":3.91,"bottom":-5.03, "width":2.61, "height":2.18}
_INIT_RECT =  {"left":-0.5,"bottom":-0.5, "width":1.0, "height":1.0}
_STARTUP_ROOM = _OFFICE_RECT
_DELIVERY_RESPONSES = [
    "You know, I'm only doing this until I get discovered... I, want to direct... For now, it's on to the next delivery.",
    "Since you asked... This, is just my day job... At night, I'm shooting an indie film... Oh well, deliveries are fun too.",
    "Waiting tables of people guzzling down their drinks is a means to an end for me... Filmmaking is, my real passion. Later."]

_LOCATION_RECTS = { "kitchen": _KITCHEN_RECT, "office": _OFFICE_RECT, "dining area": _DINING_AREA_RECT}
_dai_fps = 20 # depthai approx. FPS (adjust lower to conserve CPU usage)
_dai_fps_recip = 1.0 / _dai_fps

# Globals
_mood = "happy"
_person = "jim"
_slamtec_on = False
_goal = ""
_sub_goal = ""
_goal_queue = []
_time = "morning"
_last_phrase = "nothing"
_listen_flag = True
_action_flag = False # True means some action is in progress
_interrupt_action = False # True when interrupting a previously started action
_internet = True # True when connected to the internet
_use_internet = True # If False don't use internet
_call_out_objects = False # call out objects along route
_user_set_speed = 2
_error_last_goto = False
_deliveree = ""
_package = ""
_spoken_package = ""
_response_num = random.randint(0,2)
_all_loaded = False
_facial_recog = None
_facial_recog_thread = None
_my_depthai_thread = None
_listen_thread = None
_eyes_thread = None
_handle_resp_thread = None
_handling_resp = False
_handling_resp_lock = Lock()
_sendToGoogleAssistantFn = None
_restart_flag = False
_set_energy_threshold = None
_get_energy_threshold = None
_last_speech_heard = ""
_locations = {}
_mdai = None
_move_oak_d = None
_mic_array = None
_pixel_ring = None
_starting_up = True

import parse
import tts.sapi
import tts.flags
import time
from ctypes import *
import os
import sys
import threading
from threading import Thread
from playsound import playsound
from word2number import w2n
import io
#import pygame
#from gtts import gTTS
from my_sdp_client import MyClient
from my_sdp_server import *
import cv2
import ai_vision.detect as detect
import ai_vision.classify as classify
#import winspeech
from enum import Enum
import move_oak_d
import my_depthai
import eyes
import facial_recognize as fr
import pickle
import speech_recognition as sr
import socket
from mic_array_tuning import Tuning
import usb.core
import usb.util
import usb_pixel_ring_v2 as pixel_ring

class HandleResponseResult(Enum):
    """Enumerated type for result of handling response of command"""
    def __init__(self, number):
        self._as_parameter__ = number

    # The response was not handled because another is in progress        
    NotHandledBusy = -2
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

def move_imm(sdp, vel):
    if vel > 0:
        sdp.forward()
    elif vel < 0:
        sdp.back()
#    else:
#        sdp.cancelMoveAction()

def turn_imm(sdp, vel):
    if vel > 0:
        sdp.left()
    elif vel < 0:
        sdp.right()
#    else:
#        sdp.cancelMoveAction()

################################################################
def turn(degrees):
    global _sdp, _action_flag
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

################################################################
# function to calculate the distance between 2 points (XA YA) and (XB YB)
def distance_A_to_B(XA, YA, XB, YB):
    dist = math.sqrt((XB - XA)**2 + (YB - YA)**2)
    return dist


#x == _sdp.getX() and
#y == _sdp.getY()
################################################################

def nearest_location(x, y):    
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
    print("I am at location = ", location, " distance = ", distance)
    if (distance <= 1.0):
        closeEnough = True
    else:
        closeEnough = False
    return location, distance, closeEnough

def is_close_to(loc, max_dist=1.0):
    if _locations.get(loc) is None:
        return -1
        
    try:
        pose = _sdp.pose()
    except:
        return -1
    dist = distance_A_to_B(pose.x, pose.y, _locations[loc][0], _locations[loc][1])
    return dist <= max_dist

################################################################
# This cancels an ongoing action - which may be a goto or something else.


def cancelAction(interrupt = False):
    global _sdp, _action_flag, _interrupt_action
    if interrupt:
        _interrupt_action = True
    for attempt in range(3):
        try:
            _sdp.cancelMoveAction()
            _action_flag = False
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

_reported_35 = False
_reported_25 = False
_reported_18 = False
_reported_15 = False

def batteryMonitor():
    global _sdp, _run_flag, _person, _goal, _reported_35, _reported_25, _reported_18, _reported_15
    try:
        batteryPercent = _sdp.battery()
        person = _person if _person != "nobody" else "hello anyone"
        if batteryPercent <= 15:
            if not _reported_15:
                _reported_15 = True
                speak(person + ", my battery is exhausted, and I am shutting down now.") 
                cancelAction()
                _run_flag = False
                os.system("shutdown /s /t 30")
        elif batteryPercent <= 18:
            if not _reported_18:
                _reported_18 = True
                speak(person + ", I need to recharge my battery. I am going to the recharge station.")
                _goal = "recharge"
        elif batteryPercent <= 25:
            if not _reported_25:
                _reported_25 = True
                speak(person + ", my battery is getting low. I'll have to charge up soon.")
                setPixelPaletteRed()
        elif batteryPercent <= 35:
            if not _reported_35:
                _reported_35 = True
                setPixelPaletteYellow()
        else:
            setPixelPaletteDefault()
    except:
        None
    
################################################################
# This is where goto actions are initiated and get carried out.
# If the robot is in the process of going to a  location, but
# another goto action request is receved, the second request will
# replace the earlier request

def handleGotoLocation():
    global _run_flag, _goal, _action_flag, _sdp, _interrupt_action
    global _deliveree, _package, _sub_goal, _call_out_objects
    global _error_last_goto, _response_num, _locations

    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)

    sub_goal_cleanup = None
    while _run_flag:
        if _goal == "" or _action_flag:
            # no goal or some action is currently in progress, so sleep.
            time.sleep(0.5)
            continue
        if _sub_goal != "":
            sub_goal_cleanup = None
        _error_last_goto = False
        print("I'm free and A new goal arrived: ", _goal)
        if _goal == "recharge":
            speak("I'm going to the recharge station")
            coords = _locations.get(_goal)
            sdp.home()
        elif _goal == "deliver" and _locations.get(_goal) is None:
            mult_people = "," in _deliveree
            # For multiple person delivery, do not bother looking for a person
            if mult_people:
                pose = sdp.pose()
                _locations[_goal] = (pose.x, pose.y, math.radians(pose.yaw)) 
                continue
            else: # need to find a single person in room for delivery
                if not setDeliverToPersonAsGoal():
                    speak("Sorry, I could not find "+ _deliveree)
                    _goal = ""
                    _deliveree = ""
                    _goal_queue.clear()
                continue
        else: # expect a goal in the list of locations 
            coords = _locations.get(_goal)
            if coords is None:
                speak("Sorry, I don't know how to get there.")
                print("unknown location")
                _goal = ""
                if len(_goal_queue) > 0:
                    _goal = _goal_queue.pop(0)
                continue
            if _goal != "custom" and _goal != "deliver" and _goal != "person":
                if is_close_to(_goal, 0.5): 
                    speak("I'm already at the " + _goal)
                    _goal = ""
                    if len(_goal_queue) > 0:
                        _goal = _goal_queue.pop(0)
                    continue
                if _goal != sub_goal_cleanup and len(_goal_queue) == 0 or len(_goal_queue) > 0 and _goal_queue[0] != "deliver":
                    speak("I'm going to the " + _goal)
            elif _goal == "deliver":
                speak("hello " + _deliveree)
            else:
                speak("OK.")
            _action_flag = True
            if len(coords) == 3:
                sdp.moveToFloatWithYaw(coords[0], coords[1], coords[2])
            else:
                sdp.moveToFloat(coords[0], coords[1])

        _interrupt_action = False
        sleepTime = 0.5 if (_sub_goal == "" or _call_out_objects) else 0
        if _sub_goal != "":
            checkPersons = _sub_goal == 'person'
            if checkPersons:
                aim_oakd(pitch=70) # aim up to see people better                
                eyes.setTargetPitchYaw(-70, 0)
            else:
                aim_oakd(pitch=110) # aim down towards floor for objects
                eyes.setTargetPitchYaw(70, 0)
        else:
            checkPersons = _goal != "deliver" # avoid saying there's a person in the way going to a person
        checkObjects = not checkPersons        
        sub_goal_just_found = False
        idx = 0
        objDict = {}

        def gotLock():
            print("got a lock on ", _sub_goal)
            _goal_queue.append(_sub_goal)
            speak("I see a " + _sub_goal)
            _move_oak_d.yawHome()
            eyes.setHome()
            sdp.setSpeed(_user_set_speed) #restore speed after finding obj

        if _sub_goal != "":
            if setFoundObjAsGoal(_sub_goal):
                gotLock()
                sub_goal_just_found = True
                sub_goal_cleanup = _sub_goal
            else:
                _move_oak_d.startSweepingBackAndForth()
        while(_run_flag and _interrupt_action == False and not sub_goal_just_found):
#            try:
            if _sub_goal == "":
                if  _call_out_objects:
                    # check if object is persistant over 2 seconds ~32 checks and max dist 2 meters
                    idx = checkForObjects(_possObjObstacles, objDict, numChecks=8, maxDist=3.0, needCentered=True, checkPersons=checkPersons, idx=idx)
                    if len(objDict) > 0 and len(objDict[next(iter(objDict))]) == 32: # only start checking after 2 second buffer is full
                        objDict[next(iter(objDict))]
                        persistObjs = computePersistance(objDict)
                        obj = next(iter(persistObjs.items()))
                        if obj[1] > 0.5:
                            print("there's a", obj[0], "in my way seen", obj[1] * 100, "% of the time in the last 2 seconds.")
                            speak("There's a " + obj[0] + " in my way. I will plan a way around it.")
                            del objDict[obj[0]] # we've reported this object, so delete its history from the buffer
            else: # _sub_goal != ""
                objDict = {}
                checkForObjects([_sub_goal], objDict, 9, checkPersons=checkPersons, checkObjects=checkObjects, maxValueLen=9)
                objDict = computePersistance(objDict)

                if len(objDict) > 0 and objDict[_sub_goal] > 0.05:
                    print("spotted", _sub_goal, ", stopping to get a look")
                    sdp.cancelMoveAction()
                    _move_oak_d.stopSweepingBackAndForth()
                    time.sleep(2)
                    if setFoundObjAsGoal(_sub_goal, cam_yaw=_move_oak_d.getYaw()):
                        gotLock()
                        sub_goal_just_found = True
                        sub_goal_cleanup = _sub_goal
                        break
                    else:
                        print("saw", _sub_goal, "but lost track of it")
                        #speak("I thought I saw a " + _sub_goal + ". I'll look again.")
                        _move_oak_d.startSweepingBackAndForth(1)
                        while _move_oak_d.isSweeping():
                            if setFoundObjAsGoal(_sub_goal, cam_yaw=_move_oak_d.getYaw()):
                                gotLock()
                                sub_goal_just_found = True
                                sub_goal_cleanup = _sub_goal
                                break
                        if sub_goal_just_found:
                            break    
                        #speak("I'll keep going.")
                        sdp.moveToFloat(coords[0], coords[1])

            maStatus = getMoveActionStatus()
            if maStatus == ActionStatus.Stopped or \
                maStatus == ActionStatus.Error or \
                maStatus == ActionStatus.Finished:
                break
#            except:
#                break
        time.sleep(sleepTime)
        
        if _interrupt_action == True:
            _interrupt_action = False
            _move_oak_d.stopSweepingBackAndForth()
            _move_oak_d.allHome()
            eyes.setHome()
            _goal_queue.clear()

        if not sub_goal_just_found:
            # reaching this point, the robot first moved, then stopped - so check where it is now
            reached_goal = is_close_to(_goal)

            # and now check to see if it reached the goal
            if (_goal == "deliver" or reached_goal):
                if _goal == "deliver":
                    article = "some" if _package.endswith('s') else "a"
                    speak(_deliveree + ", I have " + article + " " + _package + " for you.")
                    taken = waitForObjectToBeTaken(_package)
                    if taken:
                        speak("Great, and you're welcome. ")
                        speak(_DELIVERY_RESPONSES[_response_num])
                        _response_num = (_response_num + 1) % len(_DELIVERY_RESPONSES)
                    else:
                        speak("Sorry, don't you want the " + _package + "?")
                    _deliveree = None
                elif _goal == sub_goal_cleanup:
                    speak("I found the " + _goal)
                    _move_oak_d.allHome()
                elif _goal != "person":
                    speak("I've arrived.")
            else:
                _error_last_goto = True
                if _goal == "deliver":
                    speak("Sorry, I could not make my delivery")
                elif _goal != "custom":
                    speak("Sorry, I didn't make it to the " + _goal)
                else:
                    speak("Sorry, I didn't make it to where you wanted.")
            if _sub_goal != "":
                speak("and I never found a "+ _sub_goal)
                sdp.setSpeed(_user_set_speed) #restore speed after finding obj

                
        # finally clear temp goals and _action_flags
        if _goal == "custom":
            del _locations["custom"]
        elif _goal == "deliver":
            del _locations["deliver"]
            _deliveree = ""
        elif _goal == sub_goal_cleanup:
            del _locations[_goal]

        _action_flag = False # you've arrived somewhere, so no further action
        _goal = ""
        _sub_goal = ""
        _move_oak_d.stopSweepingBackAndForth()
        if len(_goal_queue) > 0:
            _goal = _goal_queue.pop(0)
        else:
            _move_oak_d.allHome()
            eyes.setHome()

        time.sleep(0.5)
    sdp.disconnect()
    sdp.shutdown_server32(kill_timeout=1)
    sdp = None
        
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

def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value):
    global _last_phrase, _voice

    try:
        print(phrase)
        setPixelRingSpeak()
        _voice.say(phrase, flag)
        setPixelRingTrace()
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

def loadMap(filename):
    global _sdp
    _sdp.wakeup()
    print("Loading map and its locations")
    res = _sdp.loadSlamtecMap(str.encode(filename) + b'.stcm')
    if (res == 0):
        # set update to false because we don't want to change the map when doing a demo with people standing around messing up the map!
        _sdp.setUpdate(False)
        speak("Map and locations are loaded. Mapping is on.")
        # speak("Now let me get my bearings.")
        # result = recoverLocalization(_INIT_RECT)
        # if result == False:
        #    speak("I don't appear to be at the map starting location.")
    else:
        speak("Something is wrong. I could not load the map.")
    print("Done loading map")
    load_locations(filename)
    return res

def saveMap(filename):
    global _sdp
    print("saving map and its locations")
    res = _sdp.saveSlamtecMap(str.encode(filename) + b'.stcm')
    if res != 0:
        speak("Something is wrong. I could not save the map.")
    save_locations(filename)
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
        
def init_camera():
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    time.sleep(1.0)  # If you don't wait, the image will be dark
    camera.read()
    
def take_picture(filename):
    camera_port = 0
    camera = cv2.VideoCapture(camera_port)
    time.sleep(1.0)  # If you don't wait, the image will be dark
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
        
# rotate 360 and stop if a person is spotted
def searchForPerson(sdp, clockwise=True):
    global _action_flag, _interrupt_action

    aim_oakd(pitch=70) # aim up to see people better
    eyes.setTargetPitchYaw(-70, 0)

    ps = []
    # First see if person is already in view and if so return
    found, ps = checkForPerson()
    if found:
        return ps
    
    # if no person in view, then slowly rotate 360 degrees and check every so often
    _action_flag = True
    
    oldyaw = sdp.pose().yaw + 360
    yaw = oldyaw
    sweep = 0
    recheck_person = False
    while (sweep < 380 and not _interrupt_action):
        sdp.rotate(0.1 if clockwise else -0.1)
        found, ps = checkForPerson()
        if found:
            recheck_person = True
            break
        yaw = sdp.pose().yaw + 360
        covered = abs(yaw - oldyaw)
        if covered > 180:
            covered = 360 - covered
        sweep += covered
        #print("yaw = ", yaw, " covered = ", covered, " sweep = ", sweep)

        oldyaw = yaw
        time.sleep(0.05)

    if recheck_person:
        print("rechecking person")
        sdp.cancelMoveAction()
        time.sleep(0.3)
        for j in range(1,5):
            for i in range(1,5):
                found, ps = checkForPerson()
                if found:
                    break
            if not found:
                # go back other way
                deg = -5 if j % 2 == 0 else 5
                sdp.rotate(math.radians(deg))
                sdp.waitUntilMoveActionDone()
            else:
                break
            
    _action_flag = False
    return ps

def checkForObject(obj):
    ps = None
    p = None
    for i in range(0,18):
        try:
            if obj == "person":
                ps = _mdai.getPersonDetections()
            else:
                ps = _mdai.getObjectDetections()
            if len(ps) > 0:
                for a in ps:
                    if obj == a.label:
                        p = a
                # If bbox ctr of detection is away from edge then stop
                if p is not None and p.bboxCtr[0] >= 0.0 and p.bboxCtr[0] <= 1:
                    print(obj, " at bbox ctr: ",p.bboxCtr[0], ", ", p.bboxCtr[1])
                    return True, p
                    break
        except:
            None
            time.sleep(_dai_fps_recip)
        time.sleep(_dai_fps_recip)
    return False, None

def checkForPerson():
    ps = None
    p = None
    try:
        ps = _mdai.getPersonDetections()
        if len(ps) > 0:
            p = ps[0]
            # If bbox ctr of detection is away from edge then stop
            if p.bboxCtr[0] >= 0 and p.bboxCtr[0] <= 1:
                print("Person at bbox ctr: ",p.bboxCtr[0], ", ", p.bboxCtr[1])
                return True, ps
    except:
        None
    return False, ps

def setLocationOfObj(sdp, obj, p, cam_yaw=0):
    p.z -= 1 # come up to the object within certain distance
    pose = sdp.pose()
    xt = pose.x + p.z * math.cos(math.radians(pose.yaw + cam_yaw + p.theta))
    yt = pose.y + p.z * math.sin(math.radians(pose.yaw + cam_yaw + p.theta))
    print("set location of ", obj, " at distance ", p.z, " meters at ", cam_yaw + p.theta, "degrees")
    _locations[obj] = (xt, yt, math.radians(pose.yaw + cam_yaw + p.theta))

def setFoundObjAsGoal(obj, cam_yaw=0):
    global _sdp, _interrupt_action
    found, p = checkForObject(obj)
    if found:
        setLocationOfObj(_sdp, obj, p, cam_yaw)
        return True
    return False

def findObjAndSetGoal(obj, goal, cam_yaw=0):
    global _sdp, _interrupt_action
    found, p = checkForObject(obj)
    if found:
        setLocationOfObj(_sdp, goal, p, cam_yaw)
        return True
    return False

def setDeliverToPersonAsGoal():
    global _sdp, _interrupt_action, _goal

    if findObjAndSetGoal("person", "deliver"):
        print("immediately got a lock on the person")
        time.sleep(0.1)
        return True
    
    result = sweepToFindObjAndSetGoal("person", "deliver", 4)
    
    if _interrupt_action:
        _interrupt_action = False
        return False
    
    if result:
        print("sweep got a lock on the person")
        return True
    else:
        ps = searchForPerson(_sdp, random.randint(0,1))
        if _interrupt_action:
            _interrupt_action = False
            return False
        if len(ps):
            # take first person for now, later check gender/age match
            p = ps[0]
            setLocationOfObj(_sdp, "deliver", p)
            return True        
    
    return False
    
_possObjObstacles = [
    "person", "suitcase", "chair", "cat", "frisbee", "pottedplant", 
    "backpack", "baseball", "bottle", "handbag", "tvmonitor"
]

_possObjOnTray = [
    "fork", "orange", "knife", "spoon", "carrot", "broccoli", "remote", "toothbrush", "bowl",
    "hot dog", "book", "bottle", "banana", "cell phone", "wine glass", "apple", "donut", 
    "tie", "sandwich", "scissors", "keyboard", "baseball", "cup"
]


def checkForObjects(objectsToCheck, objDict, numChecks, maxDist=30.0, needCentered=False, 
                    checkPersons=False, checkObjects=True, maxValueLen=2*_dai_fps, idx=0):
    lastValueLen = 0
    if len(objDict) > 0:
        lastValueLen = len(objDict[next(iter(objDict))])
        if lastValueLen < maxValueLen:
            idx = lastValueLen
    for i in range(0, numChecks):
        if idx > maxValueLen - 1:
            idx = 0
        ps = []

        # Assume object will not be seen this time
        for objValue in objDict.values():
            if len(objValue) < maxValueLen:
                objValue.append(False)
            else:
                objValue[idx] = False
        try:
            if checkObjects:
                ps = _mdai.getObjectDetections()
            if checkPersons:
                ps += _mdai.getPersonDetections()

            # If object is seen within required distance, then record it.
            for obj in ps:
                if obj.label in objectsToCheck:
                    if not objDict.get(obj.label):
                        objDict[obj.label] = [False] * (min(lastValueLen + i + 1, maxValueLen))
                    if obj.z <= maxDist and (not needCentered or obj.bboxCtr[0] >= 0.25 and obj.bboxCtr[0] <= 0.75):
                        objDict[obj.label][idx] = True
        except:
            None
            time.sleep(_dai_fps_recip)
        idx += 1
        time.sleep(_dai_fps_recip)
    return idx

def checkForSpecificObject(obj, numSecs=2, maxDist=30):
    objDict = {}
    print("check for specific object ", obj)
    checkForObjects([obj], objDict, numSecs * _dai_fps, maxDist, maxValueLen=numSecs * _dai_fps, checkPersons=(obj=="person"))
    objDict = computePersistance(objDict)
    print(objDict)
    return True if len(objDict) > 0 and next(iter(objDict.values())) > 0.5 else False

def checkForObjectOnTray(numSecs=2):
    objDict = {}
    print("check for object on tray")
    checkForObjects(_possObjOnTray, objDict, numSecs * _dai_fps, maxDist=2.5, maxValueLen= numSecs * _dai_fps)
    objDict = computePersistance(objDict)
    print(objDict)
    return next(iter(objDict)) if len(objDict) > 0 and next(iter(objDict.values())) > 0.25 else None

def checkForFace(numSecs=2):
    faceDict = {}
    print("check for faces")
    checkForFaces(faceDict, numSecs * _dai_fps, maxValueLen= numSecs * _dai_fps)
    faceDict = computePersistance(faceDict)
    print(faceDict)
    return next(iter(faceDict)) if len(faceDict) > 0 and next(iter(faceDict.values())) > 0.25 else None

def computePersistance(objDict):
    def by_value(item):
        return item[1]
    persistObjDict = {}
    for obj in objDict:
        persistObjDict[obj] = sum(objDict[obj]) / len(objDict[obj])
    return {k: persistObjDict[k] for k,v in sorted(persistObjDict.items(), reverse=True, key=by_value)}

def checkForFaces(faceDict, numChecks,needCentered=False, 
                  maxValueLen=2*_dai_fps, idx=0):
    lastValueLen = 0
    if len(faceDict) > 0:
        lastValueLen = len(faceDict[next(iter(faceDict))])
        if lastValueLen < maxValueLen:
            idx = lastValueLen
    for i in range(0, numChecks):
        if idx > maxValueLen - 1:
            idx = 0
        ps = []

        # Assume object will not be seen this time
        for objValue in faceDict.values():
            if len(objValue) < maxValueLen:
                objValue.append(False)
            else:
                objValue[idx] = False
        try:
            ps = _facial_recog.get_detected_names()

            for face in ps:
                if not faceDict.get(face):
                    faceDict[face] = [False] * (min(lastValueLen + i + 1, maxValueLen))
                faceDict[face][idx] = True
        except:
            None
            time.sleep(_dai_fps_recip)
        idx += 1
        time.sleep(_dai_fps_recip)
    return idx

#test checkForObject code
# import my_depthai
# import time
# from threading import Thread
# mdai = my_depthai.MyDepthAI()
# t = Thread(target = mdai.startUp, args=(True,)).start()

# objDict = {} # dictionary of objects identifications
# idx=0
# idx = checkForObjects(_possObjObstacles, objDict, 16, checkPersons=True, idx=idx)
# idx
# objDict
# x = computePersistance(objDict)
# x

#checkForSpecificObject("person", numChecks=16, maxDist=30)

def waitForObjectToBeTaken(obj):
    global _all_loaded
    a = 0
    mult_objs = obj.endswith("s")

    aim_oakd(yaw=79, pitch=move_oak_d._PITCH_LIMITS[1])
    eyes.setTargetPitchYaw(70, -40)

    if mult_objs: # for multiple objects use voice command to indicate when taken
        speak("Please take them, and then say, '" + _hotword + ", all taken.' when you are done.")
        timeout = time.monotonic() + 45
        while _all_loaded:
            a = time.monotonic()
            if a > timeout:
                break
            time.sleep(1)
        found = _all_loaded
        _all_loaded = False
        eyes.setHome()
    else: # single object to be detected there and gone by camera
        # find it first
        found = checkForSpecificObject(obj, maxDist=2.5)
        if not found:
            # move slightly to find object
            aim_oakd(yaw=80)
            found = checkForSpecificObject(obj, maxDist=2.5)
            if not found:
                aim_oakd(yaw=95)
                found = checkForSpecificObject(obj, maxDist=2.5)
        print("Found object to be taken = ", "True" if found else "False")
        speak("Please take it.")
        timeout = time.monotonic() + 10
        while found and time.monotonic() < timeout:
            found = checkForSpecificObject(obj, maxDist=2.5)
        if not found:
            # check after moving a little to make sure its gone
            aim_oakd(yaw=80)
            eyes.setTargetPitchYaw(70, -30)
            time.sleep(1)
            found = checkForSpecificObject(obj, maxDist=2.5)
            timeout = time.monotonic() + 10
            while found and time.monotonic() < timeout:
                found = checkForSpecificObject(obj, maxDist=2.5)
        _move_oak_d.allHome()
        eyes.setHome()
    return not found

def waitForObjectOnTray():
    objLabel = None
    time.sleep(1)
    timeout = time.monotonic() + 10
    while objLabel is None and time.monotonic() < timeout:
        objLabel = checkForObjectOnTray()
    if objLabel is not None:
        return objLabel

    # Aim down to left side of tray
    aim_oakd(yaw=99)
    eyes.setTargetPitchYaw(70, -40)

    # look for 2 seconds
    timeout = time.monotonic() + 2
    while objLabel is None and time.monotonic() < timeout:
        objLabel = checkForObjectOnTray()    
    
    if objLabel is None: # try moving a little and looking again for 2 seconds
        aim_oakd(yaw=95)
        timeout = time.monotonic() + 2
        while objLabel is None and time.monotonic() < timeout:
            objLabel = checkForObjectOnTray()    

    print("Found object placed on tray = ", "True" if objLabel is not None else "False")
    return objLabel

def sweepToFindObjAndSetGoal(obj, goal, sweepCount):
    global _interrupt_action
    global _goal
    checkPersons = obj == "person"
    checkObjects = not checkPersons
    _move_oak_d.startSweepingBackAndForth(sweepCount)
    lookingAgain = False
    while (not _interrupt_action) and _move_oak_d.isSweeping():
        objDict = {}
        checkForObjects([obj], objDict, 9, checkPersons=checkPersons, checkObjects=checkObjects, maxValueLen=9)
        objDict = computePersistance(objDict)
        if len(objDict) > 0 and objDict[obj] > 0.05:
            print("spotted", obj, ", stopping to get a look")
            _move_oak_d.stopSweepingBackAndForth()
            time.sleep(2) 
            if setFoundObjAsGoal(obj, cam_yaw=_move_oak_d.getYaw()):
                print("got a lock on ", obj)
                _goal = goal
                _move_oak_d.yawHome()
                # eyes looking to obj or person
                eyes.setTargetPitchYaw(50 if checkPersons else -50)
                return True
            else: # obj not found after stopping
                if lookingAgain:
                    if _move_oak_d.isSweeping():
                        continue
                    else: # failed to find it
                        _move_oak_d.stopSweepingBackAndForth()
                        lookingAgain = False
                        _move_oak_d.allHome()
                        eyes.setHome()
                        return False
                print("saw", obj, "but lost track of it")
                _move_oak_d.startSweepingBackAndForth(2)
                lookingAgain = True
                continue

def deliverToPersonInRoom(person, package, room):
    global _deliveree, _package, _spoken_package, _goal, _goal_queue, _all_loaded
    #go to person to pick up item 
    speak("Ok. I'll come get it.")
    # aim up to see people better
    aim_oakd(pitch=70) 
    eyes.setTargetPitchYaw(-70, 0)

    if setFoundObjAsGoal("person"): 
        print("got a lock on the person")
        _goal = "person"
        while _goal == "person":  # hack - wait until person is reached
            time.sleep(1)
    elif sweepToFindObjAndSetGoal("person", "person", 4):
        while _goal == "person":
            time.sleep(1)
    else:
        speak("I could not find you to get the item for delivery.")
        print("cound not find person to get package from")
        return
    
    if _error_last_goto:
        print("Could not find person to get package from")
        return

    time.sleep(2)

    # Look at right side of tray
    aim_oakd(yaw=79, pitch=move_oak_d._PITCH_LIMITS[1])
    eyes.setTargetPitchYaw(70, -40)
    _deliveree = person
    _package = package
    _spoken_package = package
    loc, dist, closeEnough = where_am_i()
    
    # handle multiple object packages with only voice command prompting and no detection by camera
    mult_objs = _spoken_package.endswith('s')
    if mult_objs:
        _all_loaded = False
        speak("Please place the " + _spoken_package + " on my tray and say, '" + _hotword + ", all loaded' when done.")
        timeout = time.monotonic() + 30
        while not _all_loaded and time.monotonic() < timeout:
            time.sleep(1)
        if not _all_loaded:
            speak("Ok, Fine. Forget it then.")
            return
    else: # single object to be detected by camera     
        speak("Please place the " + _spoken_package + " on my tray.")
        objLabel = waitForObjectOnTray()
        if objLabel is not None:
            _package = objLabel
    
    # aim oakd up to for detecting a person
    aim_oakd(yaw=move_oak_d._YAW_HOME_, pitch=80)
    eyes.setTargetPitchYaw(-70,0)
    print("delivering ", _package, " to ", _deliveree, " in the ", room)
    if mult_objs:
        speak("Ok, I will take the " + _spoken_package + " to " + _deliveree 
            + ((" at " + room) if room is not None else ""))
    else:
        speak("Ok, I will take this " + _spoken_package + " to " + _deliveree 
            + ((" in the " + room) if room is not None else ""))
            
    # if in another room set the first goal for the room
    if room is not None and (room != loc or not closeEnough):
        _goal_queue.append("deliver")
        _goal = room
    else: # in the same room, just deliver to person
        _goal = "deliver"
    
def come_here(doa):
    global _goal, _interrupt_action

    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)
    
    yawDelta = _mic_array.rotateToDoa(doa)

    ps = searchForPerson(sdp, yawDelta < 0)
    if len(ps) > 0:
        z = 9999.0
        # find closest person
        for p in ps:
            if p.z < z:
                z = p.z

        setLocationOfObj(sdp, "person", p)
        _goal = "person"
    else:
        if _interrupt_action:
            _interrupt_action = False
        else:
            speak("sorry, i could not find you.")
    sdp.disconnect()
    sdp.shutdown_server32(kill_timeout=1)
    sdp = None

def set_handling_response(value):
    global _handling_resp
    with _handling_resp_lock:
        _handling_resp = value

def handling_response():
    with _handling_resp_lock:
        return _handling_resp or _handle_resp_thread is not None and _handle_resp_thread.is_alive()

def handle_response_sync(sdp, phrase, doa, check_hot_word = True, assist = False):
    if handling_response():
        print("already handling response, try again later.")
        return HandleResponseResult.NotHandledBusy
    set_handling_response(True)
    try:
        handled_result = handle_response(sdp, phrase, doa, check_hot_word)
        if handled_result == HandleResponseResult.NotHandledUnknown:
            if assist:
                _sendToGoogleAssistantFn(phrase.strip(_hotword))
            else:
                speak("Sorry, I don't understand \"" + phrase.strip(_hotword) + "\"?")
    finally:
        set_handling_response(False)

def handle_response_async(sdp, phrase, doa, check_hot_word = True):
    # issue the command in its own thread
    _handle_resp_thread = Thread(target = handle_response_sync, args=(sdp, phrase, doa, check_hot_word), name = "handle_response_async", daemon=False)
    _handle_resp_thread.start()

###############################################################
# Command Handler
def handle_response(sdp, phrase, doa, check_hot_word = True):
    global _run_flag, _goal, _listen_flag, _last_phrase
    global _person, _mood, _time
    global _action_flag, _internet, _use_internet
    global _eyes_flag, _hotword, _sub_goal, _all_loaded
    global _show_rgb_window

    # convert phrase to lower case for comparison
    phrase = phrase.lower()
    if phrase == "stop" or phrase == "stop stop" or phrase == "orange stop":
        cancelAction(True)
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

    if (phrase == "what" or
        phrase == "what did you say" or
        phrase == "please repeat what you just said"):
        speak("I said")
        speak(_last_phrase)
        return HandleResponseResult.Handled
    
    if phrase == "goodbye":
        answer = "See you later, " + _person
        speak(answer)
        _person = "nobody"
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
    
    if phrase.startswith("find"):
        obj_p, _, loc = phrase.partition("find")[2].partition("in the")[0:3]
        obj = obj_p.split()[-1]
        loc = loc.strip()
        inThisRoom = loc == "room" or loc ==''
        if inThisRoom:
            sdp.wakeup()
        response = "Ok. I'll search for " + obj_p 
        if len(loc):
            response += " in the " + loc
        speak(response)

        if inThisRoom:
            time.sleep(3)
            lps = sdp.getLaserScan()
            longest_dist = 0
            longest_angle = 0
            for i in range(0, lps.size):
                if lps.distance[i] > longest_dist:
                    longest_dist = lps.distance[i]
                    longest_angle = lps.angle[i]

            longest_dist -= 0.75
            longest_dist = max(longest_dist, 0)
            print("other side of room: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
            pose = sdp.pose()
            xt = pose.x + longest_dist * math.cos(math.radians(pose.yaw) + longest_angle)
            yt = pose.y + longest_dist * math.sin(math.radians(pose.yaw) + longest_angle)
            _locations["custom"] = (xt, yt)
            sdp.setSpeed(1)
            _goal = "custom"
        else:
            _goal = loc
        _sub_goal = obj
        return HandleResponseResult.Handled

    if "go across the room and come back" in phrase:
        speak("Ok. I'm going across the room and coming back.")
        sdp.wakeup()
        time.sleep(6)
        lps = sdp.getLaserScan()
        longest_dist = 0
        longest_angle = 0
        for i in range(0, lps.size):
            if lps.distance[i] > longest_dist:
                longest_dist = lps.distance[i]
                longest_angle = lps.angle[i]

        longest_dist -= 0.75
        longest_dist = max(longest_dist, 0)
        print("other side of room: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
        pose = sdp.pose()
        xt = pose.x + longest_dist * math.cos(math.radians(pose.yaw) + longest_angle)
        yt = pose.y + longest_dist * math.sin(math.radians(pose.yaw) + longest_angle)
        _locations["custom"] = (xt, yt)
        _locations["origin"] = (pose.x, pose.y)
        _goal_queue.append("origin")
        _goal = "custom"
        return HandleResponseResult.Handled

    if phrase == "go recharge" or phrase == "go to dock" or phrase == "go to the dock":
        cancelAction(interrupt = True)
        _goal = "recharge"
        return HandleResponseResult.Handled

    if phrase == "go home":
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
        phrase = phrase.replace('\xb0', ' degrees') 
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
        pose = sdp.pose()
        xt = pose.x + dist * math.cos(math.radians(pose.yaw + phi))
        yt = pose.y + dist * math.sin(math.radians(pose.yaw + phi))
        _locations["custom"] = (xt, yt)
        _goal = "custom"
        return HandleResponseResult.Handled
    
    if phrase == "wake up":
        sdp.wakeup()
        return HandleResponseResult.Handled

    if phrase == "go to sleep":
        speak("Okay. I'm going to take a nap. Press my power button to wake me up.")
        _run_flag = False
        os.system("timeout 30 /nobreak && rundll32.exe powrprof.dll,SetSuspendState 0,1,0")
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
            
    if phrase.startswith("you are in the"):
        loc = phrase.partition("you are in the")[2].strip()
        locRect = _LOCATION_RECTS.get(loc)
        if locRect is not None:
            speak("Ok. I will relocate myself in the map. Please wait.")
            recoverLocalization(locRect)
        else:
            speak("I'm sorry. I don't have that location's area.")
        return HandleResponseResult.Handled
                    
    if phrase == "recover localization":
        speak("I will search the whole map to locate myself.")
        result = recoverLocalization(_HOUSE_RECT)
        if result == False:
            speak("I could not confirm my location.")
        return HandleResponseResult.Handled

    if phrase == "list your threads":
        print()
        pretty_print_threads()
        print()
        return HandleResponseResult.Handled
    
    if phrase == "open your eyes":
        if _eyes_flag == True:
            return HandleResponseResult.Handled
        _eyes_flag = True
        start_eyes_thread()
        return HandleResponseResult.Handled
    
    if phrase == "close your eyes":
        _eyes_flag = False
        eyes.shutdown()
        return HandleResponseResult.Handled
    
    if (phrase == "initiate restart"):
        global _restart_flag
        shutdown_eyes_thread()
        speak("Okay, I'm restarting.")
        _restart_flag = True
        _run_flag = False
        return HandleResponseResult.Handled

    if (phrase == "initiate shut down" or
        phrase == "initiate shutdown" or
        phrase == "begin shut down" or
        phrase == "begin shutdown"):
        shutdown_eyes_thread()
        speak("Okay, I'm shutting down.")
        #playsound("C:/Users/bjwei/wav/R2D2d.wav")
        # List all currently running threads
        print("\nClosing these active threads:")
        pretty_print_threads()
        # Have them terminate and close
        _run_flag = False
        return HandleResponseResult.Handled        

    if phrase == "shut down system":
        speak("Okay, I'm shutting down the system.")
        _run_flag = False
        os.system("shutdown /s /t 10")
        return HandleResponseResult.Handled
            
    if "battery" in phrase or "voltage" in phrase:
        ans = "My battery is currently at "
        ans = ans + str(sdp.battery()) + " percent"
        speak(ans)
        return HandleResponseResult.Handled
            
    if phrase.startswith("load map"):
        name = phrase[9:]
        if len(name) == 0:
            name = _default_map_name
        speak("Ok. I will load map " + name)
        loadMap(name)
        return HandleResponseResult.Handled
    
    if phrase.startswith("save map"):
        name = phrase[9:]
        if len(name) == 0:
            speak("please include the name of the map")
            return HandleResponseResult.Handled
            #name = _default_map_name
        speak("Ok. I will save map " + name)
        saveMap(name)
        return HandleResponseResult.Handled
    
    if "clear map" in phrase:
        speak("Ok. I will clear my map.")
        sdp.clearSlamtecMap()
        # after clearing make sure updating is on
        sdp.setUpdate(True)
        return HandleResponseResult.Handled
    
    if "clear locations" in phrase:
        speak("Ok. I will clear the locations.")
        _locations.clear()
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
        sdp.setUpdate(enable)
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
        global _user_set_speed
        if "low" in phrase:
            speed = 1
        elif "medium" in phrase:
            speed = 2
        elif "high" in phrase:
            speed = 3
        else:
            return HandleResponseResult.Handled
        _user_set_speed = speed
        speak("Ok. I'm setting the speed.")
        if speed != sdp.setSpeed(speed):
            speak("Sorry, I could not change my speed this time.")
        return HandleResponseResult.Handled
    
    if "you see" in phrase:
        results = detect.detect_objects(top_count=3)
        print(results)
        if results is not None:
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

    if phrase == "all loaded":
        _all_loaded = True
        return HandleResponseResult.Handled
    
    if phrase == "all taken":
        _all_loaded = False
        return HandleResponseResult.Handled

    if phrase == "show me your view":
        speak("Ok.")
        if not _show_rgb_window:
            _show_rgb_window = True
            shutdown_my_depthai()
            start_depthai_thread()
        return HandleResponseResult.Handled
    
    if phrase == "hide your view":
        speak("Ok.")
        if _show_rgb_window:
            _show_rgb_window = False
            shutdown_my_depthai()
            start_depthai_thread()
        return HandleResponseResult.Handled
    
    if phrase == "follow me":
        if _follow_thread is None:
            speak("Ok. I will follow you.")
            start_following()
        else:
            speak("I am already following you.")
        return HandleResponseResult.Handled

    if phrase == "stop following me":
        speak("Ok. I will stop following you.")
        stop_following()
        return HandleResponseResult.Handled

    if phrase == "track me":
        speak("Ok. I will track you.")
        start_tracking()
        return HandleResponseResult.Handled

    if phrase == "stop tracking me":
        stop_tracking()
        speak("Ok. I stopped tracking you.")
        return HandleResponseResult.Handled

    new_name = ""
    if phrase.startswith("i am"):
        new_name = phrase.partition("i am")[2]
    elif phrase.startswith("i'm"):
        new_name = phrase.partition("i'm")[2]
    elif phrase.startswith("my name is"):
        new_name = phrase.partition("my name is")[2]
    
    if len(new_name) > 0:
        _move_oak_d.setPitch(70) # pitch up to see person better
        eyes.setTargetPitchYaw(-70, 0)
        speak("Hello " + new_name + ". It's nice to meet you.")
        shutdown_my_depthai()
        speak("Wait while I try to commit your face to memory. Please, only you.")
        start_facial_recog(new_name=new_name)
        time.sleep(2)
        speak("ok. Next time try saying, 'Orange, hello' to check my memory.")
        shutdown_facial_recog()
        start_depthai_thread()
        eyes.setHome()
        _move_oak_d.allHome()
        return HandleResponseResult.Handled

    if phrase.startswith("hello") or phrase.startswith("hi"):
        _move_oak_d.setPitch(70) # pitch up to see person better
        eyes.setTargetPitchYaw(-70, 0)
        shutdown_my_depthai()
        speak("Hello There.", tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value)
        start_facial_recog()
        ided = False
        end = time.monotonic() + 10
        while time.monotonic() < end:
            face = checkForFace(1)   
            if face is not None:
                eyes.setText(face)
                speak(face + ", it's nice to see you again.")
                if _person == "nobody":
                    _person = face # set person i am with if with nobody
                ided = True
                break
        shutdown_facial_recog()
        start_depthai_thread()
        eyes.setHome()
        _move_oak_d.allHome()
        if not ided:
            speak("I don't believe we have met before. Try telling me your name.")
        return HandleResponseResult.Handled
    
    if "list locations" in phrase:
        speak("ok.")
        print("Locations I know:")
        for l in _locations.keys():
            print(l)
        return HandleResponseResult.Handled

    if phrase.startswith("delete location"):
        loc = phrase[16:]
        speak("Ok. I will delete location " + loc)
        _locations.pop(loc)
        return HandleResponseResult.Handled

    if phrase.startswith("update location of"):
        loc = phrase[19:]
        pose = sdp.pose()
        _locations[loc] = (pose.x, pose.y, math.radians(pose.yaw))
        print("updated location", loc, "to (", pose.x, pose.y, pose.yaw, ")")
        speak("I updated location " + loc)
        return HandleResponseResult.Handled

    if phrase.startswith("come here"):
        speak("Ok.")
        Thread(target = come_here, args=(doa,), name="Come Here", daemon=False).start()
        return HandleResponseResult.Handled

    if "local speech" in phrase:
        switch_to_local_speech()
        return HandleResponseResult.Handled

    if "cloud speech" in phrase:
        switch_to_cloud_speech()
        return HandleResponseResult.Handled

    if phrase.startswith("et = "):
        global _set_energy_threshold
        if _set_energy_threshold is not None:
            _set_energy_threshold(w2n.word_to_num(phrase.split()[2]))
            return HandleResponseResult.Handled
        else:
            return HandleResponseResult.NotHandledUnknown

    if phrase.startswith("ask google"):
        try:
            question = phrase.partition("ask google")[2]
            _sendToGoogleAssistantFn(question)
        except:
            return HandleResponseResult.NotHandledUnknown
        return HandleResponseResult.Handled

    if ("bring" in phrase or "take" in phrase) and ("this" in phrase or "these" in phrase):
        room = None
        person = None
        package = ""
        words = phrase.split(' ')
        try:
            words.remove("the")
        except:
            None
        ct = len(words)
        try:
            idx_this = words.index("this")
        except:
            idx_this = -1
        if idx_this < 0:
            try:
                idx_this = words.index("these")
            except:
                None
        if idx_this >=0:
            i = 1
            while ct > idx_this + i:
                temp = words[idx_this + i]
                if temp != "to":
                    package += temp + " "
                else:
                    break

                i += 1
            if package == "":
                package = "something"
        try:
            idx_to = words.index("to")
        except:
                return HandleResponseResult.NotHandledUnknown
        if ct > 3:
            person = words[idx_to + 1]
            if ct > 5:
                try:
                    idx_in = words.index("in")
                except:
                    idx_in = -1
                if idx_in < 0:
                    try:
                        idx_in = words.index("at")
                    except:
                        None
                if idx_in >=0:
                    person = " ".join(words[idx_to + 1:idx_in])
                    room = " ".join(words[idx_in+1:]).lower()
        else:
            return HandleResponseResult.NotHandledUnknown
        if room is not None:
            room_words = room.split()
            if len(room_words) > 1:
                try:
                    room = " ".join([room_words[0], str(w2n.word_to_num(room_words[1]))])
                except:
                    None             

            room = room.strip()
        
            if room in _locations:
                # run this in a separate thread so we can take voice answers
                Thread(target=deliverToPersonInRoom, args=(person.strip(), package.strip(), room), name="deliverToPersonInRoom", daemon=False).start()
            else:
                speak("Sorry, I don't know how to get to " + room + ".")
        else: # room is None i.e. same room
            Thread(target=deliverToPersonInRoom, args=(person.strip(), package.strip(), room), name="deliverToPersonInRoom", daemon=False).start()            
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
    global _run_flag, _goal, _last_speech_heard
    global _internet, _use_internet, _hotword, _google_mode
    
    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)
    
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
    
    def adj_spch_recog_ambient(r, m):
        # adjust microphone for ambient noise:
        # default dynamic thresholding does not work well, so disable it
        # the reason is that it only can sense reduction in noise and set the threshold lower.
        # it cannot set the threshold higher because if it hears something higher, then it assumes that is
        # speech and breaks out of the adjustment code to process the speech.
        # calibrate with r.adjust_for_ambient_noise(source)
        # 0 = it hears everything. 4000 = it hears nothing.
        
        r.dynamic_energy_threshold = False
        try:
            with m as source: r.adjust_for_ambient_noise(source, duration=1)
        except:
            None
        r.energy_threshold = max(300, r.energy_threshold)
        print("ambient energy threshold changed to ", r.energy_threshold)

    # beginning of actual Listen() code - <clean this up!>
    logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO)
    
    if _use_internet:
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
        #speak("I'm connected to Google Assistant.")
        #logging.info('Connecting to %s', api_endpoint)

        # create a recognizer object
        r = sr.Recognizer()

        # create a microphone object
        mic = sr.Microphone()

        def get_energy_threshold():
            return r.energy_threshold

        def set_energy_threshold(value):
            r.energy_threshold = value
            print("energy threshold changed to ", r.energy_threshold)

        #adj_spch_recog_ambient(r, mic)

        global _set_energy_threshold
        _set_energy_threshold = set_energy_threshold

        global _get_energy_threshold
        _get_energy_threshold = get_energy_threshold

    else:
        r = None
        mic = None
        
    speak("Hello, My name is Orange. Pleased to be at your service.")

    def listenFromVoskSpeechRecog(r, mic, sr):
        global _last_speech_heard
        # obtain audio from the microphone
        try:
            with mic as source:
                print("Say something!")
                audio = r.listen(source, phrase_time_limit = 7, is_speech_cb=_mic_array.getIsSpeech)
                doa = _mic_array.getDoa()
                setPixelRingThink()
                print("Your speech ended or timed out.")
        except Exception as e:
            print(e)
            return "", 0

        # recognize speech using Vosk Speech Recognition
        try:
            phrase = json.loads(r.recognize_vosk(audio))
            phrase = phrase["text"]           
            print("I heard: \"%s\" at %d degrees." % (phrase, _sdp.heading() + _mic_array.doa2YawDelta(doa)))
            _last_speech_heard = phrase
        except sr.UnknownValueError:
            phrase = ""
            print("What?")
        except sr.RequestError as e:
            phrase = ""
            print("Recognizer error; {0}".format(e))
        except:
            phrase = ""
            print("Unknown speech recognition error.")
        return phrase, doa

    def listenFromGoogleSpeechRecog(r, mic, sr):
        global _last_speech_heard, _internet, _google_mode
        # obtain audio from the microphone
        try:
            with mic as source:
                print("Say something!")
                audio = r.listen(source, phrase_time_limit = 7, is_speech_cb=_mic_array.getIsSpeech)
                doa = _mic_array.getDoa()
                setPixelRingThink()
                print("Your speech ended or timed out.")
        except Exception as e:
            print(e)
            return "", 0
        
        # recognize speech using Google Speech Recognition
        try:
            phrase = r.recognize_google(audio)
            _internet = True
            print("I heard: \"%s\" at %d degrees." % (phrase, _sdp.heading() + _mic_array.doa2YawDelta(doa)))
            _last_speech_heard = phrase
        except sr.UnknownValueError:
            phrase = ""
            _internet = True
            print("What?")
        except sr.RequestError:
            phrase = ""
            _internet = False
            # turn off google mode (cloud speech) if no internet so that local speech will stay active
            # it can be manually turned back on via command
            _google_mode = False
            speak("I lost my internet connection.")            
        except:
            phrase = ""
            print("Unknown speech recognition error.")
        return phrase, doa
        
    def sendToGoogleAssistant(phrase):
        global _internet
        if _internet:
            print("Sending question to google assistant: ", phrase)
            with TextAssistant(lang, device_model_id, device_id, display,
                    grpc_channel, grpc_deadline) as assistant:
                response_text, response_html = assistant.assist(text_query = phrase)
                if response_text:
                    speak(response_text)
                else:
                    speak("Sorry, I don't know about that.")
        else: 
            speak("I am not sure how to help with that.")

    global _sendToGoogleAssistantFn
    _sendToGoogleAssistantFn = sendToGoogleAssistant

    def listenFromVosk(sdp, finallyFunc=lambda:None, check_hot_word=True):
        try:
            phrase, doa = listenFromVoskSpeechRecog(r, mic, sr)
            setPixelRingTrace()
            handle_response_sync(sdp, phrase, doa, check_hot_word)
        except:
            speak("sorry, i could not do what you wanted.")
        finally:
            finallyFunc()

    def listenFromGoogle(sdp, finallyFunc=lambda:None, check_hot_word=True):
        try:
            phrase, doa = listenFromGoogleSpeechRecog(r, mic, sr)
            # slip in an ambient noise level adjustment here because some speech may have just 
            # ended or a timeout occurred.
            #adj_spch_recog_ambient(r, mic)
            setPixelRingTrace()
            handle_response_sync(sdp, phrase, doa, check_hot_word)
        except:
            speak("sorry, i could not do what you wanted.")
        finally:
            finallyFunc()

    # def local_hotword_recog_cb(sdp, phrase, doa, listener, hotword, r, sr):
    #     phrase = phrase.lower()
    #     print("I heard: %s" % phrase)
    #     if phrase == "stop" or "what's your name" in phrase or "what is your name" in phrase \
    #         or "who are you" in phrase:
    #         listener.set_active(False)
    #         try:
    #             handle_response_sync(sdp, phrase, doa, assist = False)
    #         except:
    #             speak("sorry, i could not do what you wanted.")
    #         finally:
    #             listener.set_active(True)
    #         return
    #     if hotword in phrase:
    #         listener.set_active(False)
    #         speak("yes?")
    #         listenFromGoogle(sdp, lambda:listener.set_active(True), False)

    def ask_google():
        if _use_internet and _internet:
            speak("Go ahead")
            phrase, doa = listenFromGoogleSpeechRecog(r, mic, sr)
            sendToGoogleAssistant(phrase)
        else:
            speak("ask google is not available.")

    def local_speech_recog_cb(phrase, listener, hotword, r, mic, sr, sdp):
        global _internet, _use_internet, _last_speech_heard
        doa = _mic_array.getDoa()
        print("I heard: \"%s\" at %d degrees" % (phrase, _sdp.heading() + _mic_array.doa2YawDelta(doa)))
        _last_speech_heard = phrase
        phrase = phrase.lower()
        listener.set_active(False)
        if phrase == "orange ask google":
            try:
                ask_google()
            finally:
                listener.set_active(True)    
            return
#        try:
        handled_result = handle_response_sync(sdp, phrase, doa, assist = False)
        if handled_result == HandleResponseResult.NotHandledUnknown:
            speak("I am not sure how to help with that.")
        elif handled_result == HandleResponseResult.NotHandledNoHotWord:
            print("No hot word, ignoring.")
#        except:
#           speak("sorry, i could not do what you wanted.")
#        finally:
        listener.set_active(True)
    
    global _starting_up
    _starting_up = False
    setPixelRingEndStartup() # restore pixel ring to default sound sensitive mode after boot up
    
    while _run_flag:
        #local_listener = None
        use_local_speech = not _use_internet or not _internet or not _google_mode
        try:
            if use_local_speech:
                print("local listener")
                # if no internet access or google mode is inactive, use WSR / SAPI
                # to recognize a command subset
                listenFromVosk(sdp)
                #local_listener = winspeech.listen_for(None, "speech.xml", 
                #"RobotCommands", lambda phrase, listener, hotword=_hotword, r=r,
                #sr=sr, sdp=sdp: local_speech_recog_cb(phrase, listener, hotword, r, mic, sr, sdp))
            else: 
                # use google cloud speech
                listenFromGoogle(sdp)

                # winspeech to detect hotword or stop and then invoke google 
                # cloud speech
                # print("detecting hotword")
                # local_listener = winspeech.listen_for(None, "hotword.xml", 
                # "RobotHotword", lambda phrase, listener, hotword=_hotword, r=r, 
                # sr=sr: local_hotword_recog_cb(sdp, phrase, doa, listener, hotword, r, sr))
        except Exception as e:
            print(e)

        #while _run_flag and local_listener is not None and not _google_mode:
        #   time.sleep(2)
    
    # if no longer running, stop listening 
    #winspeech.stop_listening()
    sdp.disconnect()
    sdp.shutdown_server32(kill_timeout=1)
    sdp = None

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

def aim_oakd(yaw = None, pitch = None):
    if yaw is not None:
        _move_oak_d.setYaw(yaw)
    if pitch is not None:
        _move_oak_d.setPitch(pitch)

def initialize_speech():
    global _voice
    _voice = tts.sapi.Sapi()
    _voice.set_voice("Mark") # David, Mark, Eva, or Zira. 
    _voice.voice.Volume = 100
    _voice.voice.SynchronousSpeakTimeout = 1 # timeout in milliseconds

def start_facial_recog(new_name=""):
    global _facial_recog, _facial_recog_thread
    if len(new_name) > 0:
        _facial_recog = fr.FacialRecognize(add_face=True, debug=False, new_name = new_name)
    else:
        _facial_recog = fr.FacialRecognize(debug=False)

    _facial_recog_thread = Thread(target=_facial_recog.run, name="Facial Recog", daemon=False)
    _facial_recog_thread.start()
    while not _facial_recog.run_flag:
        time.sleep(1)

def shutdown_facial_recog():
    global _facial_recog_thread
    try:
        _facial_recog.shutdown()
        _facial_recog_thread.join()
    except:
        None
    
def start_depthai_thread(model="tinyYolo", use_tracker=False):
    global _my_depthai_thread, _mdai
    _mdai = my_depthai.MyDepthAI(model, use_tracker)

    _my_depthai_thread = Thread(target = _mdai.startUp, args=(_show_rgb_window, _show_depth_window), name="mdai", daemon=False)
    _my_depthai_thread.start()

def shutdown_my_depthai():
    global _my_depthai_thread
    try:
        _mdai.shutdown()
        _my_depthai_thread.join()
    except:
        None

def start_eyes_thread():
    global _eyes_thread
    _eyes_thread = Thread(target = eyes.start, args=(handle_op_request,), name = "Eyes")
    _eyes_thread.start()

def shutdown_eyes_thread():
    try:
        eyes.shutdown()
        _eyes_thread.join()
    except:
        None

def start_tracking():
    # shutdown current depth ai model
    shutdown_my_depthai()
    # start up the mobilenet with tracker. Yolo doesn't work as well.
    start_depthai_thread(model="mobileNet", use_tracker=True)    
    _move_oak_d.start_tracking(_mdai, trackTurnBase=True)

def stop_tracking():
    _move_oak_d.stop_tracking()
    shutdown_my_depthai()
    # restore default depth ai model
    start_depthai_thread()

_follow_thread = None
_following = False
_last_goal_pos = (0,0)

def follow_me():
    global _following, _last_goal_pos
    print("Follow Me thread starting")
    _following = True

    # must establish a separate client and server and connection to SDP because msl loadlib is not thread safe
    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)

    start_tracking()
    last_track_update = 0
    backup = 0
    backup_cnt = 0
    while _following:
        if time.monotonic() - last_track_update > 3.0:
            ts = _move_oak_d.get_track_status()
            if ts.tracking == move_oak_d.TrackingResult.Tracked:
                ts.object.z -= 1.2 # come up to the object within certain distance
                robot_pose = sdp.pose()
                heading = math.radians(robot_pose.yaw + _move_oak_d.getYaw() + ts.object.theta)
                xt = robot_pose.x + ts.object.z * math.cos(heading)
                yt = robot_pose.y + ts.object.z * math.sin(heading)
                if ts.object.z > 0.25 or distance_A_to_B(_last_goal_pos[0], _last_goal_pos[1], xt, yt) > 0.25:
                    #print("follow person moved, now: %2.2f meters at %3.0f degrees." % (ts.object.z, robot_pose.yaw + _move_oak_d.getYaw() + ts.object.theta))
                    # disallow base turning when setting a nav target
                    _move_oak_d.set_track_turn_base(False)
                    if ts.object.z > 0:
                        sdp.moveToFloatWithYaw(xt, yt, heading)
                    elif ts.object.z < 0: # too close, back up
                        sdp.cancelMoveAction()
                        #print("too close, backing up")
                        backup = 5
                    _last_goal_pos = (xt, yt)
                else:
                    _move_oak_d.set_track_turn_base(True)
            # elif _sdp.getMoveActionStatus() != ActionStatus.Running and ts.tracking == move_oak_d.TrackingResult.Lost:
            #     speak("Sorry, I lost you.")
            #     # tracking id will be a new one at this point so reset the tracker to take the closet person
            #     _move_oak_d.clearLastTrackedObj()
            last_track_update = time.monotonic()
        if backup > 0:
            backup_cnt += 1
            if backup_cnt >= 2:
                backup_cnt = 0
                move_imm(sdp, -1)
                backup -= 1
        time.sleep(0.1)
    print("Follow Me thread ending")
    stop_tracking()
    sdp.disconnect()
    sdp.shutdown_server32(kill_timeout=1)
    sdp = None

def start_following():
    global _follow_thread
    if _follow_thread is None:
        _follow_thread = Thread(target=follow_me, name="Follow Me", daemon=False)
        _follow_thread.start()
    else:
        print("Error - trying to start following when already following.")

def stop_following():
    global _follow_thread, _following
    _following = False
    if _follow_thread is not None:
        _follow_thread.join()
        _follow_thread = None

def switch_to_local_speech():
    global  _google_mode
    _google_mode = False
    speak("Ok.")

def switch_to_cloud_speech():
    global  _google_mode
    if not _internet or not _use_internet:
        speak("I cannot connect to cloud speech.")
    else:
        # stop local speech recog
        #winspeech.stop_listening()
        _google_mode = True
        speak("Ok.")

def radar_main():
    while _run_flag:
        
        time.sleep(0.25)

def start_radar():
    global _radar_thread
    if _radar_thread is None:
        _radar_thread = Thread(target=radar_main, name="Radar", daemon=False)
        _radar_thread.start()
    else:
        print("Error - trying to start following when already following.")

def stop_following():
    global _radar_thread, _following
    _following = False
    if _radar_thread is not None:
        _radar_thread.join()
        _radar_thread = None

def enable_radar():
    global _radar_enabled
    _radar_enabled = True


from orange_utils import *

def handle_op_request(sdp, opType : OrangeOpType, arg1=None, arg2=None):
    global _last_speech_heard
    if opType == OrangeOpType.TextCommand:
        return handle_response_async(sdp, arg1, 0, check_hot_word=False)
    elif opType == OrangeOpType.BatteryPercent:
        return sdp.battery()
    elif opType == OrangeOpType.LastSpeechHeard:
        return _last_speech_heard
    elif opType == OrangeOpType.LastSpeechSpoken:
        return _last_phrase
    elif opType == OrangeOpType.IpAddress:
        return socket.gethostbyname(socket.gethostname())
    elif opType == OrangeOpType.GoogleSpeech:
        return _google_mode
    elif opType == OrangeOpType.ToggleGoogleSpeech:
        if _google_mode:
            switch_to_local_speech()
        else:
            switch_to_cloud_speech()
        return _google_mode
    elif opType == OrangeOpType.InternetStatus:
        return _internet
    elif opType == OrangeOpType.BatteryIsCharging:
        return sdp.getBatteryIsCharging()
    elif opType == OrangeOpType.BoardTemperature:
        return sdp.getBoardTemperature()
    elif opType == OrangeOpType.LocalizationQuality:
        return sdp.getLocalizationQuality()
    elif opType == OrangeOpType.WifiSsidAndStrength:
        current_network = subprocess.check_output(["netsh", "wlan", "show", "interfaces"], text=True).split('\n')
        connected_ssid = ""
        signal = ""
        ssid_line = [x for x in current_network if 'SSID' in x and 'BSSID' not in x]
        if ssid_line:
            ssid_list = ssid_line[0].split(':')
            connected_ssid = ssid_list[1].strip()
        sig_line = [x for x in current_network if 'Signal' in x]
        if sig_line:
            signal = sig_line[0].split(':')
            signal = signal[1].strip()
        return connected_ssid, signal
    elif opType == OrangeOpType.SpeechEnergyThreshold:
        return _get_energy_threshold()
        
################################################################   
# This is where data gets initialized from information stored on disk
# and threads get started
def initialize_robot():
    global _moods, _internet, _eyes_flag, _facial_recog
    global _listen_thread

    _internet = True

    start_depthai_thread()
    
    _listen_thread = Thread(target = listen, name = "Listen")
    _listen_thread.start()
    
    Thread(target = time_update, name = "Time").start()
        
#    Thread(target = monitor_motion, name = "People Motion Monitor").start()
    
#    Thread(target = behaviors, name = "Behaviors").start()
    if _slamtec_on:    
        Thread(target = handleGotoLocation, name = "Handle Goto Location").start()

#    Thread(target = actions, name = "Actions").start()
    
#    Thread(target = robotMoving, name = "Is Robot Moving").start()
    
#    Thread(target = megaCOM7, name = "megaCOM7").start()
    
#    Thread(target = eyes, name = "Display eyes").start()
    
    #Thread(target = display, name = "Display").start()

    if _eyes_flag: # this is needed to start the display and to keep it open
        start_eyes_thread()
################################################################
# This is where data gets saved to disk
# and by setting _run_flag to False, threads are told to terminate
def shutdown_robot():
    global _run_flag, _moods, _sdp
    
    cancelAction()
    _run_flag = False

    print("stop following if doing so")
    stop_following()
    print("shutting down eyes")
    eyes.shutdown()
    print("shutting down depthai")
    shutdown_my_depthai()
    print("shutting doen facial recog")
    shutdown_facial_recog()
    print("shutting down move oakd")
    _move_oak_d.shutdown()
    _pixel_ring.close()
    _mic_array.close()
    
    # TBD join threads
    
    # Close all open threads
    print("\nWaiting for sub threads to finish.")
    while True:
        i = 1
        count = 0
        threads = threading.enumerate()
        print()
        for item in threads:
            print(i, ":", item)
            i += 1
            if not item.isDaemon():
                count += 1
        print()
        if count <= 1:
            break
        time.sleep(2)
    
    #print("waiting for listening thread to complete.")
    #if _listen_thread is not None:
    #    _listen_thread.join()
    _sdp.disconnect()
    _sdp.shutdown_server32(kill_timeout=1)
    _sdp = None
    print("\nDone!")
   
################################################################
# This initializes and runs the robot
def robot():
    global _run_flag, _restart_flag
    _run_flag = True
    _restart_flag = False

    initialize_robot()
    eyes.setTargetPitchYaw(-10, 0)
    _sdp.setSpeed(_user_set_speed)

    last_battery_check = 0
    try:
        while _run_flag:
            if time.monotonic() - last_battery_check >= 10:
                batteryMonitor()
                last_battery_check = time.monotonic()
            time.sleep(1)
    except KeyboardInterrupt:
        pretty_print_threads()

    shutdown_robot()

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

    def rotateToDoa(self, doa):
        yawDelta = self.doa2YawDelta(doa)
        print("turning toward where heard person")
        turn(yawDelta)
        return yawDelta

    def close(self):
        usb.util.dispose_resources(self.dev)

# def init_local_speech_rec():
#     # Start an in-process edge recognizer using SAPI.
#     winspeech.initialize_recognizer(winspeech.INPROC_RECOGNIZER)
    
def save_locations(name):
    with open(name + ".pkl", "wb") as f:
        pickle.dump(_locations, f)

def load_locations(name):
    global _locations
    with open(name + ".pkl", "rb") as f:
        _locations = pickle.load(f)

# TBD make this into a class
def setPixelRingSpeak():
    _pixel_ring.speak()

def setPixelRingThink():
    _pixel_ring.think()

def setPixelRingStartup():
    setPixelPaletteForSpin()
    setPixelRingSpin()

def setPixelRingEndStartup():
    setPixelPaletteDefault()
    setPixelRingTrace()

def setPixelRingTrace():
    if _starting_up:
        setPixelRingSpin()
    else:
        _pixel_ring.trace()

def setPixelRingSpin():
    _pixel_ring.spin()

def setPixelPaletteYellow():
    _pixel_ring.set_color_palette(0x005050,0x402000) 

def setPixelPaletteRed():
    _pixel_ring.set_color_palette(0x005050,0x400000)
    
def setPixelPaletteDefault():
    _pixel_ring.set_color_palette(0x003000,0x700800)

def setPixelPaletteForSpin():
    _pixel_ring.set_color_palette(0x700800,0x003000)

def setPixelPaletteBootDefault():
    _pixel_ring.set_color_palette(0x005050,0x000050)

def run():
    global _sdp, _slamtec_on, _move_oak_d, _mic_array, _pixel_ring
    # Start 32 bit bridge server
    _sdp = MyClient()

    #init_local_speech_rec()
    initialize_speech()
    _mic_array = MicArray()
    _pixel_ring = pixel_ring.PixelRing(_mic_array.dev)
    setPixelRingStartup()
    #init_camera()

    speak("I'm starting up.")
    _move_oak_d = move_oak_d.MoveOakD()
    _move_oak_d.initialize()

    res = sdp_comm.connectToSdp(_sdp)

    if _execute:
        if (res == 0):
            _slamtec_on = True
            #loadMap(_default_map_name)
            # set not to update map by default.
            _sdp.setUpdate(True)
            None
        else:
            speak("I could not connect to Slamtec. Movement is disabled.")
    
        robot()
    else:
        print("Program is in DEBUG mode.")
        #initialize_robot()

#import getopt

def main(argv):
    # try:
    #     opts, args = getopt.getopt(argv,"hi:o:",["ifile=","ofile="])
    # except getopt.GetoptError:
    #     print 'test.py -i <inputfile> -o <outputfile>'
    #     sys.exit(2)
    # for opt, arg in opts:
    #     if opt == '-h':
    #         print 'test.py -i <inputfile> -o <outputfile>'
    #         sys.exit()
    #     elif opt in ("-i", "--ifile"):
    #         inputfile = arg
    #     elif opt in ("-o", "--ofile"):
    #         outputfile = arg
    # print 'Input file is "', inputfile
    # print 'Output file is "', outputfile
    
    while 1:
        run()
        if not _restart_flag:
           break 

if __name__ == "__main__":
   main(sys.argv[1:])
