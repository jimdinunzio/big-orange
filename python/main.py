# -*- coding: utf-8 -*-
"""
Created on Sun Jun  7 17:39:44 2020

@author: LattePanda
"""
import math
import random

# Constants
_show_rgb_window = False
_show_depth_window = False
_hotword = "orange"
_google_mode = True
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
_HOUSE_RECT = {"left":-0.225,"bottom":-5.757, "width":12.962, "height":7.6}
_OFFICE_RECT = {"left":0.405,"bottom":-0.128, "width":3.6, "height":1}
_KITCHEN_RECT = {"left":10.3,"bottom":-4.9, "width":1.8, "height":1.6}
_INIT_RECT =  {"left":-0.5,"bottom":-0.5, "width":1.0, "height":1.0}
_STARTUP_ROOM = _OFFICE_RECT
_DELIVERY_RESPONSES = [
    "You know, I'm only doing this until I get discovered... I, want to direct... For now, its back to the bar",
    "Since you asked... This, is just my day job... At night, I'm shooting an indie film... Oh well, back to the bartender",
    "Waiting tables of people guzzling down their drinks is a means to an end for me... Filmmaking is, my real passion. Later."]

_LOCATION_RECTS = { "kitchen": _KITCHEN_RECT, "office": _OFFICE_RECT}
_dai_fps = 17 # depthai approx. FPS (adjust lower to conserve CPU usage)
_dai_fps_recip = 1.0 / _dai_fps

# Globals
_slamtec_on = False
_goal = ""
_sub_goal = ""
_goal_queue = []
_time = "morning"
_times = ["morning", "noon", "afternoon", "evening", "night"]
_last_phrase = "nothing"
_listen_flag = True
_action_flag = False # True means some action is in progress
_interrupt_action = False # True when interrupting a previously started action
_request = "" # comes from someone telling the robot to do something
_thought = "" # comes from the robot thinking that it wants to do something
_do_something_flag = False
_motion_flag = False  # set by monitor_motion thread, looking for humans in motion
_robot_is_moving = False # set by robotMotion thread: True when robot is in motion
_last_motion_time = 0 # the time.time() motion was last detected
_ser6 = ""
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
import winspeech
from enum import Enum
import move_oak_d as oakd
import my_depthai
import eyes
import facial_recognize as fr
import pickle

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
    global _sdp, _action_flag, _robot_is_moving
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
    print("I am at location = ", location, " distance = ", distance)
    if (distance <= 1.0):
        closeEnough = True
    else:
        closeEnough = False
    return location, distance, closeEnough

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

def batteryMonitor():
    global _sdp, _run_flag, _person
    reported_25 = False
    reported_15 = False
    reported_10 = False
    while _run_flag:
        try:
            batteryPercent = _sdp.battery()
            if batteryPercent <= 10:
                if not reported_10:
                    reported_10 = True
                    speak(_person + ", my battery is exhausted, and I am shutting down now.") 
                    cancelAction()
                    _run_flag = False
                    os.system("shutdown /s /t 30")
            elif batteryPercent <= 15:
                if not reported_15:
                    reported_15 = True
                    speak(_person + ", I need to recharge my battery now or I will have to shut down.")
            elif batteryPercent <= 25:
                if not reported_25:
                    reported_25 = True
                    speak(_person + ", my battery is getting low. I'll have to charge up soon.")
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
    global _deliveree, _package, _sub_goal, _call_out_objects
    global _error_last_goto, _response_num, _locations
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
            _sdp.home()
        elif _goal == "deliver" and _locations.get(_goal) is None:
            # Hack for multiple person delivery, do not bother looking for a person
            pose = _sdp.pose()
            _locations[_goal] = (pose.x, pose.y, math.radians(pose.yaw)) 
            continue
            # need to find person in room for delivery
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
                location, distance, closeEnough = where_am_i()
                if _goal == location and distance < 0.50:
                    speak("I'm already at the " + _goal)
                    _goal = ""
                    if len(_goal_queue) > 0:
                        _goal = _goal_queue.pop(0)
                    continue
                if _goal != sub_goal_cleanup and len(_goal_queue) == 0 or len(_goal_queue) > 0 and _goal_queue[0] != "deliver":
                    None
                    #speak("I'm going to the " + _goal)
            elif _goal == "deliver":
                speak("hello " + _deliveree)
            # else:
            #     speak("OK.")
            _action_flag = True
            if len(coords) == 3:
                _sdp.moveToFloatWithYaw(coords[0], coords[1], coords[2])
            else:
                _sdp.moveToFloat(coords[0], coords[1])

        _interrupt_action = False
        sleepTime = 0.5 if (_sub_goal == "" or _call_out_objects) else 0
        if _sub_goal != "":
            checkPersons = _sub_goal == 'person'
            if checkPersons:
                aim_oakd(pitch=80) # aim up to see people better                
                eyes.setAngleOffset(90, -50)
            else:
                aim_oakd(pitch=110) # aim down towards floor for objects
                eyes.setAngleOffset(90, 50)
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
            oakd.yawHome()
            eyes.setHome()
            _sdp.setSpeed(_user_set_speed) #restore speed after finding obj

        if _sub_goal != "":
            if setFoundObjAsGoal(_sub_goal):
                gotLock()
                sub_goal_just_found = True
                sub_goal_cleanup = _sub_goal
            else:
                oakd.startSweepingBackAndForth()
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
                    _sdp.cancelMoveAction()
                    oakd.stopSweepingBackAndForth()
                    time.sleep(2)
                    if setFoundObjAsGoal(_sub_goal, cam_yaw=oakd.getYaw()):
                        gotLock()
                        sub_goal_just_found = True
                        sub_goal_cleanup = _sub_goal
                        break
                    else:
                        print("saw", _sub_goal, "but lost track of it")
                        #speak("I thought I saw a " + _sub_goal + ". I'll look again.")
                        oakd.startSweepingBackAndForth(1)
                        while oakd.isSweeping():
                            if setFoundObjAsGoal(_sub_goal, cam_yaw=oakd.getYaw()):
                                gotLock()
                                sub_goal_just_found = True
                                sub_goal_cleanup = _sub_goal
                                break
                        if sub_goal_just_found:
                            break    
                        #speak("I'll keep going.")
                        _sdp.moveToFloat(coords[0], coords[1])

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
            oakd.stopSweepingBackAndForth()
            oakd.allHome()
            eyes.setHome()
            _goal_queue.clear()

        if not sub_goal_just_found:
            # reaching this point, the robot first moved, then stopped - so check where it is now
            location, distance, closeEnough = where_am_i()
            # and now check to see if it reached the goal
            if (_goal == "deliver" or location == _goal and closeEnough):
                if _goal == "deliver":
                    # aim down to tray
                    if _package is not None:
                        if _package =="bottle":
                            aim_oakd(yaw=90, pitch=120) # best angle for bottle
                        elif _package == "cup":
                            aim_oakd(yaw=90, pitch=105) # best angle for can (detected as cup)

                    eyes.set(0,0)
                    eyes.setAngleOffset(-70, -40)                    
                    speak(_deliveree + ", I have some " + _spoken_package + " for you.")
                    taken = waitForObjectToBeTaken(_package)
                    if taken:
                        speak("Great, and you're welcome. ")
                        speak(_DELIVERY_RESPONSES[_response_num])
                        _response_num = (_response_num + 1) % len(_DELIVERY_RESPONSES)
                    else:
                        speak("Well, I hope you got your drinks. I'm headed back to the bar.")
                        #speak("Sorry, don't you want the " + _package + "?")
                    _deliveree = None
                elif _goal == sub_goal_cleanup:
                    speak("I found the " + _goal)
                    oakd.allHome()
                elif _goal == "bar":
                    speak("Hey barkeep! I'm back for more punishment. Hit me up.")
                elif _goal != "person":
                    None
                    #speak("I've arrived.")
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
                _sdp.setSpeed(_user_set_speed) #restore speed after finding obj

                
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
        oakd.stopSweepingBackAndForth()
        if len(_goal_queue) > 0:
            _goal = _goal_queue.pop(0)
        else:
            oakd.allHome()
            eyes.setHome()

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

def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value):
    global _last_phrase, _voice

    try:
        print(phrase)
        _voice.say(phrase, flag)
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
    res = _sdp.saveSlamtecMap(b'MyHouse.stcm')
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
def searchForPerson(clockwise):
    global _sdp, _action_flag, _interrupt_action
    
    aim_oakd(pitch=80) # aim up to see people better
    eyes.set(0,0)             
    eyes.setAngleOffset(90, -50)

    ps = []
    # First see if person is already in view and if so return
    found, ps = checkForPerson()
    if found:
        return ps
    
    # if no person in view, then slowly rotate 360 degrees and check every so often
    _action_flag = True
    
    oldyaw = _sdp.pose().yaw + 360
    yaw = oldyaw
    sweep = 0
    nextPause = 20
    recheck_person = False
    while (sweep < 380 and not _interrupt_action):
        _sdp.right()# if clockwise else _sdp.left()
        time.sleep(0.1)
        found, ps = checkForPerson()
        if found:
            recheck_person = True
            break
        yaw = _sdp.pose().yaw + 360
        covered = abs(yaw - oldyaw)
        if covered > 180:
            covered = 360 - covered
        sweep += covered
        #print("yaw = ", yaw, " covered = ", covered, " sweep = ", sweep)

        oldyaw = yaw
        if sweep >= nextPause:
            _sdp.cancelMoveAction()
            time.sleep(0.3)
            found, ps = checkForPerson()
            if found:
                break
            nextPause += 20
    if recheck_person:
        print("rechecking person")
        _sdp.cancelMoveAction()
        time.sleep(0.3)
        for j in range(1,5):
            for i in range(1,5):
                found, ps = checkForPerson()
                if found:
                    break
            if not found:
                # go back other way
                deg = -5 if j % 2 == 0 else 5
                _sdp.rotate(math.radians(deg))
                _sdp.waitUntilMoveActionDone()
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
                ps = my_depthai.getPersonDetections()
            else:
                ps = my_depthai.getObjectDetections()
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
    for i in range(1,5):
        try:
            ps = my_depthai.getPersonDetections()
            if len(ps) > 0:
                p = ps[0]
                # If bbox ctr of detection is away from edge then stop
                if p.bboxCtr[0] >= 0 and p.bboxCtr[0] <= 1:
                    print("Person at bbox ctr: ",p.bboxCtr[0], ", ", p.bboxCtr[1])
                    return True, ps
                    break
        except:
            None
            time.sleep(_dai_fps_recip)
        time.sleep(_dai_fps_recip)
    return False, ps

def setLocationOfObj(obj, p, cam_yaw=0):
    global _locations
    p.z -= 1 # come up to the object within certain distance
    pose = _sdp.pose()
    xt = pose.x + p.z * math.cos(math.radians(pose.yaw + cam_yaw + p.theta))
    yt = pose.y + p.z * math.sin(math.radians(pose.yaw + cam_yaw + p.theta))
    print("set location of ", obj, " at distance ", p.z, " meters at ", cam_yaw + p.theta, "degrees")
    _locations[obj] = (xt, yt)

def setFoundObjAsGoal(obj, cam_yaw=0):
    global _locations, _sdp, _interrupt_action
    found, p = checkForObject(obj)
    if found:
        setLocationOfObj(obj, p, cam_yaw)
        return True
    return False

def findObjAndSetGoal(obj, goal, cam_yaw=0):
    global _locations, _sdp, _interrupt_action
    found, p = checkForObject(obj)
    if found:
        setLocationOfObj(goal, p, cam_yaw)
        return True
    return False

def setDeliverToPersonAsGoal():
    global _locations, _sdp, _interrupt_action, _goal

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
        ps = searchForPerson(random.randint(0,1))
        if _interrupt_action:
            _interrupt_action = False
            return False
        if len(ps):
            # take first person for now, later check gender/age match
            p = ps[0]
            setLocationOfObj("deliver", p)
            return True        
    return False
    
_possObjObstacles = [
    "person", "suitcase", "chair", "cat", "frisbee", "pottedplant", 
    "backpack", "baseball", "bottle", "handbag", "tvmonitor"
]

_possObjOnTray = [
#    "fork", "orange", "knife", "spoon", "carrot", "broccoli", "remote", "toothbrush", "bowl",
#    "hot dog", "book", "bottle", "banana", "cell phone", "wine glass", "apple", "donut", 
#    "tie", "sandwich", "scissors", "keyboard", "baseball", "cup"
"cup", "bottle"
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
                ps = my_depthai.getObjectDetections()
            if checkPersons:
                ps += my_depthai.getPersonDetections()

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
# t = Thread(target = my_depthai.startUp, args=(True,)).start()

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
#    time.sleep(2)
    a = 0
    #if obj is None:
    if 1:
        speak("Please take them, and then say, 'Orange, all taken.' when you are done.")
        timeout = time.monotonic() + 45
        while _all_loaded:
            a = time.monotonic()
            if a > timeout:
                break
            time.sleep(1)
        found = _all_loaded
        _all_loaded = False
        eyes.setHome()
    else:
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
            eyes.setAngleOffset(-110, -40)
            time.sleep(1)
            found = checkForSpecificObject(obj, maxDist=2.5)
            timeout = time.monotonic() + 10
            while found and time.monotonic() < timeout:
                found = checkForSpecificObject(obj, maxDist=2.5)
        oakd.allHome()
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
    #aim_oakd(yaw=99)
    #eyes.setAngleOffset(-110, -40)

    # aim_oakd(pitch=129) #check for beer (detected as bottle)
    # timeout = time.monotonic() + 2
    # while objLabel is None and time.monotonic() < timeout:
    #     objLabel = checkForObjectOnTray()    
    
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
    oakd.startSweepingBackAndForth(sweepCount)
    lookingAgain = False
    while (not _interrupt_action) and oakd.isSweeping():
        objDict = {}
        checkForObjects([obj], objDict, 9, checkPersons=checkPersons, checkObjects=checkObjects, maxValueLen=9)
        objDict = computePersistance(objDict)
        if len(objDict) > 0 and objDict[obj] > 0.05:
            print("spotted", obj, ", stopping to get a look")
            oakd.stopSweepingBackAndForth()
            time.sleep(2) 
            if setFoundObjAsGoal(obj, cam_yaw=oakd.getYaw()):
                print("got a lock on ", obj)
                _goal = goal
                oakd.yawHome()
                # eyes looking to obj or person
                eyes.set(0,0)
                eyes.setAngleOffset(90, -50 if checkPersons else 50)
                return True
            else: # obj not found after stopping
                if lookingAgain:
                    if oakd.isSweeping():
                        continue
                    else: # failed to find it
                        oakd.stopSweepingBackAndForth()
                        lookingAgain = False
                        oakd.allHome()
                        eyes.setHome()
                        return False
                print("saw", obj, "but lost track of it")
                oakd.startSweepingBackAndForth(2)
                lookingAgain = True
                continue

def deliverToPersonInRoom(person, package, room):
    global _deliveree, _package, _spoken_package, _goal, _goal_queue, _all_loaded
    # #go to person to pick up item 
    # speak("Ok. I'll come get it.")
    # # aim up to see people better
    # aim_oakd(pitch=80) 
    # eyes.set(0,0)
    # eyes.setAngleOffset(90, -50)

    # if setFoundObjAsGoal("person"): 
    #     print("got a lock on the person")
    #     _goal = "person"
    #     while _goal == "person":  # hack - wait until person is reached
    #         time.sleep(1)
    # elif sweepToFindObjAndSetGoal("person", "person", 4):
    #     while _goal == "person":  # hack - wait until person is reached
    #         time.sleep(1)
    # else:
    #     print("cound not find person to get package from")
    #     return
    #     # ps = searchForPerson(True)
    #     # if len(ps):
    #     #     p = ps[0]
    #     #     setLocationOfObj("deliver", p)
    #     # else:
    #     #     speak("I could not find you to get the item for delivery.")
    #     #     return

    # if _error_last_goto:
    #     print("Could not find person to get package from")
    #     return
    # time.sleep(2)
    # Look at right side of tray
    aim_oakd(yaw=90, pitch=105) # can best pitch
    eyes.set(0,0)
    eyes.setAngleOffset(-70, -40)
    _deliveree = person
    _package = package
    _spoken_package = package
    loc, dist, closeEnough = where_am_i()
    _all_loaded = False
    speak("Please place the " + _spoken_package + "on my tray and say, 'Orange, all loaded' when done.")
    timeout = time.monotonic() + 30
    while not _all_loaded and time.monotonic() < timeout:
        time.sleep(1)
    if not _all_loaded:
        speak("Ok, Fine. Forget it then.")
        return
    #objLabel = waitForObjectOnTray()
    #if objLabel is not None:
    #    _package = objLabel
    # aim oakd up to for detecting a person
    aim_oakd(yaw=oakd._YAW_HOME_, pitch=80)
    eyes.set(0,0)
    eyes.setAngleOffset(90, -50)
    print("delivering ", _package, " to ", _deliveree, " in the ", room)
    speak("Ok, I will take the " + _spoken_package + " to " + _deliveree 
          + ((" at " + room) if room is not None else ""))
    # if in another room set the first goal for the room
    if room is not None and (room != loc or not closeEnough):
        _goal_queue.append("deliver")
        _goal_queue.append("bar") # return to bar
        _goal = room
    else: # in the same room, just deliver to person
        _goal = "deliver"
        
    
###############################################################
# Speech recognition using Google over internet connection
def listen():
    import speech_recognition as sr
    global _run_flag, _goal
    global _internet, _use_internet, _hotword, _google_mode
    
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
    else:
        r = None
        
    speak("Hello, My name is Orange. Pleased to be at your service.")

    def handle_response(phrase, check_hot_word = True):
        global _run_flag, _goal, _listen_flag, _last_phrase, _motion_flag
        global _thought, _person, _new_person_flag, _mood, _time
        global _request, _action_flag, _internet, _use_internet
        global _eyes_flag, _sdp, _hotword, _sub_goal, _all_loaded, _locations

        # convert phrase to lower case for comparison
        phrase = phrase.lower()
        if phrase == "stop" or phrase == "orange stop":
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
        
        if phrase.startswith("find"):
            obj_p, _, loc = phrase.partition("find")[2].partition("in the")[0:3]
            obj = obj_p.split()[-1]
            loc = loc.strip()
            inThisRoom = loc == "room" or loc ==''
            if inThisRoom:
                _sdp.wakeup()
            response = "Ok. I'll search for " + obj_p 
            if len(loc):
                response += " in the " + loc
            speak(response)

            if inThisRoom:
                time.sleep(3)
                lps = _sdp.getLaserScan()
                longest_dist = 0
                longest_angle = 0
                for i in range(0, lps.size):
                    if lps.distance[i] > longest_dist:
                        longest_dist = lps.distance[i]
                        longest_angle = lps.angle[i]

                longest_dist -= 0.75
                longest_dist = max(longest_dist, 0)
                print("other side of room: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
                pose = _sdp.pose()
                xt = pose.x + longest_dist * math.cos(math.radians(pose.yaw) + longest_angle)
                yt = pose.y + longest_dist * math.sin(math.radians(pose.yaw) + longest_angle)
                _locations["custom"] = (xt, yt)
                _sdp.setSpeed(1)
                _goal = "custom"
            else:
                _goal = loc
            _sub_goal = obj
            return HandleResponseResult.Handled

        if "go across the room and come back" in phrase:
            speak("Ok. I'm going across the room and coming back.")
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
            longest_dist = max(longest_dist, 0)
            print("other side of room: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
            pose = _sdp.pose()
            xt = pose.x + longest_dist * math.cos(math.radians(pose.yaw) + longest_angle)
            yt = pose.y + longest_dist * math.sin(math.radians(pose.yaw) + longest_angle)
            _locations["custom"] = (xt, yt)
            _locations["origin"] = (pose.x, pose.y)
            _goal_queue.append("origin")
            _goal = "custom"
            return HandleResponseResult.Handled

        if phrase == "go recharge" or phrase == "go to dock":
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
            pose = _sdp.pose()
            xt = pose.x + dist * math.cos(math.radians(pose.yaw + phi))
            yt = pose.y + dist * math.sin(math.radians(pose.yaw + phi))
            _locations["custom"] = (xt, yt)
            _goal = "custom"
            return HandleResponseResult.Handled
        
        if phrase == "wake up":
            _sdp.wakeup()
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
            Thread(target = eyes.start, name = "Eyes").start()
            return HandleResponseResult.Handled
        
        if phrase == "close your eyes":
            _eyes_flag = False
            eyes.shutdown()
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

        if phrase == "all loaded":
            _all_loaded = True
            return HandleResponseResult.Handled
        
        if phrase == "all taken":
            _all_loaded = False
            return HandleResponseResult.Handled
        
        new_name = ""
        if phrase.startswith("i am"):
            new_name = phrase.partition("i am")[2]
        elif phrase.startswith("i'm"):
            new_name = phrase.partition("i'm")[2]
        elif phrase.startswith("my name is"):
            new_name = phrase.partition("my name is")[2]
        
        if len(new_name) > 0:
            oakd.setPitch(70)
            eyes.setAngleOffset(90, -50)
            speak("Hello " + new_name + ". It's nice to meet you.")
            shutdown_my_depthai()
            speak("Wait while I try to commit your face to memory. Please, only you.")
            start_facial_recog(new_name=new_name)
            time.sleep(2)
            speak("ok. Next time try saying, 'Orange, hello' to check my memory.")
            shutdown_facial_recog()
            start_depthai_thread()
            eyes.setHome()
            oakd.allHome()
            return HandleResponseResult.Handled

        if phrase.startswith("hello") or phrase.startswith("hi"):
            oakd.setPitch(70)
            eyes.setAngleOffset(90, -50)
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
                    ided = True
                    break
            shutdown_facial_recog()
            start_depthai_thread()
            eyes.setHome()
            oakd.allHome()
            if not ided:
                speak("I don't believe we have met before. Try telling me your name.")
            return HandleResponseResult.Handled
        
        if phrase.startswith("update location of"):
            loc = phrase[19:]
            pose = _sdp.pose()
            _locations[loc] = (pose.x, pose.y, math.radians(pose.yaw))
            print("updated location", loc, "to (", pose.x, pose.y, pose.yaw, ")")
            speak("I updated location " + loc)
            save_locations()
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
            room_words = room.split()
            if len(room_words) > 1:
                try:
                    room = " ".join([room_words[0], str(w2n.word_to_num(room_words[1]))])
                except:
                    None             

            # run this in a separate thread so we can take voice answers
            Thread(target=deliverToPersonInRoom, args=(person, package, room), name="deliverToPersonInRoom").start()
            return HandleResponseResult.Handled
            #if math.abs(p[0].theta) > 2):
            #    turn(p[0].theta)
            
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
        
    def listenFromGoogleSpeechRecog(r, sr):
        global _goal, _internet
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
        except Exception as e:
            print(e)
            return ""
        
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
            speak("I lost my internet connection.")            
        except:
            phrase = ""
            print("Unknown speech recognition error.")
        return phrase
        
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

    def listenFromGoogle(finallyFunc=lambda:None, check_hot_word=True):
        try:
            phrase = listenFromGoogleSpeechRecog(r, sr)
            handled_result = handle_response(phrase, check_hot_word)
            if handled_result == HandleResponseResult.NotHandledUnknown:
                words = phrase.split()
                if words[0].lower().startswith(_hotword):
                    words[0] = ''
                phrase = " ".join(words)
                sendToGoogleAssistant(phrase)
        except:
            speak("sorry, i could not do what you wanted.")
        finally:
            finallyFunc()

    def local_hotword_recog_cb(phrase, listener, hotword, r, sr):
        phrase = phrase.lower()
        print("I heard: %s" % phrase)
        if phrase == "stop" or "what's your name" in phrase or "what is your name" in phrase \
            or "who are you" in phrase:
            listener.set_active(False)
            try:
                handle_response(phrase)
            except:
                speak("sorry, i could not do what you wanted.")
            finally:
                listener.set_active(True)
            return
        if hotword in phrase:
            listener.set_active(False)
            speak("yes?")
            listenFromGoogle(lambda:listener.set_active(True), False)

    def local_speech_recog_cb(phrase, listener, hotword, r, sr):
        global _internet, _use_internet
        print("I heard: %s" % phrase)
        phrase = phrase.lower()
        listener.set_active(False)
        if phrase == "orange ask google":
            try:
                if _use_internet and _internet:
                    speak("Go ahead")
                    phrase = listenFromGoogleSpeechRecog(r, sr)
                    sendToGoogleAssistant(phrase)
                else:
                    speak("ask google is not available.")
            finally:
                listener.set_active(True)
            return
#        try:
        handled_result = handle_response(phrase)
        if handled_result == HandleResponseResult.NotHandledUnknown:
            speak("I am not sure how to help with that.")
        elif handled_result == HandleResponseResult.NotHandledNoHotWord:
            print("No hot word, ignoring.")
#        except:
#           speak("sorry, i could not do what you wanted.")
#        finally:
        listener.set_active(True)

    while _run_flag:
        local_listener = None
        use_local_speech = not _use_internet or not _internet or not _google_mode
        try:
            if use_local_speech:
                print("local listener")
                # if no internet access or google mode is inactive, use WSR / SAPI
                # to recognize a command subset
                local_listener = winspeech.listen_for(None, "speech.xml", 
                "RobotCommands", lambda phrase, listener, hotword=_hotword, r=r,
                sr=sr: local_speech_recog_cb(phrase, listener, hotword, r, sr))
            else: 
                # use google cloud speech
                listenFromGoogle()

                # winspeech to detect hotword or stop and then invoke google 
                # cloud speech
                # print("detecting hotword")
                # local_listener = winspeech.listen_for(None, "hotword.xml", 
                # "RobotHotword", lambda phrase, listener, hotword=_hotword, r=r, 
                # sr=sr: local_hotword_recog_cb(phrase, listener, hotword, r, sr))
        except Exception as e:
            print(e)

        while _run_flag and local_listener is not None:
            time.sleep(2)

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
        oakd.setYaw(yaw)
    if pitch is not None:
        oakd.setPitch(pitch)

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

    _facial_recog_thread = Thread(target=_facial_recog.run)
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
    
def start_depthai_thread():
    global _my_depthai_thread
    _my_depthai_thread = Thread(target = my_depthai.startUp, args=(_show_rgb_window, _show_depth_window), name="my_depthai")
    _my_depthai_thread.start()

def shutdown_my_depthai():
    global _my_depthai_thread
    try:
        my_depthai.shutdown()
        _my_depthai_thread.join()
    except:
        None

################################################################   
# This is where data gets initialized from information stored on disk
# and threads get started
def initialize_robot():
    global _moods, _people, _ser6, _internet, _eyes_flag, _facial_recog
    
    # speak("Initializing data from memory.")
    
    # with open('moods_data_file') as json_file:  
    #     _moods = json.load(json_file)

    # with open('people_data_file') as json_file:  
    #     _people = json.load(json_file)
        
    # _ser6 = serial.Serial('COM6', 9600, timeout=1.0)
    
    _internet = True

    start_depthai_thread()
    
    Thread(target = listen, name = "Listen").start()
    
    Thread(target = time_update, name = "Time").start()
        
#    Thread(target = monitor_motion, name = "People Motion Monitor").start()
    
#    Thread(target = behaviors, name = "Behaviors").start()
    if _slamtec_on:    
        Thread(target = handleGotoLocation, name = "Handle Goto Location").start()
        Thread(target = batteryMonitor, name = "Battery monitor").start()
#    Thread(target = actions, name = "Actions").start()
    
#    Thread(target = robotMoving, name = "Is Robot Moving").start()
    
#    Thread(target = megaCOM7, name = "megaCOM7").start()
    
#    Thread(target = eyes, name = "Display eyes").start()
    
    #Thread(target = display, name = "Display").start()

    if _eyes_flag: # this is needed to start the display and to keep it open
        Thread(target = eyes.start, name = "Eyes").start()    
    
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
    
    cancelAction()
    _run_flag = False

    eyes.shutdown()
    shutdown_my_depthai()
    shutdown_facial_recog()
    oakd.shutdown()

    # TBD join threads
    
    # Close all open threads
    #while (threading.active_count() > 1):
    #    print("\nClosing... ")
    #    pretty_print_threads()
    #    time.sleep(1)

    # Close the serial port
#    _ser6.close()
   
#    playsound("C:/Users/bjwei/wav/R2D2.wav")
    #_sdp.disconnect()
    del _sdp
    print("\nDone!")
   
################################################################
# This initializes and runs the robot
def robot():
    global _run_flag, _ser6
    
    initialize_robot()
    eyes.setAngleOffset(90, -10)

    try:
        while _run_flag:
            time.sleep(1)
    except KeyboardInterrupt:
        pretty_print_threads()

    save_locations()
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
    
def save_locations():
    with open("locations.pkl", "wb"):
        pickle.dump(_locations, f)

def load_locations():
    global _locations
    with open("locations.pkl", "rb") as f:
        _locations = pickle.load(f)

def run():
    global _sdp, _slamtec_on
    # Start 32 bit bridge server
    _sdp = MyClient()

    init_local_speech_rec()
    initialize_speech()
    #init_camera()
    load_locations()

    speak("I'm starting up.")
    oakd.initialize()

    if _execute:
        res = connectToSdp()

        if (res == 0):
            _slamtec_on = True
            print("I'm now connected to Slamtec. Now loading the map.")
            loadMap()
            # set not to update map by default.
            _sdp.setUpdate(False)
            None
        else:
            speak("Something is wrong. I could not connect to Slamtec.")
    
        robot()
    else:
        print("Program is in DEBUG mode.")
        #initialize_robot()
         
if __name__ == '__main__':
    run()