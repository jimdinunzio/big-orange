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
import typing
from speaker_pixel_ring import SpeakerPixelRing
from chatbot_socket_client import ChatbotSocketClient
from orange_openai_chatbot import OrangeOpenAiChatbot
from orange_textgen_chatbot import OrangeTextGenChatbot
from aws_mqtt_listener import AwsMqttListener
import asyncio
import my_depthai
from robo_gripper import RoboGripper
from button_pad import Button4Pad
from my_sdp_client import MyClient
from my_sdp_server import *
from pyFirmata.pyfirmata import util as pyfirmata_util, Pin
import facial_recognize as fr
import re
from cmd_embed_mgr import CmdEmbedMgr
import pyautogui
from my_langgraph import RobotPlannerGraph

# Constants
_show_rgb_window = False
_show_depth_window = False
_default_map_name = 'office'
_current_map_name = ''
_hotword = "orange"
_google_mode = False
_execute = True # False for debugging, must be True to run as: >python main.py
_run_flag = True # setting this to false kills all threads for shut down
_eyes_flag = False # should eyes be displayed or not
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
_WALKED_AWAY_RESPONSES = [
    "Maybe I can help.",
    "I can be quite entertaining.",
    "I take a host of commands."]
_LOCATION_RECTS = { "kitchen": _KITCHEN_RECT, "office": _OFFICE_RECT, "dining area": _DINING_AREA_RECT}
_dai_fps = 20 # depthai approx. FPS (adjust lower to conserve CPU usage)
_dai_fps_recip = 1.0 / _dai_fps
_movement_timeout = 60
_movement_towards_away = 30
_chatbot_server_ip_addr = "192.168.1.41"
_chatbot_port = 5124
_sonar_grasp_offset = -0.063 # distance to back of grasper
_maps_dir = "maps"
_sounds_dir = "sounds"
_closest_cmd_dist_thresh = 0.2
_closest_cmd_dist_low_conf = 0.13

MicArray = typing.NewType("MicArray", object)

# Globals
_sdp : MyClient = None
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
_use_internet = False # If False don't use internet
_call_out_objects = False # call out objects along route
_user_set_speed = 2
_error_last_goto = False
_deliveree = ""
_package = ""
_package_ontray = False
_spoken_package = ""
_response_num = random.randint(0,2)
_all_loaded = False
_facial_recog : fr.FacialRecognize = None
_facial_recog_thread = None
_my_depthai_thread = None
_listen_thread = None
_eyes_thread = None
_handle_resp_thread = None
_radar_thread = None
_button_pad_thread = None
_handling_resp = False
_handling_resp_lock = Lock()
_sendToGoogleAssistantFn = None
_restart_flag = False
_set_energy_threshold = None
_get_energy_threshold = None
_last_speech_heard = ""
_locations = {}
_mdai : my_depthai.MyDepthAI = None
_move_oak_d = None
_mic_array : MicArray = None
_lpArduino = None
_pixel_ring:SpeakerPixelRing = None
_starting_up = True
_radar = None
_enable_movement_sensing = False
_grasper : RoboGripper = None
_enable_grasper = True
_blazepose_thread = None
_chatbot_socket = ChatbotSocketClient(_chatbot_server_ip_addr, _chatbot_port)
_chatbot_openai : OrangeOpenAiChatbot = OrangeOpenAiChatbot()
_chatbot_textgen : OrangeTextGenChatbot = None
_aws_mqtt_listener = AwsMqttListener()
_aws_mqtt_listener_thread = None
_enable_aws_mqtt_listener = False
_button_pad = Button4Pad()
_grasper_sonar : Pin
_last_grasper_sonar : float = 4.50
_cmdEmbedMgr : CmdEmbedMgr = None
_map_proc : subprocess.Popen = None
_langgraph : RobotPlannerGraph

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
import cv2
# import ai_vision.detect as detect
# import ai_vision.classify as classify
#import winspeech
from enum import Enum
from latte_panda_arduino import LattePandaArduino
import move_oak_d
import eyes
import pickle
import speech_recognition as sr
import socket
from mic_array_tuning import Tuning
import usb.core
import usb.util
import radar
import human_pose as hp
import numpy as np

_move_oak_d : move_oak_d.MoveOakD
_hp : hp.MyBlazePose = None

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

#import logging
import json
#import click
# import google.auth.transport.grpc
# import google.auth.transport.requests
# import google.oauth2.credentials

# from google.assistant.embedded.v1alpha2 import (
#     embedded_assistant_pb2,
#     embedded_assistant_pb2_grpc
# )


###############################################################
# Movement releated
    
def getMoveActionStatus(sdp):
    status = sdp.getMoveActionStatus()
    if status == ActionStatus.Finished:
        print("move action finished.")
    elif status == ActionStatus.Error:
        errStr = sdp.getMoveActionError()
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
def turn(degrees, sdp=None):
    global _action_flag
    if sdp is None:
        sdp = _sdp
    if degrees == 0:
        return
    # if already in action, ignore this
    if _action_flag:
        return

    print("rotating ", degrees, "degrees")
    _action_flag = True # The robot is being commanded to move
    sdp.rotate(math.radians(degrees))

    result = sdp.waitUntilMoveActionDone()
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
# if the robot is within half a meter of the goal, then success:
# return the closest location and the distance to it and if it is close enough

def where_am_i(sdp=None):
    if sdp is None:
        sdp = _sdp
    try:
        pose = sdp.pose()
    except:
        return "unknown", 0, False
    location, distance = nearest_location(pose.x, pose.y)
    print("I am at location = ", location, " distance = ", distance)
    if (distance <= 1.0):
        closeEnough = True
    else:
        closeEnough = False
    return location, distance, closeEnough

def is_close_to(loc, max_dist=1.0, max_heading_diff=math.radians(15)):
    if _locations.get(loc) is None:
        print("is_close_to: location not found: ", loc)
        return -1
    try:
        pose = _sdp.pose()
    except:
        print("is_close_to: error getting pose.")
        return -1
    dist = distance_A_to_B(pose.x, pose.y, _locations[loc][0], _locations[loc][1])
    heading_diff = 0 if len(_locations[loc]) < 3 else _locations[loc][2] - math.radians(pose.yaw)

    return dist <= max_dist and heading_diff <= max_heading_diff

################################################################
# This cancels an ongoing action - which may be a goto or something else.

def cancelAction(interrupt = False, sdp=None):
    global _action_flag, _interrupt_action
    if sdp is None:
        sdp = _sdp
    if interrupt:
        _interrupt_action = True
    for attempt in range(3):
        try:
            sdp.cancelMoveAction()
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
        # if batteryPercent <= 15:
        #     if not _reported_15:
        #         _reported_15 = True
        #         speak(person + ", my battery is exhausted, and I am shutting down now.") 
        #         cancelAction()
        #         _run_flag = False
        #         os.system("shutdown /s /t 30")
        # elif batteryPercent <= 18:
        #     if not _reported_18:
        #         _reported_18 = True
        #         speak(person + ", I need to recharge my battery. I am going to the recharge station.")
        #         _goal = "recharge"
        if batteryPercent <= 25:
            if not _reported_25:
                _reported_25 = True
                #speak(person + ", my battery is getting low. I'll have to charge up soon.")
                _pixel_ring.setPaletteRed()
        elif batteryPercent <= 35:
            if not _reported_35:
                _reported_35 = True
                _pixel_ring.setPaletteYellow()
        else:
            _pixel_ring.setPaletteDefault()
    except:
        None

def getGrasperSonar() -> float:
    global _last_grasper_sonar
    duration = _grasper_sonar.ping()
    if duration:
        _last_grasper_sonar = pyfirmata_util.ping_time_to_distance(duration)
    return _last_grasper_sonar

def getGraspDist() -> float:
    return getGrasperSonar() + _sonar_grasp_offset

def testGraspDist():
    dist = getGraspDist()
    for i in range(0,10):
        dist = getGraspDist()
        print("dist = {} cm".format(dist * 100))
        time.sleep(.5)

def moveToFloatWithYawCapt(sdp, xt, yt, yaw):
    global _action_flag
    sdp.moveToFloatWithYaw(xt, yt, yaw)
    closest_angle = yaw
    shortest_dist = 4.5
    _action_flag = True
    while True:
        dist, angle, done = checkIfDoneMovingOrCaptured(sdp)
        if dist < shortest_dist:
            shortest_dist = dist
            closest_angle = angle
        if done:
            break
        time.sleep(0.05)

    _action_flag = False
    return shortest_dist, closest_angle

def moveForward(sdp, dist):
    print("move forward {} cm".format(dist * 100))
    pose = sdp.pose()
    xt = pose.x + dist * math.cos(math.radians(pose.yaw))
    yt = pose.y + dist * math.sin(math.radians(pose.yaw))
    sdp.moveToFloatWithYaw(xt, yt, math.radians(pose.yaw))

def sonarSweep(sdp, angle, min_dist, min_angle):
    pose = sdp.pose()
    sdp.moveToFloatWithYaw(pose.x, pose.y, math.radians(angle))
    while True:
        maStatus = sdp.getMoveActionStatus()
        if maStatus == ActionStatus.Stopped or \
            maStatus == ActionStatus.Error or \
            maStatus == ActionStatus.Finished:
            break
        dist = getGraspDist()
        cur_angle = sdp.heading()
        if dist < min_dist:
            min_angle = cur_angle            
            min_dist = dist
            print("min_dist = {} cm, min_angle = {} deg.".format(min_dist * 100.0, min_angle))
        time.sleep(0.1)
    return min_dist, min_angle
    
def sonarSweepStop(sdp, angle, min_dist, min_angle, dist_thresh):
    pose = sdp.pose()
    last_dist = min_dist
    sdp.moveToFloatWithYaw(pose.x, pose.y, math.radians(angle))
    stopped = False
    while True:
        maStatus = sdp.getMoveActionStatus()
        if maStatus == ActionStatus.Stopped or \
            maStatus == ActionStatus.Error or \
            maStatus == ActionStatus.Finished:
            break
        dist = getGraspDist()
        #print("sonar dist = {}".format(dist))
        cur_angle = sdp.heading()
        if dist <= dist_thresh:# and dist - last_dist < -0.10:
            stopped = True
            min_angle = cur_angle            
            min_dist = dist
            sdp.cancelMoveAction()
            print("min_dist = {} cm, min_angle = {} deg.".format(min_dist * 100.0, min_angle))
            time.sleep(0.2)
            break
        time.sleep(0.1)
    return min_dist, min_angle, stopped

def rotateToPrecise(sdp, angle):
    pose = sdp.pose()
    sdp.moveToFloatWithYaw(pose.x, pose.y, math.radians(angle))
    sdp.waitUntilMoveActionDone()

def sonarSweepSearchForObject(sdp):
    min_dist = getGraspDist()
    min_angle = sdp.heading()
    pose = sdp.pose()

    print("sonar sweep")
    min_dist, min_angle = sonarSweep(sdp, pose.yaw + 20, min_dist, min_angle)
    if _interrupt_action:
        return 0,0
    min_dist, min_angle = sonarSweep(sdp, pose.yaw - 20, min_dist, min_angle)
    if _interrupt_action:
        return 0,0
    return min_dist, min_angle

def centerObjWithSonar(sdp, dist_thresh):
    orig_yaw = sdp.pose().yaw
    for i in range(3):
        rotateToPrecise(sdp, orig_yaw)
        if _interrupt_action:
            return 0
        min_dist, min_angle = sonarSweepSearchForObject(sdp)
        dist = min_dist
        if min_dist > dist_thresh:
            continue
        print("turning to min_angle {}".format(min_angle))
        rotateToPrecise(sdp, min_angle)
        dist = getGraspDist()
        print("now dist = {} cm".format(dist*100))
        for i in range(3):
            if _interrupt_action:
                return 0
            if dist > min_dist + .02:
                print("readjusting angle")
                rotateToPrecise(sdp, min_angle)
                dist = getGraspDist()
            else:
                break
        print("final dist = {} cm".format(dist*100))
        return dist
    rotateToPrecise(sdp, orig_yaw)
    return dist

def sonarSweepFindObjAndStop(sdp, dir, spread_angle, dist_thresh):
    min_dist = getGraspDist()
    min_angle = sdp.heading()
    pose = sdp.pose()

    spread_angle = dir * spread_angle

    print("sonar sweep")
    min_dist, min_angle, stopped = sonarSweepStop(sdp, pose.yaw + spread_angle, min_dist, min_angle, dist_thresh)
    if _interrupt_action:
        return 0,0
    if not stopped:
        min_dist, min_angle, _ = sonarSweepStop(sdp, pose.yaw - spread_angle, min_dist, min_angle, dist_thresh)
    if _interrupt_action:
        return 0,0
    return min_dist, min_angle

def findObjWithSonar(sdp, dist_thresh):
    orig_yaw = sdp.pose().yaw
    spread_angle = 20
    for i in range(3):
        dir = random.randint(0,1)
        if dir == 0:
            dir = -1
        rotateToPrecise(sdp, orig_yaw)
        if _interrupt_action:
            return 0
        min_dist, min_angle = sonarSweepFindObjAndStop(sdp, dir, spread_angle, dist_thresh)
        #input("sweep stopped. press key to continue.")
        time.sleep(0.5)
        if min_dist > dist_thresh:
            spread_angle += 5
            print("min_dist > dist_thresh, new spread angle = {}".format(spread_angle))
            continue
        dist = getGraspDist()
        print("now dist = {} cm".format(dist*100))
        if dist - min_dist > .02:
            turnImm(sdp, -dir)
        return getGraspDist()
    return getGraspDist()

def checkIfDoneMovingOrCaptured(sdp):
    dist = getGraspDist()
    angle = sdp.heading()
    print("dist = {} cm".format(dist * 100))

    if dist <= 0.025:
        sdp.cancelMoveAction()
        return dist, angle, True

    maStatus = sdp.getMoveActionStatus()
    if maStatus == ActionStatus.Stopped or \
        maStatus == ActionStatus.Error or \
        maStatus == ActionStatus.Finished:
        return dist, angle, True

    return dist, angle, False

# takes degrees
def rotateTo(angle, sdp):
    sdp.rotateTo(math.radians(angle))
    sdp.waitUntilMoveActionDone()
    
def finalCaptureObject(obj, grasp_hold_angle, sdp : MyClient):
    global _action_flag
    sdp.setSpeed(1)
        
    retries = 3
    # assume it is right in front of grasper

    while retries > 0: 
        if _interrupt_action:
            return False
        dist = getGraspDist()
        print("after moving close, dist = {} cm".format(dist * 100))
        moved_forward = False
        if dist >= 0.025 and dist <= 0.25:
            #input("Press Enter to try capture moving foward")
            moveForward(sdp, dist + 0.08 if dist > .15 else max(dist + 0.08, 0.20))
            moved_forward = True
    
            while True:
                dist, _, done = checkIfDoneMovingOrCaptured(sdp)
                if done:
                    break
                time.sleep(0.05)
        
        dist = getGraspDist()
        if dist > 0.025:
            print("Missed it")
            orig_yaw = sdp.pose().yaw
            dist = findObjWithSonar(sdp, 0.25)
            if dist <= 0.25:
                continue
            rotateToPrecise(sdp, orig_yaw)
			#back up and try again
            if moved_forward:
                backup(sdp, 7)
            retries -= 1
            time.sleep(2)
            if retries == 0:
                return False  
        else:
            print("got it")
            _grasper.setGrasp(grasp_hold_angle)
            time.sleep(0.25)
            _grasper.setWristVertOrient()
            return True
    
def captureObject(obj, sdp):
    if obj == "bottle" or obj == "remote":
        grasp_hold_angle = 70
        _grasper.setWristHorizOrient()
    elif obj == "frisbee":
        grasp_hold_angle = 50
        _grasper.setWristVertOrient()                
    else:
        grasp_hold_angle = 65
        _grasper.setWristHorizOrient()

    _grasper.setGraspFullOpen()

    dist = getGraspDist()
    print("capturing, dist = {} cm".format(dist*100))
    if dist > 0.25:
        retries = 3
        od = 0.15
        while True:
            if _interrupt_action:
                return False
            for i in range(3):
                found, p = checkForObject(obj)
                if not found:
                    time.sleep(0.2)
                    continue
                break
            if found:
                #input("press a key to try moving close to object")
                yaw, xt, yt = getLocationNearObj(sdp, obj, p, cam_yaw=0, offset_dist=od)
                shortest_dist, closest_angle = moveToFloatWithYawCapt(sdp, xt, yt, math.radians(yaw + p.theta))
                print("finished moving closer to object")
                if shortest_dist <= 0.25:
                    print("distance <= 0.25, moving to final capture.")
                    break
                else:
                    found, p = checkForObject(obj)
                    if found:
                        od -= 0.05
                        continue
                    orig_yaw = sdp.pose().yaw
                    shortest_dist = findObjWithSonar(sdp, 0.25)
                    if shortest_dist <= 0.25:
                        break
                    rotateToPrecise(sdp, orig_yaw)
                    if retries <= 0:
                        return False
                    found, p = checkForObject(obj)
                    if found:
                        continue
                    else:
                        #input("still too far and out of sight, press key to back up and try again")
                        # backup and try again
                        backup(sdp, 20)
                        retries -= 1
                        continue
            else:
                print("Lost object")
                # orig_yaw = sdp.pose().yaw
                # shortest_dist = findObjWithSonar(sdp, 0.25)
                # if shortest_dist <= 0.25:
                #     break                
                # rotateToPrecise(sdp, orig_yaw)
                if retries <= 0:
                    return False
                #input("press a key to backup")
                backup(sdp, 20)
                retries -= 1

    return finalCaptureObject(obj, grasp_hold_angle, sdp)    

def getFurthestLaserScanFront(sdp: MyClient):
    lps = sdp.getLaserScan()  
    longest_angle = 0
    longest_dist = 0
    for i in range(0, lps.size):
        if abs(lps.angle[i]) < math.pi / 32.0:
            if lps.distance[i] > longest_dist:
                longest_angle = lps.angle[i]
                longest_dist = lps.distance[i]
    return longest_dist, longest_angle

def getFurthestLaserScan(sdp : MyClient):
    lps = sdp.getLaserScan()
    longest_dist = 0
    longest_angle = 0
    for i in range(0, lps.size):
        if lps.distance[i] > longest_dist:
            longest_dist = lps.distance[i]
            longest_angle = lps.angle[i]
    return longest_dist, longest_angle

def getLocationFromAngleDist(angle, dist, sdp : MyClient):
    pose = sdp.pose()
    xt = pose.x + dist * math.cos(math.radians(pose.yaw) + angle)
    yt = pose.y + dist * math.sin(math.radians(pose.yaw) + angle)
    return xt, yt

def findOrRetrieveObject(loc, obj, op, person, orig_yaw, sdp : MyClient):
    global _goal, _sub_goal
    inThisRoom = loc == "room" or loc ==''
    if inThisRoom:
        sdp.wakeup()
    response = "Ok. I'll search for a " + obj 
    if len(loc):
        response += " in the " + loc
    if op.endswith("_to_me"):
        response += " and bring it back to you, " + person
    elif op.endswith("_to_person"):
        response += " and take it to " + person
    speak(response)

    if inThisRoom:
        #time.sleep(3)
        # turn back to original direction to find destination
        rotateTo(orig_yaw, sdp)            
        time.sleep(3)
        # if op == "retrieve":
        # find furthest point in front of robot
        sdp.getLaserScan()  
        longest_dist, longest_angle = getFurthestLaserScanFront(sdp)
        # else: # op != "retrieve"
        #     # or find the furthest point in the whole scan
        #     longest_dist, longest_angle = getFurthestLaserScan(sdp)

        longest_dist -= 1
        longest_dist = max(longest_dist, 0)

        print("furthest distance: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
        _locations["custom"] = (getLocationFromAngleDist(longest_angle, longest_dist, sdp))
        sdp.setSpeed(1)
        _goal = "custom"
    else:
        _goal = loc
    _sub_goal = obj + ":" + op
    
################################################################
# This is where goto actions are initiated and get carried out.
# If the robot is in the process of going to a  location, but
# another goto action request is receved, the second request will
# replace the earlier request

def handleGotoLocation():
    global _run_flag, _goal, _action_flag, _interrupt_action
    global _deliveree, _package, _sub_goal, _call_out_objects
    global _error_last_goto, _response_num, _locations

    def moveToLocation():
        if len(coords) == 3:
            sdp.moveToFloatWithYaw(coords[0], coords[1], coords[2])
        else:
            sdp.moveToFloat(coords[0], coords[1])

    def setNextGoal():
        global _goal, _deliveree, _package
        _goal = ""
        if len(_goal_queue) > 0:
            _goal = _goal_queue.pop(0)
        else:
            _move_oak_d.allHome()
            eyes.setHome()
        if _goal == "release_obj": # not a location goal but a command token
            speak("Hi " + _deliveree +". I have a " + _package + " for you. I'm releasing it now.")
            _grasper.setWristHorizOrient()
            _grasper.setGraspFullOpen()
            backup(sdp, 5)
            _move_oak_d.allHome()
            eyes.setHome()
            #_grasper.allHome()
            _goal = ""
            _deliveree = ""
            _package = ""
        elif _goal == "went_to_face":
            speak("Hello " + _deliveree)
            _move_oak_d.allHome()
            eyes.setHome()
            _goal = ""
            _deliveree = ""

    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)

    retrieve_to_loc = None
    sub_goal_cleanup = None
    while _run_flag:
        if _goal == "" or _action_flag:
            # no goal or some action is currently in progress, so sleep.
            time.sleep(0.5)
            continue
        if _sub_goal != "":
            part = _sub_goal.partition(":")
            _sub_goal = part[0]
            op = part[2] # empty means find it, retrieve means find it and return it.
            if op == "retrieve": # save location to come back to 
                pose = sdp.pose()
                retrieve_to_loc = (pose.x, pose.y, math.radians(pose.yaw))
            sub_goal_cleanup = None
        _error_last_goto = False
        
        # if finding face, extract command. Default is just find them.
        if _goal.startswith("find_face"):
            face_cmd = _goal.partition(":")[2]
            _goal = "find_face"

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
                if not setDeliverToPersonAsGoal(sdp):
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
                setNextGoal()
                continue
            if _goal != "custom" and _goal != "deliver" and _goal != "person" and _goal != "find_face":
                if is_close_to(_goal, 0.5) and _goal != sub_goal_cleanup:
                    speak("I'm already at the " + _goal)
                    setNextGoal()
                    _move_oak_d.allHome()
                    continue
                if _goal != sub_goal_cleanup and len(_goal_queue) == 0 or len(_goal_queue) > 0 and _goal_queue[0] != "deliver":
                    speak("I'm going to " + _goal)
            elif _goal == "deliver" and _package_ontray:
                speak("hello " + _deliveree)
            else:
                speak("OK.")
            _action_flag = True
            moveToLocation()

        _interrupt_action = False
        sleepTime = 0.5 if (_sub_goal == "" or _call_out_objects) else 0
        if _goal == "find_face":
            aim_oakd(pitch=75) # aim up to see people better                
            eyes.setTargetPitchYaw(-70, 0)
        if _sub_goal != "":
            checkPersons = _sub_goal == 'person'
            if checkPersons:
                aim_oakd(pitch=75) # aim up to see people better                
                eyes.setTargetPitchYaw(-70, 0)
            else:
                aim_oakd(pitch=125) # aim down towards floor for objects
                eyes.setTargetPitchYaw(70, 0)
        else:
            checkPersons = _goal != "deliver" # avoid saying there's a person in the way going to a person
        checkObjects = not checkPersons        
        sub_goal_just_found = False
        face_just_found = False

        idx = 0
        objDict = {}
            
        def gotLock():
            sdp.cancelMoveAction()
            print("got a lock on ", _sub_goal)
            _goal_queue.append(_sub_goal)
            speak("I see a " + _sub_goal)
            _move_oak_d.yawHome()
            #eyes.setHome()

        def gotFace():
            sdp.cancelMoveAction()
            print("got a lock on ", _deliveree)
            _goal_queue.append(_deliveree)
            speak("I see " + _deliveree)
            _move_oak_d.yawHome()
            if face_cmd == "deliver":
                _goal_queue.append("release_obj")
            else:
                _goal_queue.append("went_to_face")
            #eyes.setHome()

        def getOffsetDist(op):
            return 1.25 if op.startswith("retrieve") else 0.75

        if _goal == "find_face":
            ret = setFoundFaceAsGoal(_deliveree, offset_dist=1.0, sdp=sdp)
            if ret == True:
                gotFace()
                face_just_found = True
            else:
                _move_oak_d.startSweepingBackAndForth(0)                

        if _sub_goal != "":
            if setFoundObjAsGoal(_sub_goal, cam_yaw=0, offset_dist = getOffsetDist(op), sdp=sdp):
                gotLock()
                sub_goal_just_found = True
                sub_goal_cleanup = _sub_goal
            else:
                _move_oak_d.startSweepingBackAndForth(0)

        while(_run_flag and _interrupt_action == False and not sub_goal_just_found and not face_just_found):
#            try:
            if _sub_goal == "":
                if _goal == "find_face":
                    faceDict = {}    
                    checkForFaces(faceDict, 9, maxValueLen=9)
                    faceDict = computePersistance(faceDict)
                    if len(faceDict) > 0 and next(iter(faceDict)) == _deliveree and faceDict[_deliveree] > 0.05:
                        print("spotted the face, stopping to get a look")
                        sdp.cancelMoveAction()
                        _move_oak_d.stopSweepingBackAndForth()
                        time.sleep(2)
                        ret = setFoundFaceAsGoal(_deliveree, cam_yaw=_move_oak_d.getYaw(), offset_dist=1.0, sdp=sdp)
                        if ret == True:
                            gotFace()
                            face_just_found = True
                            break
                        elif ret == False:
                            print("saw the face but I lost track of it")
                            _move_oak_d.startSweepingBackAndForth(1)
                            while _move_oak_d.isSweeping():
                                ret = setFoundFaceAsGoal(_deliveree, cam_yaw=_move_oak_d.getYaw(), offset_dist=1.0, sdp=sdp)
                                if ret == True:
                                    gotFace()
                                    face_just_found = True
                                    break
                            if face_just_found:
                                break    
                            print("I'll keep going.")
                            _move_oak_d.startSweepingBackAndForth(0)
                            moveToLocation()
                        else: # saw wrong face
                            print("saw the wrong face")
                            _move_oak_d.startSweepingBackAndForth(0)
                            moveToLocation()

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
                    if setFoundObjAsGoal(_sub_goal, cam_yaw=_move_oak_d.getYaw(), offset_dist = getOffsetDist(op), sdp=sdp):
                        gotLock() 
                        sub_goal_just_found = True
                        sub_goal_cleanup = _sub_goal
                        break
                    else:
                        print("saw", _sub_goal, "but lost track of it")
                        #speak("I thought I saw a " + _sub_goal + ". I'll look again.")
                        _move_oak_d.startSweepingBackAndForth(1)
                        while _move_oak_d.isSweeping():
                            if setFoundObjAsGoal(_sub_goal, cam_yaw=_move_oak_d.getYaw(), offset_dist = getOffsetDist(op), sdp=sdp):
                                gotLock()
                                sub_goal_just_found = True
                                sub_goal_cleanup = _sub_goal
                                break
                        if sub_goal_just_found:
                            break    
                        #speak("I'll keep going.")
                        moveToLocation()

            maStatus = getMoveActionStatus(sdp)
            if maStatus == ActionStatus.Stopped or \
                maStatus == ActionStatus.Error or \
                maStatus == ActionStatus.Finished:
                break
#            except:
#                break
        time.sleep(sleepTime)
        #end of while(_run_flag and _interrupt_action == False and not sub_goal_just_found)

        if _move_oak_d.isSweeping():
            _move_oak_d.stopSweepingBackAndForth()
            _move_oak_d.yawHome()

        if _interrupt_action == True:
            _interrupt_action = False
            _move_oak_d.allHome()
            eyes.setHome()
            _goal_queue.clear()

        if not sub_goal_just_found and not face_just_found:
            # reaching this point, the robot first moved, then stopped - so check where it is now
            reached_goal = is_close_to(_goal)

            # and now check to see if it reached the goal
            if (_goal == "deliver" or reached_goal):
                if _goal == "deliver":
                    article = "some" if _package.endswith('s') else "a"
                    aim_oakd(pitch=75) # aim up to see people better                
                    eyes.setTargetPitchYaw(-70, 0)
                    speak(_deliveree + ", I have " + article + " " + _package + " for you.")
                    if _package_ontray:
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
                    if op.startswith("retrieve"):
                        #switch to lower stereo camera
                        _mdai.changeCamera("BOTTOM")
                        _mdai.waitUntilChangeFinished()
                        #looking down for object
                        eyes.setTargetPitchYaw(70, 0)
                        print("now capture the object")
                        speak("Now I'll retrieve it.")
                        #captured = False
                        captured = captureObject(_goal, sdp)
                        if _interrupt_action:
                            _interrupt_action = False
                        #switch back to upper stereo camera
                        _mdai.changeCamera("TOP")
                        _move_oak_d.yawHome()
                        eyes.setHome()                        
                        if not captured:
                            speak("Sorry, I could not get the " + _goal)
                        else:
                            speak("I have the " + _goal)
                            if op == "retrieve_to_me":
                                speak("Now I'll bring it to you.")
                                _package = _goal
                                _locations["deliver"] = _locations[_deliveree]
                                _goal_queue.append("deliver") # deliver it to person in room
                                _goal_queue.append("release_obj")
                            elif op == "retrieve_to_person":
                                speak("Now I'll find " + _deliveree + " and take it to them.")
                                _package = _goal
                                _locations["find_face"] = _locations[_deliveree]
                                del _locations[_deliveree]
                                _goal_queue.append("find_face:deliver") # find_face of person in room
                                shutdown_my_depthai()
                                start_facial_recog(with_spatial=True, with_tracking=False)
                            else:
                                speak("Now I'll bring it back.")
                                _locations["origin"] = retrieve_to_loc
                                _goal_queue.append("origin") # bring back retrieved object
                                _goal_queue.append("release_obj")
                    sdp.setSpeed(_user_set_speed) #restore speed after finding obj
                    _move_oak_d.allHome()
                elif _goal != "person" and _goal != "find_face":
                    speak("I've arrived.")
            else: # (_goal != "deliver" and not reached_goal)
                _error_last_goto = True
                if _goal == "deliver":
                    speak("Sorry, I could not make my delivery")
                elif _goal != "custom":
                    speak("Sorry, I didn't make it to the " + _goal)
                elif _goal != "find_face":
                    speak("Sorry, I didn't make it to where you wanted.")
            if _sub_goal != "":
                speak("and I never found a "+ _sub_goal)
                sdp.setSpeed(_user_set_speed) #restore speed after finding obj
            if _goal == "find_face": # didn't find face, try one last time with 360 rotation
                print("didn't find face while going to goal. Trying 360 rotation")
                for x in range(0,2):
                    p = searchForFace(sdp, _deliveree, True) #bool(random.randint(0,1)
                    if p is not None:
                        break
                if _interrupt_action:
                    _interrupt_action = False
                if p is not None:
                    setLocationOfObj(sdp, _deliveree, p, cam_yaw=_move_oak_d.getYaw(), offset_dist=1.25)
                    gotFace()
                    face_just_found = True
                else:
                    speech = "Sorry, I could not find " + _deliveree
                    if face_cmd == "deliver":
                        speech += " to make my delivery."
                    speak(speech)
                    shutdown_facial_recog()
                    start_depthai_thread()                 
                    sdp.setSpeed(_user_set_speed)                    

        # finally clear temp goals and _action_flags
        if _goal == "custom":
            del _locations["custom"]
        elif _goal == "deliver":
            del _locations["deliver"]
            _deliveree = ""
        elif _goal == sub_goal_cleanup:
            del _locations[_goal]

        if face_just_found:
            shutdown_facial_recog()
            start_depthai_thread()
            del _locations["find_face"]
            sdp.setSpeed(_user_set_speed) #restore speed after finding obj

        _action_flag = False # you've arrived somewhere, so no further action
        _goal = ""
        _sub_goal = ""
        _move_oak_d.stopSweepingBackAndForth()
        setNextGoal()
        time.sleep(0.5)
    sdp.disconnect()
    sdp.shutdown_server32(kill_timeout=1)
    sdp = None
            
def move_through_locations_thread(done_callback=None):
    global _action_flag
    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)

    while(_run_flag and _interrupt_action == False):
        maStatus = getMoveActionStatus(sdp)
        if maStatus == ActionStatus.Stopped or \
            maStatus == ActionStatus.Error or \
            maStatus == ActionStatus.Finished:
            break
        time.sleep(0.5)

    sdp.disconnect()
    sdp.shutdown_server32(kill_timeout=1)
    _action_flag = False
    if done_callback is not None:
        done_callback(maStatus)

def move_through_locations(sdp, locations: list, final_yaw: float, done_callback=None):
    global _action_flag
    """
    Function to move robot along a series of locations.

    Args:
        locations (list): List of dictionaries with 'x' and 'y' values for each locations.
        Maximum number of locations is defined by MAX_NUM_ROBOT_LOCATIONS. anymore will be ignored.

        final_yaw in degrees (float): The desired orientation after reaching the final locations.

    """
    print("Moving through the following locations with yaw {}:".format(final_yaw))
    locs = LOCATIONS()
    print(locations)
    locs.count = min(MAX_NUM_ROBOT_LOCATIONS, len(locations))
    for i in range(0, locs.count):
        locs.values[i].x = locations[i]['x']
        locs.values[i].y = locations[i]['y']

    _action_flag = True
    sdp.moveTosFloatWithYaw(locs, math.radians(final_yaw))
    # start thread to monitor the move action status
    move_thread = threading.Thread(target=move_through_locations_thread, args=(done_callback,))
    move_thread.start()

    
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

def setPixelRingTrace():
    if _starting_up:
        _pixel_ring.setSpin()
    else:
        _pixel_ring.setTrace()

###############################################################
# Speech Related

def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value):
    global _last_phrase, _voice

    try:
        print(phrase)
        _pixel_ring.setSpeak()
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
    global _sdp, _current_map_name
    _sdp.wakeup()
    print("Loading map and its locations")
    filepath = os.path.join(_maps_dir, filename)
    res = _sdp.loadSlamtecMap(str.encode(filepath) + b'.stcm')
    if (res == 0):
        # set update to false because we don't want to change the map when doing a demo with people standing around messing up the map!
        _sdp.setMapUpdate(False)
        speak("Map and locations are loaded. Mapping is off.")
        # speak("Now let me get my bearings.")
        # result = recoverLocalization(_INIT_RECT)
        # if result == False:
        #    speak("I don't appear to be at the map starting location.")
    else:
        speak("Something is wrong. I could not load the map.")
    print("Done loading map")
    load_locations(filepath)
    _current_map_name = filename
    return res

def saveMap(filename):
    global _sdp, _current_map_name
    filepath = os.path.join(_maps_dir, filename)
    print("saving map and its locations")
    res = _sdp.saveSlamtecMap(str.encode(filepath) + b'.stcm')
    if res != 0:
        speak("Something is wrong. I could not save the map.")
    save_locations(filepath)
    _current_map_name = filename
    return res
                
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
    filepath = _sounds_dir + "\/camera-shutter.wav"
    playsound(filepath, block=True)
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
def searchForPerson(sdp, is_clockwise=True):
    global _action_flag, _interrupt_action

    aim_oakd(pitch=75) # aim up to see people better
    eyes.setTargetPitchYaw(-70, 0)

    ps = []
    # First see if person is already in view and if so return
    found, ps = checkForPerson()
    if found:
        return ps
    
    # if no person in view, then slowly rotate 360 degrees and check every so often
    _action_flag = True
    
    oldyaw = sdp.heading() + 360
    yaw = oldyaw
    sweep = 0
    recheck_person = False
    while (sweep < 380 and not _interrupt_action):
        sdp.rotate(-0.1 if is_clockwise else 0.1)
        found, ps = checkForPerson()
        if found:
            recheck_person = True
            break
        yaw = sdp.heading() + 360
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
        deg = 5 if is_clockwise else -5
        for j in range(1,5):
            for i in range(1,5):
                found, ps = checkForPerson()
                if found:
                   break 
            if not found:
                # go back other way
                sdp.rotate(math.radians(deg))
                sdp.waitUntilMoveActionDone()
                deg = -deg
            else:
                break
            
    _action_flag = False
    return ps

# rotate 360 and stop if the person's face is spotted
def searchForFace(sdp, name, is_clockwise=True):
    global _action_flag, _interrupt_action

    aim_oakd(pitch=75) # aim up to see people better
    eyes.setTargetPitchYaw(-70, 0)

    # First see if person is already in view and if so return
    p = findFace(name, 1)
    if p is not None and p is not False:
        return p
    
    # if correct face not in view, then slowly rotate 360 degrees and check every so often
    _action_flag = True
    
    oldyaw = sdp.heading() + 360
    yaw = oldyaw
    sweep = 0
    while sweep < 380:
        p = None
        recheck_face = False
        while (sweep < 380 and not _interrupt_action):
            sdp.rotate(-0.1 if is_clockwise else 0.1)
            ps = _facial_recog.get_detections()
            for face in ps:
                if face.name == name:
                    recheck_face = True
                    break
            if recheck_face:
                sdp.cancelMoveAction()
                break
            yaw = sdp.heading() + 360
            covered = abs(yaw - oldyaw)
            if covered > 180:
                covered = 360 - covered
            sweep += covered
            #print("yaw = ", yaw, " covered = ", covered, " sweep = ", sweep)

            oldyaw = yaw
            time.sleep(0.07)
        
        if recheck_face:
            print("rechecking face")
            sdp.cancelMoveAction()
            time.sleep(0.3)
            deg = 5 if is_clockwise else -5
            for j in range(1,5):
                p = findFace(name, 1)
                if p is not None:
                    break
                else:
                    # go back other way
                    sdp.rotate(math.radians(deg))
                    sdp.waitUntilMoveActionDone()
                    deg = -deg
            if p == False: # wrong face seen so keep going
                continue
        if p is not None:
            break
    _action_flag = False
    return p

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
                closest_z = 999
                for a in ps:
                    if obj == a.label:
                        if p is None or p.z > 0 and p.z < closest_z:
                            p = a
                            closest_z = p.z
                # If bbox ctr of detection is away from edge then stop
                if p is not None and p.bboxCtr[0] >= 0.0 and p.bboxCtr[0] <= 1:
                    print(obj, " at bbox ctr: ",p.bboxCtr[0], ", ", p.bboxCtr[1], " z = ", p.z)
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
                #print("Person at bbox ctr: ",p.bboxCtr[0], ", ", p.bboxCtr[1])
                return True, ps
    except:
        None
    return False, ps

def getLocationNearObj(sdp, obj, p, cam_yaw=0, offset_dist=0.75):
    p.z -= offset_dist # come up to the object within certain distance
    if p.z < 0.0:
        p.z = 0.0
    pose = sdp.pose()
    xt = pose.x + p.z * math.cos(math.radians(pose.yaw + cam_yaw + p.theta))
    yt = pose.y + p.z * math.sin(math.radians(pose.yaw + cam_yaw + p.theta))
    print("location near ", obj, " is at distance ", p.z, " meters at ", cam_yaw + p.theta, "degrees")
    return pose.yaw, xt, yt

def setLocationOfObj(sdp, obj, p, cam_yaw=0, offset_dist=0.75):
    yaw, xt, yt = getLocationNearObj(sdp, obj,p, cam_yaw, offset_dist)
    _locations[obj] = (xt, yt, math.radians(yaw + cam_yaw + p.theta))

def setFoundObjAsGoal(obj, cam_yaw=0, offset_dist=0.75, sdp=None):
    if sdp is None:
        sdp = _sdp
    found, p = checkForObject(obj)
    if found:
        setLocationOfObj(sdp, obj, p, cam_yaw, offset_dist)
        return True
    return False

def findObjAndSetGoal(sdp, obj, goal, cam_yaw=0):
    found, p = checkForObject(obj)
    if found:
        setLocationOfObj(sdp, goal, p, cam_yaw)
        return True
    return False

def setDeliverToPersonAsGoal(sdp):
    global _interrupt_action, _goal

    if findObjAndSetGoal(sdp, "person", "deliver"):
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
        ps = searchForPerson(sdp, bool(random.randint(0,1)))
        if _interrupt_action:
            _interrupt_action = False
            return False
        if len(ps):
            # take first person for now, later check gender/age match
            p = ps[0]
            setLocationOfObj(sdp, "deliver", p)
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

def findFace(name, numSecs=2):
    faceDict = {}
    print("find a face")
    spatial_dict = checkForFaces(faceDict, numSecs * _dai_fps, maxValueLen= numSecs * _dai_fps)
    faceDict = computePersistance(faceDict)
    #print(faceDict)
    if faceDict.get(name,0) > 0.15:
        return spatial_dict.get(name)
    # else if the wrong face is detected, then return False
    elif len(faceDict) > 0 and next(iter(faceDict.values())) > 0.15:
        return False
    return None

def computePersistance(objDict):
    def by_value(item):
        return item[1]
    persistObjDict = {}
    for obj in objDict:
        persistObjDict[obj] = sum(objDict[obj]) / len(objDict[obj])
    return {k: persistObjDict[k] for k,v in sorted(persistObjDict.items(), reverse=True, key=by_value)}

def checkForFaces(faceDict, numChecks, needCentered=False, 
                  maxValueLen=2*_dai_fps, idx=0):
    lastValueLen = 0
    if len(faceDict) > 0:
        lastValueLen = len(faceDict[next(iter(faceDict))])
        if lastValueLen < maxValueLen:
            idx = lastValueLen
    spatial_dict = {}
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
        # try:
        ps = _facial_recog.get_detections()

        for face in ps:
            if not faceDict.get(face.name):
                faceDict[face.name] = [False] * (min(lastValueLen + i + 1, maxValueLen))
            faceDict[face.name][idx] = True
            if face.x != 0 or face.y != 0 or face.z !=0:
                spatial_dict[face.name] = face
        # except:
        #     None
        #     if i + 1 < numChecks:
        #         time.sleep(_dai_fps_recip)
        idx += 1
        if i + 1 < numChecks:
            time.sleep(_dai_fps_recip)
    return spatial_dict

def setFoundFaceAsGoal(name, cam_yaw=0, offset_dist=1, sdp=None):
    if sdp is None:
        sdp = _sdp
    p = findFace(name, 1)
    if p is not None and p is not False:
        setLocationOfObj(sdp, name, p, cam_yaw, offset_dist)
        return True
    else:
        if p is False:
            return -1
        else:
            return False

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
    global _deliveree, _package, _package_ontray, _spoken_package, _goal, _goal_queue, _all_loaded
    #go to person to pick up item 
    speak("Ok. I'll come get it.")
    # aim up to see people better
    aim_oakd(pitch=75) 
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
    _package_ontray = True
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

def deliverObjToPerson(package, deliveree, room):
    global _deliveree, _package, _goal, _goal_queue

    _deliveree = deliveree if deliveree != "me" else _person
    _package = package
    loc, _, closeEnough = where_am_i()
    
    _goal_queue.append("deliver")
    # if in another room set the first goal for the room
    if room is not None and (room != loc or not closeEnough):
        _goal = room
    else: # in the same room, go to room center first
        _goal = loc

def findPerson(person, doa, sdp):
    global _deliveree
    # look towards sound of voice and if person spotted, record location
    yawDelta = _mic_array.rotateToDoa(doa, sdp)
    
    # Remove deliveree from locations
    if _locations.get(person):
        _locations.pop(person)
    # find person / deliveree
    ps = searchForPerson(sdp, yawDelta > 0)
    if len(ps) > 0:
        z = 9999.0
        # find closest person
        for p in ps:
            if p.z < z:
                z = p.z

        setLocationOfObj(sdp, person, p, cam_yaw=0, offset_dist=1.0)
    else:
        # if no person found then come back to this loc to look after retrieving object
        return_to_loc, _, _ = where_am_i()
        _locations[person] = return_to_loc 
    _deliveree = person
    return person

def come_here(doa):
    global _goal, _interrupt_action

    sdp = MyClient()
    sdp_comm.connectToSdp(sdp)
    
    yawDelta = _mic_array.rotateToDoa(doa, sdp)

    ps = searchForPerson(sdp, yawDelta > 0)
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

def forward(sdp, n=5):
    for i in range(0,n):
        move_imm(sdp, 1)
        time.sleep(0.1)

def backup(sdp, n=5):
    for i in range(0,n):
        move_imm(sdp, -1)
        time.sleep(0.1)

def turnImm(sdp, dir, n=3):
    for i in range(0,n):
        turn_imm(sdp, dir)
        time.sleep(0.1)

def set_handling_response(value):
    global _handling_resp
    with _handling_resp_lock:
        _handling_resp = value

def handling_response():
    with _handling_resp_lock:
        return _handling_resp or _handle_resp_thread is not None and _handle_resp_thread.is_alive()

def handle_response_sync(sdp, phrase, doa, check_hot_word = True, assist = False, listenResponseFn=None):
    if handling_response():
        print("already handling response, try again later.")
        return HandleResponseResult.NotHandledBusy
    set_handling_response(True)
    try:
        handled_result = handle_response(sdp, phrase, doa, check_hot_word, listenResponseFn=listenResponseFn)
        if handled_result == HandleResponseResult.NotHandledUnknown:
            # if assist:
            #     _sendToGoogleAssistantFn(phrase.split(_hotword)[-1])
            # else:
            speak("Sorry, I don't understand \"" + phrase.split(_hotword)[-1] + "\"?")
    finally:
        set_handling_response(False)

def handle_response_async(sdp, phrase, doa, check_hot_word = True):
    # issue the command in its own thread
    _handle_resp_thread = Thread(target = handle_response_sync, args=(sdp, phrase, doa, check_hot_word), name = "handle_response_async", daemon=False)
    _handle_resp_thread.start()



###############################################################
# Command Handler
def handle_response(sdp, phrase, doa, check_hot_word = True, listenResponseFn : typing.Union[typing.Callable[[object, int], str], None] = None):
    global _run_flag, _goal, _listen_flag, _last_phrase
    global _person, _mood, _time
    global _action_flag, _internet, _use_internet
    global _eyes_flag, _hotword, _sub_goal, _all_loaded
    global _chatbot_openai, _chatbot_textgen, _deliveree, _map_proc

    class ImageCallback:
        def __init__(self):
            self._image = None
    
        def get_picture_cb(self, frame):
            # resize the image's larger dimension to 512 pixels while keeping the aspect ratio
            if frame.shape[0] > frame.shape[1]:
                frame = cv2.resize(frame, (int(512 * frame.shape[1] / frame.shape[0]), 512))
            else:
                frame = cv2.resize(frame, (512, int(512 * frame.shape[0] / frame.shape[1])))
            _, self._image = cv2.imencode(".jpg", frame)

        def get_image(self):
            return self._image

    tried_closest_cmd = False

    # max of twice through this loop. if first time command is not recognized then closest command is tried
    while True:
        # convert phrase to lower case for comparison
        phrase = phrase.lower().strip() 

        if "stop moving" in phrase or "stop motors" in phrase or "stop stop" in phrase:
            cancelAction(True, sdp)
            speak("Stopping.")
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

        # check if the hot word is in the string, and take the words after it, otherwise ignore speech
        # if check_hot_word and not tried_closest_cmd:
        #     hot_word_idx = phrase.rfind(_hotword)
        #     if hot_word_idx >= 0:
        #         phrase = phrase[hot_word_idx + len(_hotword):].strip()
        #         print("cmd extracted: ", phrase)
        #     else:
        #         return HandleResponseResult.NotHandledNoHotWord

        # some verbal commands are handled inside the listen thread
        if phrase == "":
            speak("That's my name. Don't wear it out...")
            return HandleResponseResult.Handled

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
            speak("Okay, I will stop listening.")
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
                if len(location) > 0 and location != "unknown":
                    answer = "I'm closest to the " + location
                else:
                    answer = "I don't know where I am."
                speak(answer)
                return HandleResponseResult.Handled
            answer = "I'm at the " + location + " location."
            speak(answer)
            return HandleResponseResult.Handled
                    
        if phrase == "where are you going?":
            if _action_flag == True and _goal != "":
                speak("I'm going to the " + _goal)
            else:
                speak("I'm happy where I am at the moment.")
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

        if re.match(r"^list (people|persons|faces) you know", phrase):
            names = fr.get_known_faces()
            print(names)
            return HandleResponseResult.Handled

        # parse var
        if re.match(r"^(find|look for) (mr|mister|ms|miss)", phrase):
            # name = 3rd word until end of phrase
            name = phrase.split()[2:]
            name = ' '.join(name)

            names = fr.get_known_faces()
            if name not in names:
                speak("sorry, i have not met " + name + ", and I don't know what they look like.")
                return HandleResponseResult.Handled

            _mic_array.rotateToDoa(doa, sdp)
            sdp.wakeup()
            speak("Ok, i'll look around for " + name)
            shutdown_my_depthai()
            start_facial_recog(with_spatial=True, with_tracking=False)
            # use current location
            _deliveree = name

            longest_dist, longest_angle = getFurthestLaserScanFront(sdp)

            longest_dist -= 1.75
            longest_dist = max(longest_dist, 0)

            sdp.setSpeed(1) #slow speed

            print("furthest distance: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
            _locations["find_face"] = (getLocationFromAngleDist(longest_angle, longest_dist, sdp))
            
            #pose = sdp.pose()
            #_locations["find_face"] = (pose.x, pose.y, math.radians(pose.yaw))
            _goal = "find_face"
            return HandleResponseResult.Handled

        # parse var
        # HBRC Floor Bot Challenge III
        # HBRC Floor Bot Challenge II
        if phrase.startswith("find") or phrase.startswith("retrieve"):
            person = ""
            orig_yaw = sdp.pose().yaw
            phrase = phrase.replace("retrieved", "retrieve")
            op = phrase.split()[0]
            # fix common error of speech recognizer
            phrase = phrase.replace("in bring it to me", "and bring it to me")
            phrase = phrase.replace("in take it to", "and take it to")
            if "and bring it to me" in phrase:
                if "in the" in phrase:
                    obj_p, _, loc = phrase.partition(op)[2].partition("in the")[0:3]
                    loc = loc.split("and bring it to me")[0]
                else:
                    loc = ''
                    obj_p = phrase.partition(op)[2].partition("and bring it to me")[0]
                obj = obj_p.split()[-1]
                op += "_to_me"
                person = findPerson(_person, doa, sdp)
            elif "and take it to" in phrase:
                if "in the" in phrase:
                    obj_p, _, loc = phrase.partition(op)[2].partition("in the")[0:3]
                    loc, person_p = loc.split("and take it to")[0,3]
                else:
                    loc = ''
                    obj_p, _, person_p= phrase.partition(op)[2].partition("and take it to")[0:3]
                obj = obj_p.split()[-1]
                person = person_p.split()[-1]
                names = fr.get_known_faces()
                if person not in names:
                    speak("sorry, i have not met " + person + ", and I don't know what they look like.")
                    return HandleResponseResult.Handled
                op += "_to_person"
                findPerson(person, doa, sdp)
            else: # not "bring it to me" in phrase
                obj_p, _, loc = phrase.partition(op)[2].partition("in the")[0:3]
                obj = obj_p.split()[-1]
            loc = loc.strip()

            findOrRetrieveObject(loc, obj, op, person, orig_yaw, sdp)
            return HandleResponseResult.Handled

        # HBRC Floor Bot Challenge I
        if "go across the room and come back" in phrase:
            speak("Ok. I'm going across the room and coming back.")
            sdp.wakeup()
            time.sleep(6)
            longest_dist, longest_angle = getFurthestLaserScan(sdp)
            longest_dist -= 1.75
            longest_dist = max(longest_dist, 0)
            print("other side of room: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)        
            pose = sdp.pose()
            _locations["custom"] = (getLocationFromAngleDist(longest_angle, longest_dist, sdp))
            _locations["origin"] = (pose.x, pose.y)
            _goal_queue.append("origin")
            _goal = "custom"
            return HandleResponseResult.Handled

        if phrase == "go recharge" or phrase == "go to your dock":
            cancelAction(True, sdp)
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

        if phrase.startswith("backup") or phrase.startswith("back up"):
            backup(sdp)
            return HandleResponseResult.Handled

        # parse var
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
            elif unit == "ft" or unit == "feet":
                dist /= 3.281
            else:
                print("unknown unit")
                return HandleResponseResult.NotHandledUnknown
            
            #if unit not mentioned assume meters
            pose = sdp.pose()
            xt = pose.x + dist * math.cos(math.radians(pose.yaw + phi))
            yt = pose.y + dist * math.sin(math.radians(pose.yaw + phi))
            
            print("going to ", xt ,", ", yt)

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

        # parse var
        if "go to" in phrase:
            words = phrase.split()
            try:
                words.remove("the")
            except:
                None
            if len(words) > 2:
                cancelAction(True, sdp)
                _goal = " ".join(words[2:]).lower()
            return HandleResponseResult.Handled
                
        # parse var
        if phrase.startswith("you are in the"):
            loc = phrase.partition("you are in the")[2].strip()
            locRect = _LOCATION_RECTS.get(loc)
            if locRect is not None:
                speak("Ok. I will relocate myself in the map. Please wait.")
                recoverLocalization(locRect)
            else:
                speak("I'm sorry. I don't have that location's area.")
            return HandleResponseResult.Handled
                        
        if phrase == "recover localization" or phrase == "locate yourself":
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
        
        if phrase == "open your eyes" or phrase == "show your eyes":
            if _eyes_flag == True:
                return HandleResponseResult.Handled
            _eyes_flag = True
            start_eyes_thread()
            return HandleResponseResult.Handled
        
        if phrase == "close your eyes" or phrase == "hide your eyes":
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

        if phrase == "shut down system" or phrase == "shutdown system":
            speak("Okay, I'm shutting down the system.")
            _run_flag = False
            os.system("shutdown /s /t 10")
            return HandleResponseResult.Handled
                
        if "battery" in phrase or "voltage" in phrase:
            ans = "My battery is currently at "
            ans = ans + str(sdp.battery()) + " percent"
            speak(ans)
            return HandleResponseResult.Handled
                
        # parse var
        if phrase.startswith("load map"):
            name = phrase[9:]
            if len(name) == 0:
                name = _current_map_name
                if len(name) == 0:
                    name = _default_map_name
            speak("Ok. I will load map " + name)
            loadMap(name)
            return HandleResponseResult.Handled
        
        # parse var
        if phrase.startswith("save map"):
            name = phrase[9:]
            if len(name) == 0:
                name = _current_map_name
                if len(name) == 0:
                    speak("please include the name of the map")
                    return HandleResponseResult.Handled
            speak("Ok. I will save map " + name)
            saveMap(name)
            return HandleResponseResult.Handled
        
        if "clear map" in phrase:
            speak("Ok. I will clear my map.")
            sdp.clearSlamtecMap()
            # after clearing make sure updating is on
            sdp.setMapUpdate(True)
            return HandleResponseResult.Handled
        
        if "clear locations" in phrase:
            speak("Ok. I will clear the locations.")
            _locations.clear()
            return HandleResponseResult.Handled

        if "enable mapping" in phrase:
            speak("Ok. I will enable map updating.")
            sdp.setMapUpdate(True)
            return HandleResponseResult.Handled                

        if "disable mapping" in phrase:
            speak("Ok. I will disable map updating.")
            sdp.setMapUpdate(False)
            return HandleResponseResult.Handled

        if "show map" in phrase:
            # launch robostudio
            path = os.path.join(os.path.abspath('../../../DLLs/RoboStudio_2.1.1_rtm'), "RoboStudio.exe")
            # Launch RoboStudio and keep track of the process so it can be killed later
            _map_proc = subprocess.Popen([path, "--autofollow", "--fps", "10", "192.168.11.1"])
            # To kill later, call: proc.terminate() or proc.kill()
            time.sleep(1.5)
            coordinates = [
                (159, 59),
                (13, 152),
                (127, 206),
                (183, 314),
                (243, 110)
            ]

            # Click each coordinate
            for x, y in coordinates:
                pyautogui.click(x=x, y=y)
                time.sleep(0.25)
            return HandleResponseResult.Handled

        if "hide map" in phrase:
            if _map_proc is not None:
                _map_proc.terminate()
                _map_proc = None
            return HandleResponseResult.Handled
        
        if "take a picture" in phrase:
            if not _mdai.rgbWindowVisible():
                speak("I have to open the RGB window first. Hold on.")
                _mdai.showRgbWindow(True)
                _mdai.waitUntilChangeFinished()
                # wait for camera exposure to adjust
                time.sleep(1.5)
                speak("Ok. Taking picture.")
            _mdai.takePicture()
            time.sleep(0.5)
            speak("Ok. Here is the picture I took.")
            return HandleResponseResult.Handled

        if "take my picture" in phrase:
            if not _mdai.rgbWindowVisible():
                speak("I have to open the RGB window first.")
                _mdai.showRgbWindow(True)
                _mdai.waitUntilChangeFinished()
                # wait for camera exposure to adjust
                time.sleep(0.5)
            _mic_array.rotateToDoa(doa, sdp)
            search_dir = -1 # assume we have to raise camera to get person in frame
            timeout = time.monotonic() + 10
            timed_out = False
            while True:
                found, ps = checkForPerson()
                if found:
                    upper_body_y = (ps[0].ymax - ps[0].ymin) * 0.75 + ps[0].ymin
                    #print("upper body y = ", upper_body_y, ", ymin = ", ps[0].ymin)
                    if ps[0].ymin > 0.05 and upper_body_y >= 0.7 and upper_body_y <= 0.8:
                        break

                if time.monotonic() >= timeout:
                    timed_out = True
                    break

                _move_oak_d.offsetPitch(search_dir)
                pitch = _move_oak_d.getPitch()
                if pitch <= -60:
                    search_dir = 1
                elif pitch >= -10:
                    search_dir = -1
                time.sleep(0.050)            

            if timed_out:
                speak("Sorry, I could not find your face.")
            else:
                time.sleep(0.5)
                _mdai.takePicture()
                time.sleep(0.5)
                speak("Ok. Here is the picture I took.")
    
            _move_oak_d.allHome()
            return HandleResponseResult.Handled

        # parse var
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
        
        # if "you see" in phrase:
        #     results = detect.detect_objects(top_count=3)
        #     print(results)
        #     if results is not None:
        #         res_count = len(results) 
        #         if res_count > 0 and results[0].percent >= 40:
        #             reply_str = "I see a " + results[0].label
        #             if res_count == 2 and results[1].percent >= 40:
        #                 reply_str += " and a " + results[1].label
        #             elif res_count > 2:
        #                 for i in range(1, res_count - 1):
        #                     if results[i].percent >= 40:
        #                         reply_str += ", a " +results[i].label
        #                 if results[res_count - 1].percent >= 40:
        #                     reply_str += " and a " + results[res_count - 1].label
        #         else:
        #             reply_str = "I don't see anything I recognize."
        #         speak(reply_str)
        #     return HandleResponseResult.Handled
        
        # if "identify this" in phrase:
        #     model = classify.ModelType.General
        #     if "bird" in phrase:
        #         model = classify.ModelType.Birds
        #     elif "insect" in phrase:
        #         model = classify.ModelType.Insects
        #     elif "plant" in phrase:
        #         model = classify.ModelType.Plants
        #     results = classify.classify(model)
        #     print(results)
        #     if len(results) > 0 and results[0].percent > 40 and "background" not in results[0]:
        #         speak("it looks like a " + results[0].label)
        #     else:
        #         speak("Sorry, I do not know what it is.")
        #     return HandleResponseResult.Handled
        
        if "close pictures" in phrase:
            _mdai.closePictures()
            return HandleResponseResult.Handled

        if phrase == "all loaded":
            _all_loaded = True
            return HandleResponseResult.Handled
        
        if phrase == "all taken":
            _all_loaded = False
            return HandleResponseResult.Handled

        if phrase == "open depth window":
            speak("Ok.")
            _mdai.showDepthWindow(True)
            return HandleResponseResult.Handled

        if phrase == "close depth window":
            speak("Ok.")
            _mdai.showDepthWindow(False)
            return HandleResponseResult.Handled
            
        if phrase == "open rgb window":
            speak("Ok.")
            _mdai.showRgbWindow(True)
            return HandleResponseResult.Handled
        
        if phrase == "close rgb window":
            speak("Ok.")
            _mdai.showRgbWindow(False)        
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

        if phrase == "look over here":
            _mic_array.rotateToDoa(doa, sdp)
            return HandleResponseResult.Handled

        if "never mind" in phrase or "nevermind" in phrase or "forget it" in phrase:
            speak("ok.")
            return HandleResponseResult.Handled

        if phrase == "dance with me":
            speak("Ok. Let me spin up a tune.")
            _mic_array.rotateToDoa(doa, sdp)
            eyes.setTargetPitchYaw(-70, 0)
            _move_oak_d.setPitch(70) # pitch up to see person better
            speak("Ok. Let's dance.")
            _sdp.setSpeed(3)
            timeout = time.monotonic() + 30
            dir = random.randint(0,1)
            if dir == 0:
                dir = -1
            orig_yaw = sdp.pose().yaw
            spread_angle = 30
            # play an mp3 music file file.mp3
            playsound("C:/Users/LattePanda/Music/07 If This is It.wav", block=False)
            timeout = time.monotonic() + 30

            while time.monotonic() < timeout:
                turnImm(sdp, dir, 20)
                turnImm(sdp, -dir, 20)
                #rotateToPrecise(sdp, orig_yaw + spread_angle)
                #rotateToPrecise(sdp, orig_yaw - spread_angle)
                
            speak("ok. i'm tired and need to rest a minute. Thank you.")
            _sdp.setSpeed(_user_set_speed)
            _move_oak_d.allHome()
            eyes.setHome()
            return HandleResponseResult.Handled
        
        if phrase == "go there" or phrase == "go where i am pointing":
            speak("Ok. Let me look where you are pointing.", tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value)
            _mic_array.rotateToDoa(doa, sdp)
            shutdown_my_depthai()
            start_blazepose_thread()

            while not _hp.get_is_running():
                time.sleep(0.050)

            pose = sdp.pose()
            loc = _hp.get_target()
            
            search_dir = -1 # assume we have to raise camera to get person in frame
            timeout = time.monotonic() + 10
            timed_out = False
            while True:
                loc = _hp.get_target()

                if loc is not None:
                    top_points = _hp.get_rect_points()
                    #print("top_points = ", top_points)
                    tl = top_points[0][1]
                    tr = top_points[1][1]
                    head_visible = tl > -75 and tr > -75 and tl < 25 and tr < 25

                    score = _hp.get_lm_score()
                    #print("score = ", score)
                    #print("head_visible = ", head_visible)
                    if time.monotonic() >= timeout:
                        timed_out = True
                    if timed_out or (score > 0.95 and head_visible):
                        break

                    if tl <= -75 or tr <= -75: 
                        search_dir = -1
                    elif tl >= 25 or tr >= 25:
                        search_dir = 1
                    else:
                        search_dir = 0

                _move_oak_d.offsetPitch(search_dir)
                pitch = _move_oak_d.getPitch()
                if pitch <= -60:
                    search_dir = 1
                elif pitch >= -10:
                    search_dir = -1
                time.sleep(0.050)            
            time.sleep(0.5)
            
            if timed_out:
                speak("Sorry I don't know where you want me to go.")
            else:
                print("GOT a detection")
                if loc[1] == -1: # pointing up too high
                    rnd = random.randint(0,2)
                    if rnd == 0:
                        speak("Uh, I'll need a pair of wings to go there.")
                    elif rnd == 1:
                        speak("You'll need to call NASA for my jet pack.")
                    else:
                        speak("If only I could fly like a drone.")
                    _move_oak_d.allHome()
                    shutdown_blazepose_thread()
                    start_depthai_thread()
                    return HandleResponseResult.Handled                

                def safe_asin(x):
                    return math.asin(max(-1.0, min(1.0, x)))
            
                person = _hp.get_person_loc()
                x_cam = loc[0]
                z_cam = loc[2]
                print("x_cam = {}, z_cam = {}".format(x_cam, z_cam))

                theta = -safe_asin(x_cam/z_cam) if z_cam != 0.0 else 0
                cam_yaw = _move_oak_d.getYaw()
                xt_w = pose.x + z_cam * math.cos(math.radians(pose.yaw + cam_yaw) + theta)
                yt_w = pose.y + z_cam * math.sin(math.radians(pose.yaw + cam_yaw) + theta)

                px_cam = person[0]
                pz_cam = person[2]
                print("px_cam = {}, pz_cam = {}".format(px_cam, pz_cam))

                pTheta = -safe_asin(px_cam/pz_cam) if pz_cam != 0.0 else 0
                px_w = pose.x + pz_cam * math.cos(math.radians(pose.yaw + cam_yaw) + pTheta)
                py_w = pose.y + pz_cam * math.sin(math.radians(pose.yaw + cam_yaw) + pTheta)

                look_at_v = np.array([px_w - xt_w, py_w - yt_w])
                look_at_v = look_at_v / np.linalg.norm(look_at_v)

                la_heading = math.atan2(look_at_v[1], look_at_v[0])
                
                speak("I am going", tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value)
                time.sleep(.5)
                print("going to location (", xt_w, ", ", yt_w, " @ heading ", math.degrees(la_heading))
                _move_oak_d.allHome()
                _locations["custom"] = (float(xt_w), float(yt_w), float(la_heading))
                _goal = "custom"
            shutdown_blazepose_thread()
            start_depthai_thread()
            return HandleResponseResult.Handled

        # parse var
        new_name = ""
        if phrase.startswith("i am"):
            new_name = phrase.partition("i am")[2]
        elif phrase.startswith("i'm"):
            new_name = phrase.partition("i'm")[2]
        elif phrase.startswith("my name is"):
            new_name = phrase.partition("my name is")[2]

        if len(new_name) > 0:
            _mic_array.rotateToDoa(doa, sdp)
            speak("hello, ", new_name)
            _person = new_name
            #_move_oak_d.setPitch(70) # pitch up to see person better
            eyes.setTargetPitchYaw(-70, 0)
            speak("Hello " + new_name + ". It's nice to meet you.")
            shutdown_my_depthai()
            speak("Wait while I try to commit your face to memory. Please, only you.")
            start_facial_recog(new_name=new_name)
            while not _facial_recog.was_face_added():
                time.sleep(0.2)
            speak("ok. Next time try saying, 'Orange, hello' to check my memory.")
            shutdown_facial_recog()
            start_depthai_thread()
            eyes.setHome()
            _move_oak_d.allHome()
            return HandleResponseResult.Handled

        if phrase.startswith("hello") or phrase.startswith("nice to meet you"):
            _mic_array.rotateToDoa(doa, sdp)
            #_move_oak_d.setPitch(70) # pitch up to see person better
            eyes.setTargetPitchYaw(-70, 0)
            shutdown_my_depthai()
            speak("Hello There.", tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value)
            start_facial_recog()
            ided = False
            end = time.monotonic() + 5
            while time.monotonic() < end:
                face = checkForFace(1)   
                if face is not None:
                    eyes.setText(face)
                    speak(face + ", it's nice to see you again.")
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

        # parse var
        if phrase.startswith("delete location"):
            loc = phrase[16:]
            speak("Ok. I will delete location " + loc)
            _locations.pop(loc)
            return HandleResponseResult.Handled

        # parse var
        # regex for update | set | save location of
        if re.match(r"^(update|set|save) location", phrase):
            # loc = 3rd word until end of phrase
            loc = phrase.split()[2:]
            loc = ' '.join(loc)
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

        if "enable radar" in phrase:
            speak("ok, i've enabled radar.")
            start_radar()
            return HandleResponseResult.Handled

        if "disable radar" in phrase:
            speak("ok, i've disabled radar.")
            stop_radar()
            return HandleResponseResult.Handled

        if "open weather chat" in phrase:
            p = os.path.join(os.path.abspath(''),'riva-sample-apps/virtual-assistant')
            os.chdir(p)
            result = os.system('riva_weather')
            os.chdir('..')
            print(result)
            if result != 0:
                speak("Sorry, I could not open the weather chat.")
            return HandleResponseResult.Handled

        if "enable chat bot" in phrase:
            if _chatbot_openai is None:
                _chatbot_openai = OrangeOpenAiChatbot()
                speak("chat cloud enabled.")
            #speak(_chatbot_openai.intro_line)
            return HandleResponseResult.Handled

        if "disable chat bot" in phrase:
            _chatbot_openai = None
            speak("chatbot disabled.")
            return HandleResponseResult.Handled
        
        # if "open chat" in phrase:
        #     _chatbot_textgen = OrangeTextGenChatbot()
        #     speak(_chatbot_textgen.intro_line)
        #     return HandleResponseResult.Handled

        # if "open chat local" in phrase:
        #     conn = _chatbot_socket.connect()
        #     intro = ""
        #     if conn:
        #         intro = _chatbot_socket.get_response()
        #     if _chatbot_socket.is_connected():
        #         speak(intro)
        #     else:
        #         speak("sorry, i'm unable to open a chat right now.")
        #     return HandleResponseResult.Handled
            
        # parse var    
        if phrase.startswith("et = "):
            global _set_energy_threshold
            if _set_energy_threshold is not None:
                _set_energy_threshold(w2n.word_to_num(phrase.split()[2]))
                return HandleResponseResult.Handled
            else:
                return HandleResponseResult.NotHandledUnknown

        # if phrase.startswith("ask google"):
        #     try:
        #         question = phrase.partition("ask google")[2]
        #         _sendToGoogleAssistantFn(question)
        #     except:
        #         return HandleResponseResult.NotHandledUnknown
        #     return HandleResponseResult.Handled
        
        # parse var
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
                # room_words = room.split()
                # if len(room_words) > 1:
                #     try:
                #         room = " ".join([room_words[0], str(w2n.word_to_num(room_words[1]))])
                #     except:
                #         None             

                room = room.strip()
            
                if room in _locations:
                    # run this in a separate thread so we can take voice answers
                    Thread(target=deliverToPersonInRoom, args=(person.strip(), package.strip(), room), name="deliverToPersonInRoom", daemon=False).start()
                else:
                    speak("Sorry, I don't know how to get to " + room + ".")
            else: # room is None i.e. same room
                Thread(target=deliverToPersonInRoom, args=(person.strip(), package.strip(), room), name="deliverToPersonInRoom", daemon=False).start()            
            return HandleResponseResult.Handled
            
        # parse var
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
                if split_len >= 4 and "counter" not in phrase_split[3]:
                    deg = -deg
            except:
                if phrase_split[1] == "around":
                    deg = 180
                else:
                    None
            turn(deg)
            return HandleResponseResult.Handled

        # if chatbot command is recognized then skip to chatbot
        if _chatbot_openai and (phrase == "reset chat" or phrase == "show chat log" or "you see" in phrase 
                                or "describe this" in phrase or "identify this" in phrase or "what is this" in phrase):
            break

        if tried_closest_cmd:
            print("error - closest command did not parse. check parser and command list")
            return HandleResponseResult.NotHandledUnknown

        print("parser failed, check if langgraph system tool would apply")
        #if unrecognized movment command, try the Langgraph system with tools
        if "move " in phrase or "dr " in phrase or "drive " in phrase:
            _langgraph.send_input(phrase)
            return HandleResponseResult.Handled
        
        print("parser failed, looking up closest command")
        # if not understood, try to match the command using the embeddings manager
        closest_command, dist = _cmdEmbedMgr.find_closest_command(phrase)
        print("closest command is '{}' with distance: {}", closest_command, dist)
        low_conf = dist < _closest_cmd_dist_thresh and dist > _closest_cmd_dist_low_conf
        if dist >= _closest_cmd_dist_thresh:
            closest_command = None
        elif low_conf:
            #ask user if correct
            speak("Did you mean, "+ closest_command + "?")
            if listenResponseFn is not None:
                response = listenResponseFn(sdp, 5)
                if response != "yes":
                    return HandleResponseResult.NotHandledUnknown
            
        tried_closest_cmd = True
        if closest_command is not None:
            phrase = closest_command
            if not low_conf:
                speak("I assume you meant " + phrase + ".")
            print("trying again with closest command \"{}\", dist = {}".format(phrase, dist))
        else:
            break
        # end of parse while(true)

    # if not handled by old school parsing send it to the chatbot
    if _chatbot_openai:
        print("sending speech to chatbot")
        image = None
        if len(phrase) > 0:
            if phrase == "reset chat":
                _chatbot_openai.init_chat_log()
                speak("chatbot reset.")
                return HandleResponseResult.Handled
            
            elif phrase == "show chat log":
                print(_chatbot_openai.get_log())
                return HandleResponseResult.Handled

            if "you see" in phrase or "describe this" in phrase or "identify this" in phrase or "what is this" in phrase:
                imageCallback = ImageCallback()
                _mdai.setGetPictureCb(imageCallback.get_picture_cb)
                timeout = time.monotonic() + 5
                while imageCallback.get_image() is None and time.monotonic() < timeout:
                    time.sleep(0.1)
                if imageCallback.get_image() is None:
                    speak("I'm sorry, I can't see anything.")
                    return HandleResponseResult.Handled
                image = imageCallback.get_image()

            print(f"Human: {phrase}")
            try:
                response = _chatbot_openai.get_response(phrase, image)
            except Exception as e:
                print("Error in getting response: ", e)
                response = "Sorry, I could not get a response."

            if len(response) > 0:
                print(f"Orange: {response}")
                speak(response)
                _chatbot_openai.add_to_chat_log(response)
            else:
                speak("I got nothing on that.")

    # send text to chatbot and get response if connected
    # if _chatbot_socket.is_connected():
    #     print("handling chat speech")
    #     if len(phrase) > 0:
    #         speak_response = True

    #         if phrase == "reset chat":
    #             phrase = ".reset"
    #         elif phrase == "restart chat":
    #             phrase = ".restart"
    #             speak_response = False
    #         elif phrase == "show log":
    #             phrase = ".log"
    #             speak_response = False
        
    #         if phrase[0] != '.':
    #             print(f"Human: {phrase}")
            
    #         result = _chatbot_socket.send_msg(phrase)
    #         if result:
    #             response = _chatbot_socket.get_response()
    #             if len(response) > 0:
    #                 if speak_response:
    #                     print("Orange: ", end='')
    #                     speak(response)

    #                 print(response)
    #             else:
    #                 speak("I got nothing on that.")
    #         else:
    #             speak("I'm sorry, I can't continue the chat at the moment.")

    #         if "goodbye" in phrase.lower() or phrase == ".restart":
    #             _chatbot_socket.close()
    #             speak("chat has ended.")
        
    #     return HandleResponseResult.Handled
    # if _chatbot_textgen:
    #     print("handling textgen chat speech")
    #     if len(phrase) > 0:
    #         if phrase == "reset chat":
    #             _chatbot_textgen.init_chat_log()
    #             speak("chatbot reset.")
    #             return HandleResponseResult.Handled
            
    #         elif phrase == "show log":
    #             print(_chatbot_textgen.get_log())
    #             return HandleResponseResult.Handled
        
    #         print(f"Human: {phrase}")

    #         async def speak_response(inp):
    #             response = ""
    #             async for sent in _chatbot_textgen.get_response_stream(inp):
    #                 if len(sent) > 0:
    #                     speak(sent)
    #                     response += sent
    #             return response

    #         response = asyncio.run(speak_response(phrase))

    #         if len(response) > 0:
    #             print(f"Orange: {response}")
    #         else:
    #             speak("I got nothing on that.")

    #         if "stop chat" in phrase.lower():
    #             _chatbot_textgen = None
    #             speak("chat cloud has ended.")

    #     return HandleResponseResult.Handled      

        
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
    # ASSISTANT_API_ENDPOINT = 'embeddedassistant.googleapis.com'
    # DEFAULT_GRPC_DEADLINE = 60 * 3 + 5
    # PLAYING = embedded_assistant_pb2.ScreenOutConfig.PLAYING

    # api_endpoint = ASSISTANT_API_ENDPOINT
    # credentials = os.path.join(click.get_app_dir('google-oauthlib-tool'), 'credentials.json')
    # device_model_id = "orbital-clarity-197305-marvin-hcmekv"
    # device_id = "orbital-clarity-197305"
    # lang = 'en-US'
    # display = False
    # verbose = False
    # grpc_deadline = DEFAULT_GRPC_DEADLINE
    
    # class TextAssistant(object):
    #     """Text Assistant that supports text based conversations.
    #     Args:
    #       language_code: language for the conversation.
    #       device_model_id: identifier of the device model.
    #       device_id: identifier of the registered device instance.
    #       display: enable visual display of assistant response.
    #       channel: authorized gRPC channel for connection to the
    #         Google Assistant API.
    #       deadline_sec: gRPC deadline in seconds for Google Assistant API call.
    #     """
        
    #     def __init__(self, language_code, device_model_id, device_id,
    #                  display, channel, deadline_sec):
    #         self.language_code = language_code
    #         self.device_model_id = device_model_id
    #         self.device_id = device_id
    #         self.conversation_state = None
    #         # Force reset of first conversation.
    #         self.is_new_conversation = True
    #         self.display = display
    #         self.assistant = embedded_assistant_pb2_grpc.EmbeddedAssistantStub(channel)
    #         self.deadline = deadline_sec
        
    #     def __enter__(self):
    #         return self
        
    #     def __exit__(self, etype, e, traceback):
    #         if e:
    #             return False
        
    #     def assist(self, text_query):
    #         """Send a text request to the Assistant and playback the response.
    #         """
    #         def iter_assist_requests():
    #             config = embedded_assistant_pb2.AssistConfig(
    #                 audio_out_config=embedded_assistant_pb2.AudioOutConfig(
    #                     encoding='LINEAR16',
    #                     sample_rate_hertz=16000,
    #                     volume_percentage=0,
    #                 ),
    #                 dialog_state_in=embedded_assistant_pb2.DialogStateIn(
    #                     language_code=self.language_code,
    #                     conversation_state=self.conversation_state,
    #                     is_new_conversation=self.is_new_conversation,
    #                 ),
    #                 device_config=embedded_assistant_pb2.DeviceConfig(
    #                     device_id=self.device_id,
    #                     device_model_id=self.device_model_id,
    #                 ),
    #                 text_query=text_query,
    #             )
    #             # Continue current conversation with later requests.
    #             self.is_new_conversation = False
    #             if self.display:
    #                 config.screen_out_config.screen_mode = PLAYING
    #             req = embedded_assistant_pb2.AssistRequest(config=config)
    #             #assistant_helpers.log_assist_request_without_audio(req)
    #             yield req

    #         text_response = None
    #         html_response = None
            
    #         try:
    #             for resp in self.assistant.Assist(iter_assist_requests(),
    #                                               self.deadline):
    #                 #assistant_helpers.log_assist_response_without_audio(resp)
    #                 if resp.screen_out.data:
    #                     html_response = resp.screen_out.data
    #                 if resp.dialog_state_out.conversation_state:
    #                     conversation_state = resp.dialog_state_out.conversation_state
    #                     self.conversation_state = conversation_state
    #                 if resp.dialog_state_out.supplemental_display_text:
    #                     text_response = resp.dialog_state_out.supplemental_display_text
    #         except Exception as e:
    #             print("got error from assistant: "+ str(e))
    #         return text_response, html_response
    
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
            with m as source: r.adjust_for_ambient_noise(source, duration=1, is_speech_cb=_mic_array.getIsSpeech)
        except:
            None
        r.energy_threshold = max(300, r.energy_threshold)
        print("final ambient threshold changed to ", r.energy_threshold)

    # beginning of actual Listen() code - <clean this up!>
    #logging.basicConfig(level=logging.DEBUG if verbose else logging.INFO)
    
    #if _use_internet:
        # # Load OAuth 2.0 credentials.
        # try:
        #     with open(credentials, 'r') as f:
        #         credentials = google.oauth2.credentials.Credentials(token=None, **json.load(f))
        #         http_request = google.auth.transport.requests.Request()
        #         credentials.refresh(http_request)
        # except Exception as e:
        #     logging.error('Error loading credentials: %s', e)
        #     logging.error('Run google-oauthlib-tool to initialize '
        #                     'new OAuth 2.0 credentials.')
        
        # Create an authorized gRPC channel.
        #grpc_channel = google.auth.transport.grpc.secure_authorized_channel(credentials, http_request, api_endpoint)
        #speak("I'm connected to Google Assistant.")
        #logging.info('Connecting to %s', api_endpoint)

    # create a recognizer object
    r : sr.Recognizer = sr.Recognizer()

    # create a microphone object
    mic = sr.Microphone(sample_rate=16000, chunk_size=512)

    def get_energy_threshold():
        return r.energy_threshold

    def set_energy_threshold(value):
        r.energy_threshold = value
        print("energy threshold changed to ", r.energy_threshold)

    adj_spch_recog_ambient(r, mic)

    global _set_energy_threshold
    _set_energy_threshold = set_energy_threshold

    global _get_energy_threshold
    _get_energy_threshold = get_energy_threshold
        
    # prime the Vosk recognizer 
    r.prime_vosk()

    speak("Hello, My name is Orange. Pleased to be at your service.")

    HEY_ORANGE_KEYWORD_IDX = 0
    STOP_NOW_KEYWORD_IDX = 1
    GET_RESPONSE_IDX = 2
    
    def listenFromVoskSpeechRecog(r : sr.Recognizer, mic, sr, porcupine_config : typing.Union[sr.Recognizer.PorcupineListener.Config, None], timeout=None) -> tuple[str, float]:
        global _last_speech_heard
      # obtain audio from the microphone
        try:
            with mic as source:
                print("Say something!")
                if porcupine_config is not None:
                    _pixel_ring.setOff() # turn off from trace mode so wake word volume effect is noticable
                audio = r.listen(source, timeout = timeout, phrase_time_limit = 8, porcupine_config = porcupine_config, is_speech_cb=_mic_array.getIsSpeech)
                doa = _mic_array.getDoa()
                _pixel_ring.setThink()
                print("Your speech ended.")
        except sr.WaitTimeoutError:
            adj_spch_recog_ambient(r, mic)
            return "", 0
        except sr.ReturnAfterKeywordDetection as e:
            phrase = ""
            if e.args[0] == STOP_NOW_KEYWORD_IDX:
                phrase = "stop moving"
            return phrase, 0

        except Exception as e:
            print(e)
            return "", 0

        # recognize speech using Vosk Speech Recognition
        try:
            result = r.recognize_vosk(audio, arg2=None, alts=3)
            print(result)
            phrase = json.loads(result)
            phrase = phrase["alternatives"][0]["text"].strip()          
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
                _pixel_ring.setThink()
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
        
    # def sendToGoogleAssistant(phrase):
    #     global _internet
    #     if _internet:
    #         print("Sending question to google assistant: ", phrase)
    #         with TextAssistant(lang, device_model_id, device_id, display,
    #                 grpc_channel, grpc_deadline) as assistant:
    #             response_text, response_html = assistant.assist(text_query = phrase)
    #             if response_text:
    #                 speak(response_text)
    #             else:
    #                 speak("Sorry, I don't know about that.")
    #     else: 
    #         speak("I am not sure how to help with that.")

    # global _sendToGoogleAssistantFn
    # _sendToGoogleAssistantFn = sendToGoogleAssistant

    def listenFromVoskResponse(sdp, timeout=5):
        on_detection(GET_RESPONSE_IDX)
        try:
            phrase, _ = listenFromVoskSpeechRecog(r, mic, sr, None, timeout=5)
        except:
            speak("sorry, i am having trouble understanding.")
        return phrase

    def listenFromVosk(sdp, porcupine_config : sr.Recognizer.PorcupineListener.Config, finallyFunc=lambda:None, check_hot_word=True):
        try:
            phrase, doa = listenFromVoskSpeechRecog(r, mic, sr, porcupine_config)
            if phrase != "stop moving":
                setPixelRingTrace()
            handle_response_sync(sdp, phrase, doa, check_hot_word, listenResponseFn=listenFromVoskResponse)
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

    # def ask_google():
    #     if _use_internet and _internet:
    #         speak("Go ahead")
    #         phrase, doa = listenFromGoogleSpeechRecog(r, mic, sr)
    #         sendToGoogleAssistant(phrase)
    #     else:
    #         speak("ask google is not available.")

    def local_speech_recog_cb(phrase, listener, hotword, r, mic, sr, sdp):
        global _internet, _use_internet, _last_speech_heard
        doa = _mic_array.getDoa()
        print("I heard: \"%s\" at %d degrees" % (phrase, _sdp.heading() + _mic_array.doa2YawDelta(doa)))
        _last_speech_heard = phrase
        phrase = phrase.lower()
        listener.set_active(False)
        # if phrase == "orange ask google":
        #     try:
        #         ask_google()
        #     finally:
        #         listener.set_active(True)    
        #     return
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
    _pixel_ring.setEndStartup() # restore pixel ring to default sound sensitive mode after boot up

    def on_detection(index):
        if index == HEY_ORANGE_KEYWORD_IDX:
            for i in range(1, 12):
                _pixel_ring.setColoredVolume(i)
                time.sleep(0.005)
        elif index == STOP_NOW_KEYWORD_IDX:
            _pixel_ring.setRedVolume()
        elif index == GET_RESPONSE_IDX:
            for i in range(1, 12):
                _pixel_ring.setBlueVolume(i)
                time.sleep(0.0075)
                
    def on_listen_timeout(index):
        if index == HEY_ORANGE_KEYWORD_IDX:
            for i in range(11, -1, -1):
                _pixel_ring.setColoredVolume(i)
                time.sleep(0.005)

    keyword_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "models", "Hey-Orange_en_windows_v3_0_0.ppn"))
    access_key = os.getenv("PORCUPINE_ACCESS_KEY")
    if access_key is None:
        porcupine_config = None
        print("No Porcupine access key set. Hot word detection will not be available.")
        keyword_path = None
    else:
        from pvporcupine import KEYWORD_PATHS
        porcupine_config = r.PorcupineListener.Config(access_key=access_key, 
                                                      keyword_paths=[keyword_path, KEYWORD_PATHS['grapefruit']],
                                                      keyword_types=[r.PorcupineListener.KeywordType.LISTEN, r.PorcupineListener.KeywordType.IMMEDIATE],
                                                      sensitivities=[0.25, 0.5],
                                                      on_detection=on_detection,
                                                      on_det_timeout=on_listen_timeout)

    while _run_flag:
        #local_listener = None
        use_local_speech = not _use_internet or not _internet or not _google_mode
        try:
            if use_local_speech:
                print("local listener")
                # if no internet access or google mode is inactive, use WSR / SAPI
                # to recognize a command subset
                listenFromVosk(sdp, porcupine_config=porcupine_config)
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

def start_button_pad_thread():
    global _button_pad_thread
    if _button_pad_thread is not None:
        return
    _button_pad_thread = Thread(target=_button_pad.startUp, daemon=False)
    _button_pad_thread.start()

def shutdown_button_pad_thread():
    global _button_pad_thread
    try:
        _button_pad.shutdown()
        _button_pad_thread.join()
    except:
        None

def start_facial_recog(with_spatial=False, with_tracking=True, new_name=""):
    global _facial_recog, _facial_recog_thread

    if _facial_recog_thread is not None:
        print("facial recognize thread already running")
        return
    
    if with_tracking:
        getPitch = _move_oak_d.getPitch
        offsetPitch = _move_oak_d.offsetPitch
        getYaw =_move_oak_d.getYaw
        offsetYaw=_move_oak_d.offsetYaw
    else:
        getPitch = None
        offsetPitch = None
        getYaw = None
        offsetYaw = None
              
    if len(new_name) > 0:
        _facial_recog = fr.FacialRecognize(getPitch, offsetPitch, getYaw, offsetYaw,
                                           compute_spatial=with_spatial, add_face=True, debug=True, new_name = new_name)
    else:
        _facial_recog = fr.FacialRecognize(getPitch, offsetPitch, getYaw, offsetYaw,
                                           compute_spatial=with_spatial, debug=True)

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
    _facial_recog_thread = None

def start_blazepose_thread():
    global _blazepose_thread, _hp

    if _blazepose_thread is not None:
        return

    _hp = hp.MyBlazePose(device_id=my_depthai.TOP_MOUNTED_OAK_D_ID)

    _blazepose_thread = Thread(target = _hp.run, name="hp", daemon=False)
    _blazepose_thread.start()

def shutdown_blazepose_thread():
    global _hp, _blazepose_thread
    if _blazepose_thread is None:
        return
    try:
        _hp.shutdown()
        _blazepose_thread.join()
        _blazepose_thread = None
    except:
        None
    del(_hp)
    _hp = None

def start_depthai_thread(model="tinyYolo", use_tracker=False, loc="TOP"):
    global _my_depthai_thread, _mdai

    if _my_depthai_thread is not None:
        return
    _mdai = my_depthai.MyDepthAI(model, use_tracker)

    _my_depthai_thread = Thread(target = _mdai.startUp, args=(loc, _show_rgb_window, _show_depth_window), name="mdai", daemon=False)
    _my_depthai_thread.start()

def shutdown_my_depthai():
    global _my_depthai_thread
    if _my_depthai_thread is None:
        return
    try:
        _mdai.shutdown()
        _my_depthai_thread.join()
        _my_depthai_thread = None
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

def start_tracking(trackTurnBase=True):
    # shutdown current depth ai model
    shutdown_my_depthai()
    # start up the mobilenet with tracker. Yolo doesn't work as well.
    start_depthai_thread(model="mobileNet", use_tracker=True)    
    _move_oak_d.start_tracking(_mdai, trackTurnBase=trackTurnBase)

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
    # initialize
    global _radar_enabled
    next_movement = 0
    next_towards = 0
    next_away = 0
    _radar_enabled = True
    _radar.start_sensing()
    while _radar_enabled:
        if not _action_flag and _radar.has_message():
            msg = _radar.pop_message()
            if msg.report_type == radar.BODYSIGN_OUT:
                if msg.value > 30.0 and time.monotonic() >= next_movement:
                    next_movement = time.monotonic() + _movement_timeout
                    speak("Hello, I noticed you came in the room. I am ready to help.")
            elif msg.value == radar.TOWARDS_AWAY_OUT:
                if time.monotonic() >= next_towards and msg.status == radar.CA_TOWARDS_OUT:
                    speak("I noticed you've approached. What can I do for you?")
                    next_towards = time.monotonic() + _movement_towards_away
                elif time.monotonic() >= next_away and msg.status == radar.CA_AWAY_OUT:
                    r = random.randint(0,2)
                    speak("I noticed you've walked away. " + _WALKED_AWAY_RESPONSES[r])
                    next_away = time.monotonic() + _movement_towards_away
                next_movement = time.monotonic() + _movement_timeout
            print(msg)
        time.sleep(0.25)
    _radar.stop_sensing()

def start_radar():
    global _radar_thread
    if _radar_thread is None:
        _radar_thread = Thread(target=radar_main, name="Radar", daemon=False)
        _radar_thread.start()
    else:
        print("Error - trying to start radar when already started.")

def stop_radar():
    global _radar_thread, _radar_enabled
    _radar_enabled = False
    if _radar_thread is not None:
        _radar_thread.join()
        _radar_thread = None

def start_aws_mqtt_listener():
    global _aws_mqtt_listener_thread
    if _aws_mqtt_listener_thread is None:   
        _aws_mqtt_listener_thread = Thread(target = _aws_mqtt_listener.start, args=(handle_op_request,), name = "AWS MQTT Listener")
        _aws_mqtt_listener_thread.start()

def stop_aws_mqtt_listener():
    global _aws_mqtt_listener_thread
    if _aws_mqtt_listener_thread is not None:
        _aws_mqtt_listener.shutdown()
        _aws_mqtt_listener_thread.join()
        _aws_mqtt_listener_thread = None

from orange_utils import *

def handle_op_request(sdp : MyClient, opType : OrangeOpType, arg1=None, arg2=None):
    global _last_speech_heard, _goal
    if opType == OrangeOpType.TextCommand:
        return handle_response_async(sdp, arg1, 0, check_hot_word=False)
    elif opType == OrangeOpType.BatteryPercent:
        return sdp.battery()
    elif opType == OrangeOpType.Location:
        location, distance, closeEnough = where_am_i(sdp)
        if not closeEnough:
            answer = "near the " + location
        else:
            answer = "at the " + location
        return answer
    elif opType == OrangeOpType.Status:
        batt = sdp.battery()
        loc = handle_op_request(sdp, OrangeOpType.Location)
        return batt, loc
    elif opType == OrangeOpType.GotoCommand:
        location = arg1.strip()
        words = location.split()
        try:
            words.remove("the")
        except:
            None
        location = " ".join(words[0:]).lower()
        print("searching for location \"", location, "\"")
        coords = _locations.get(location)
        if coords is None and location != "recharge":
            return False        
        else:
            cancelAction(interrupt=True, sdp=sdp)
            _goal = location
        return True
    elif opType == OrangeOpType.TakeAPictureCommand:
        _mdai.takePicture()
        return True
    elif opType == OrangeOpType.LastSpeechHeard:
        return _last_speech_heard
    elif opType == OrangeOpType.LastSpeechSpoken:
        return _last_phrase
    elif opType == OrangeOpType.IpAddress:
        # return the ip address of the robot's LAN adapter
        ip = subprocess.check_output(["ipconfig"], text=True).split('\n')
        ip_address = ""
        line_num = 0
        for line_num, line in enumerate(ip):
            if "Wireless LAN adapter Wi-Fi" in line:
                break
        for line in ip[line_num:]:
            if "IPv4 Address" in line:
                ip_address = line.split(':')[1].strip()
                break
        if ip_address == "":
            ip_address = "???"
        return ip_address
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
    elif opType == OrangeOpType.InitiateShutdown:
        global _run_flag
        shutdown_eyes_thread()
        speak("Okay, I'm shutting down.")
        print("\nClosing these active threads:")
        pretty_print_threads()
        # Have them terminate and close
        _run_flag = False
             
################################################################   
# This is where data gets initialized from information stored on disk
# and threads get started
def initialize_robot():
    global _moods, _internet, _eyes_flag, _facial_recog
    global _listen_thread, _cmdEmbedMgr, _langgraph

    _internet = True

    start_button_pad_thread()

    start_depthai_thread()
    #start_blazepose_thread()

    _cmdEmbedMgr = CmdEmbedMgr()
    _cmdEmbedMgr.load_cmds_embeddings()

    _listen_thread = Thread(target = listen, name = "Listen")
    _listen_thread.start()
    
    Thread(target = time_update, name = "Time").start()
            
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

    if _enable_movement_sensing:
        start_radar()

    if _enable_aws_mqtt_listener:
        start_aws_mqtt_listener()
    
    _langgraph = RobotPlannerGraph.create_langgraph(move_through_locations)

################################################################
# This is where data gets saved to disk
# and by setting _run_flag to False, threads are told to terminate
def shutdown_robot():
    global _run_flag, _moods, _sdp, _grasper, _sdp, _lpArduino, _cmdEmbedMgr
    
    cancelAction(True, _sdp)
    _run_flag = False

    print("stop following if doing so")
    stop_following()
    print("shutting down eyes")
    eyes.shutdown()
    print("shutting down depthai")
    shutdown_my_depthai()
    print("shutting down human pose")
    shutdown_blazepose_thread()
    print("shutting down facial recog")
    shutdown_facial_recog()
    print("shutting down move oakd")
    _move_oak_d.shutdown()
    print("shutting down radar")
    stop_radar()
    if _grasper is not None:
        print("shutting down grasper")
        _grasper.shutdown()
        _grasper = None
    print("shutting down aws mqtt listener")
    stop_aws_mqtt_listener()
    print("shutting down button pad")
    shutdown_button_pad_thread()
    print("shutting down microphone array")
    _mic_array.close()
    print("shutting down leonardo")
    _lpArduino.shutdown()
    _lpArduino = None
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

    _cmdEmbedMgr = None
    
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

    def rotateToDoa(self, doa, sdp):
        yawDelta = self.doa2YawDelta(doa)
        print("turning toward where heard person")
        turn(yawDelta, sdp)
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

# ButtonPad Button Handlers
def handleButton1Event(pressed, sdp: MyClient):
    if pressed:
        cancelAction(True, sdp)
        speak("Cancelling Current Action.")

def handleButton2Event(pressed, sdp: MyClient):
    if pressed:
        new_val = not _mdai.rgbWindowVisible()
        text = "Showing" if new_val else "Hiding"
        text += " RGB view."
        speak(text)
        _mdai.showRgbWindow(new_val)

def handleButton3Event(pressed, sdp: MyClient):
    if pressed:
        new_val = not sdp.getMapUpdate()
        sdp.setMapUpdate(new_val)
        text = "Enabling" if new_val else "Disabling"
        text += " map updating."
        speak(text)

def handleButton4Event(pressed, sdp: MyClient):
    if pressed:
        if len(_current_map_name) != 0:
            speak("Ok. I will save map " + _current_map_name)
            saveMap(_current_map_name)
        else:
            speak("please ask me to save map <name>.")

def buttonEventCb(change_mask, button_state_mask, sdp: MyClient):
      for i in range(0, 4):
        if change_mask & 1<<i != 0: 
            pressed = button_state_mask & 1<<i != 0          
            print("button {} {}.".format(i+1, pressed and "pressed." or "released."))
            if i == 0:
                handleButton1Event(pressed, sdp)
            elif i == 1:
                handleButton2Event(pressed, sdp)
            elif i == 2:
                handleButton3Event(pressed, sdp) 
            elif i == 3:
                handleButton4Event(pressed, sdp)

def run():
    global _sdp, _slamtec_on, _move_oak_d, _mic_array, _pixel_ring, _lpArduino, _radar, _grasper, _grasper_sonar

    # Start 32 bit bridge server
    _sdp = MyClient()
    _lpArduino = LattePandaArduino()
    _lpArduino.initialize()
    #init_local_speech_rec()
    initialize_speech()
    _mic_array = MicArray()
    _pixel_ring = SpeakerPixelRing(_mic_array, )
    _pixel_ring.setStartup()
    #init_camera()

    speak("I'm starting up.")
    _move_oak_d = move_oak_d.MoveOakD()
    _move_oak_d.initialize(_lpArduino.board)

    if _enable_movement_sensing:
        _radar = radar.Radar()
        _radar.initialize(_lpArduino.board)

    if _enable_grasper:
        _grasper = RoboGripper()
        _grasper.initialize(_lpArduino.board)
        #_grasper.setGrasp(120)
        _grasper.allHome()

    _grasper_sonar = _lpArduino.board.get_pin('d:13:o')

    _button_pad.initialize(_lpArduino.board, buttonEventCb)
           
    res = sdp_comm.connectToSdp(_sdp)

    if _execute:
        if (res == 0):
            _slamtec_on = True
            _sdp.setMapUpdate(True)
            pose = _sdp.pose()
            _locations["home"] = (pose.x, pose.y, math.radians(pose.yaw))
            #loadMap(_default_map_name)
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
