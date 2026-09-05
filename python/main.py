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
from aws_mqtt_listener import AwsMqttListener
import my_depthai
from robo_gripper import RoboGripper
from button_pad import Button4Pad
from my_sdp_client import MyClient
from my_sdp_server import *
from sdp_client_manager import manager
from pyFirmata.pyfirmata import util as pyfirmata_util, Pin
import facial_recognize as fr
import re
#from cmd_embed_mgr import CmdEmbedMgr
from nano_vlm_client import NanoVlmClient
from nano_owl_client import NanoOwlClient
from nano_owl_manager import NanoOwlManager
from jetson_supervisor_client import SupervisorClient
from arm_client import ArmClient
import robot_frames
import pyautogui
from my_langgraph import RobotPlannerGraph
import traceback
from move_by_deltas_alert import post_alert
from typing import Dict, List, Callable, Tuple, Optional

# Constants
_show_rgb_window = False
_show_depth_window = False
_default_map_name = 'my house'
_current_map_name = ''
_hotword = "orange"
_google_mode = False
_execute = True # False for debugging, must be True to run as: >python main.py
_run_flag = True # setting this to false kills all threads for shut down
_eyes_flag = False # should eyes be displayed or not
_moods = {"happy":50, "bored":20, "hungry":10}
# whole map rect is represented by all 0s
_WHOLE_MAP_RECT = {"left":0.0,"bottom":0.0, "width":0.0, "height":0.0} 
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
_sonar_grasp_offset = -0.063 # distance to back of grasper
_maps_dir = "maps"
_sounds_dir = "sounds"
_closest_cmd_dist_thresh = 0.15
_closest_cmd_dist_low_conf = 0.12
_jetson_on = True

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
_mqtt_listener = AwsMqttListener()
_aws_mqtt_listener_thread = None
_enable_aws_mqtt_listener = False
_button_pad = Button4Pad()
_grasper_sonar : Pin
_last_grasper_sonar : float = 4.50
#_cmdEmbedMgr : CmdEmbedMgr = None
_map_proc : subprocess.Popen = None
_langgraph : RobotPlannerGraph = None
_langgraph_initiated_move = False
_keep_camera_orientation = False  # prevent handleGotoLocation from resetting OAK-D/eyes
_goto_location_status = "idle"
_nano_vlm : NanoVlmClient = None
_nano_owl : NanoOwlClient = None
_nano_owl_mgr : NanoOwlManager = None
_arm_client : ArmClient = None
# What the gripper is carrying, None when empty. The arm server has no such
# query -- pick_can/place_can are two processes with the object living in the
# planning scene between them -- so the one record of it is here.
_held_object = None

# --- Camera-AI service switching (Jetson supervisor) -------------------------
# The Jetson runs two GPU services that cannot run at once: NanoOWL (object
# search) and NanoVLM (scene description). The supervisor switches between them;
# the single camera-AI layer below (see _activate_camera_ai) is the ONLY code
# that constructs/connects/disconnects _nano_owl / _nano_vlm. A switch is fired
# non-blocking and a short-lived poll thread (_poll_switch) waits for it to
# settle, then connects the client and voices readiness.
_supervisor : SupervisorClient = None
_active_camera_ai = "owl"                 # "owl" | "vlm" | None (connected + ready now)
_pending_camera_ai = None                 # target of an in-flight switch, else None
_SWITCH_SECONDS = {"owl": 40, "vlm": 50}  # approx model-load wall-clock, voiced to user
_CAMERA_AI_FRIENDLY = {"owl": "object search", "vlm": "scene description"}
_awaiting_user_response = False           # True only during an interactive user-speech capture window

# Async operation callback globals

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
    # The response was handled
    Handled = 1
    # The response was handled by LangGraph
    HandledByLangGraph = 2

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
        sdp.turnLeft()
    elif vel < 0:
        sdp.turnRight()
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
        sdp = manager.reader()  # pose read: safe shared, lock-guarded, any thread
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
        pose = manager.reader().pose()  # read: safe shared, lock-guarded, any thread
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

    retval = 0
    # if no action then nothing to do
    if _action_flag:
        if sdp is None:
            sdp = _sdp

        if sdp.getMoveActionStatus() == ActionStatus.Running:
            retval = 1
            for attempt in range(3):
                try:
                    sdp.cancelMoveAction()
                except:            
                    print("An error occurred canceling the action, trying again.")
                    time.sleep(0.1)
                else:
                    break

        if interrupt:
            _interrupt_action = True
            _action_flag = False
    return retval

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
    global _run_flag, _person, _goal, _reported_35, _reported_25, _reported_18, _reported_15
    try:
        batteryPercent = manager.reader().battery()  # read: safe shared, any thread
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

    acrossTheRoom = loc == "across the room"
    if acrossTheRoom:
        sdp.wakeup()
    response = "Ok. I'll search for a " + obj 
    if len(loc):
        response += " in the " + loc
    if op.endswith("_to_me"):
        response += " and bring it back to you, " + person
    elif op.endswith("_to_person"):
        response += " and take it to " + person
    if not _langgraph_initiated_move:
        speak(response)

    _sub_goal = obj + ":" + op
    if acrossTheRoom:
        #time.sleep(3)
        # turn back to original direction to find destination
        rotateTo(orig_yaw, sdp)            
        time.sleep(5) #wait for LiDAR to spin up.
                        
        sdp.getLaserScan()  
        longest_dist, longest_angle = getFurthestLaserScan(sdp)
        # else: # op != "retrieve"
        #     # or find the furthest point in the whole scan
        #     longest_dist, longest_angle = getFurthestLaserScan(sdp)

        longest_dist -= 1.75
        longest_dist = max(longest_dist, 0)

        print("furthest distance: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
        _locations["custom"] = (getLocationFromAngleDist(longest_angle, longest_dist, sdp))
        sdp.setSpeed(1)
        _goal = "custom"
    else:
        _goal = loc
    
################################################################
# This is where goto actions are initiated and get carried out.
# If the robot is in the process of going to a  location, but
# another goto action request is receved, the second request will
# replace the earlier request

def handleGotoLocation():
    global _run_flag, _goal, _action_flag, _interrupt_action
    global _deliveree, _package, _sub_goal, _call_out_objects
    global _error_last_goto, _response_num, _locations, _goto_location_status

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
        elif not _keep_camera_orientation:
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

    sdp = manager.dedicated('goto')

    retrieve_to_loc = None
    sub_goal_cleanup = None
    while _run_flag:
        if _goal == "" or _action_flag:
            _goto_location_status = "idle"
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
        _goto_location_status = "moving to " + _goal + (" with subgoal " + (op if op != "" else "find") + 
                                                        " the " + _sub_goal if _sub_goal != "" else "")

        if _goal == "recharge":
            if not _langgraph_initiated_move:
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
                    if not _langgraph_initiated_move:
                        speak("I'm already at the " + _goal)
                    setNextGoal()
                    _move_oak_d.allHome()
                    continue
                if _goal != sub_goal_cleanup and len(_goal_queue) == 0 or len(_goal_queue) > 0 and _goal_queue[0] != "deliver":
                    if not _langgraph_initiated_move:
                        speak("I'm going to " + _goal)
            elif _goal == "deliver" and _package_ontray:
                speak("hello " + _deliveree)
            elif not _langgraph_initiated_move:
                speak("OK.")
            moveToLocation()
            _action_flag = True

        _interrupt_action = False
        sleepTime = 0.5 if (_sub_goal == "" or _call_out_objects) else 0
        if _goal == "find_face":
            aim_oakd(pitch=85) # aim up to see people better                
            eyes.setTargetPitchYaw(-70, 0)
        if _sub_goal != "":
            checkPersons = _sub_goal == 'person'
            if checkPersons:
                aim_oakd(pitch=85) # aim up to see people better                
                eyes.setTargetPitchYaw(-70, 0)
            else:
                aim_oakd(pitch=135) # aim down towards floor for objects
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
            # _goal_queue.append(_sub_goal)
            # speak("I see a " + _sub_goal)
            _move_oak_d.yawHome()
            #eyes.setHome()

        def gotFace():
            sdp.cancelMoveAction()
            print("got a lock on ", _deliveree)
            #_goal_queue.append(_deliveree)
            #speak("I see " + _deliveree)
            _move_oak_d.yawHome()
            #if face_cmd == "deliver":
            #    _goal_queue.append("release_obj")
            #else:
            #    _goal_queue.append("went_to_face")
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
            time.sleep(0.25)
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
                    aim_oakd(pitch=85) # aim up to see people better                
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
                    if not _langgraph_initiated_move:
                        speak("I've arrived.")
            else: # (_goal != "deliver" and not reached_goal)
                _error_last_goto = True

                if not _langgraph_initiated_move:
                    if _goal == "deliver":
                        speak("Sorry, I could not make my delivery")
                    elif _goal != "custom":
                        speak("Sorry, I didn't make it to the " + _goal)
                    elif _goal != "find_face":
                        speak("Sorry, I didn't make it to where you wanted.")
            if _sub_goal != "":
                speak("and I never found a "+ _sub_goal)
                sdp.setSpeed(_user_set_speed) #restore speed after finding obj
            # if _goal == "find_face": # didn't find face, try one last time with 360 rotation
            #     print("didn't find face while going to goal. Trying 360 rotation")
            #     p = searchForFace(sdp, _deliveree, True)
            #     if p is None:
            #         p = searchForFace(sdp, _deliveree, False)
                    
            #     if _interrupt_action:
            #         _interrupt_action = False
            #     if p is not None:
            #         setLocationOfObj(sdp, _deliveree, p, cam_yaw=_move_oak_d.getYaw(), offset_dist=1.25)
            #         gotFace()
            #         face_just_found = True
            #     else:
            #         speech = "Sorry, I could not find " + _deliveree
            #         if face_cmd == "deliver":
            #             speech += " to make my delivery."
            #         speak(speech)
            #         shutdown_facial_recog()
            #         start_depthai_thread()                 
            #         sdp.setSpeed(_user_set_speed)

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
    manager.release(sdp)
    sdp = None
            
def moveActionMonitor(sdp=None, location_name=None):
    # Monitors the move action status until it is done.
    # if location_name is provided, it indicates the intended destination and handleGotoLocation thread will handle
    # and must wait until action flag is true
    # otherwise it is assumed that the action flag was already set to true
    if location_name is not None:
        while _action_flag == False and _run_flag:
            if _goal == "":  # Goal was cleared - error occurred
                return "unknown location"
            time.sleep(0.50)

    while(_run_flag and _interrupt_action == False):
        maStatus = getMoveActionStatus(sdp)
        if maStatus == ActionStatus.Stopped or \
            maStatus == ActionStatus.Error or \
            maStatus == ActionStatus.Finished:
            break
        time.sleep(0.5)

    # wait for action to finish
    while _action_flag and _run_flag:
        time.sleep(0.1)
    
    if maStatus != ActionStatus.Finished:
        message = "move cancelled" if maStatus == ActionStatus.Stopped else "move error"
    else:
        message = "move finished"

    # check if robot actually made it to the destination
    # if way is blocked the Finished status may be returned.
    if location_name is not None and location_name != "custom" and location_name != "find_face" and location_name != "find_obj":
        reached_goal = is_close_to(location_name)
        message += ": arrived at " if reached_goal else ": did not arrive at "
        message += location_name

    return message

def moveActionMonitorWithOwl(sdp, location_name, obj_name, poll_interval=0.5):
    """Monitor movement while polling NanoOWL for an object.
    Sweeps camera back and forth while moving. On detection, stops to recheck.
    If recheck fails, resumes movement to destination.

    Args:
        sdp: The SDP client connection
        location_name: The intended destination
        obj_name: Object name to search for via NanoOWL
        poll_interval: How often to poll OWL detections (seconds)

    Returns:
        (result_message, spatial_detection_or_None)
    """
    global _action_flag, _interrupt_action, _nano_owl_mgr

    # Wait for action flag to become true (movement started)
    if location_name is not None:
        while _action_flag == False and _run_flag:
            if _goal == "":
                return "unknown location", None
            time.sleep(0.25)

    maStatus = ActionStatus.Running
    spatial_det = None
    obj_confirmed = False

    RECHECK_FRAMES = 5

    _move_oak_d.startSweepingBackAndForth(0)

    while _run_flag and _interrupt_action == False:
        # Poll OWL for the object
        try:
            found, spatial = _nano_owl_mgr.check_for_object(obj_name)
            if found:
                _mdai.drawText(obj_name, 1, 14)
                print(f"OWL spotted {obj_name}, stopping to get a lock: z={spatial.z:.2f}m theta={spatial.theta:.1f}deg")
                sdp.cancelMoveAction()
                _move_oak_d.stopSweepingBackAndForth()
                time.sleep(2)

                # Recheck detection over multiple frames to confirm lock
                for attempt in range(RECHECK_FRAMES):
                    recheck_found, recheck_spatial = _nano_owl_mgr.check_for_object(obj_name)
                    if recheck_found:
                        spatial_det = recheck_spatial
                        print(f"OWL confirmed {obj_name} (frame {attempt+1}): z={recheck_spatial.z:.2f}m theta={recheck_spatial.theta:.1f}deg")
                        obj_confirmed = True
                        break
                    time.sleep(poll_interval)

                if obj_confirmed:
                    maStatus = ActionStatus.Finished
                    break

                # Lost track after multiple frames, sweep 1/3 range to try to reacquire
                print(f"OWL saw {obj_name} but lost track after {RECHECK_FRAMES} frames, sweeping 1/3 range to reacquire")
                _move_oak_d.startSweepingBackAndForth(1, min=75, max=105)
                while _move_oak_d.isSweeping():
                    recheck_found, recheck_spatial = _nano_owl_mgr.check_for_object(obj_name)
                    if recheck_found:
                        spatial_det = recheck_spatial
                        print(f"OWL reacquired {obj_name}: z={recheck_spatial.z:.2f}m theta={recheck_spatial.theta:.1f}deg")
                        obj_confirmed = True
                        break
                    time.sleep(poll_interval)
                if obj_confirmed:
                    _move_oak_d.stopSweepingBackAndForth()
                    maStatus = ActionStatus.Finished
                    break
                # Failed to reacquire, resume going to destination
                print(f"Could not reacquire {obj_name}, resuming movement to {location_name}")
                _move_oak_d.startSweepingBackAndForth(0)
                goToLocation(location_name)
                # Wait for action flag to become true again
                while _action_flag == False and _run_flag:
                    if _goal == "":
                        break
                    time.sleep(0.25)
                continue
        except Exception as e:
            print(f"Error during OWL object check: {e}")

        # Check movement status
        maStatus = getMoveActionStatus(sdp)
        if maStatus == ActionStatus.Stopped or \
            maStatus == ActionStatus.Error or \
            maStatus == ActionStatus.Finished:
            break

        time.sleep(poll_interval)

    if _move_oak_d.isSweeping():
        _move_oak_d.stopSweepingBackAndForth()

    # Wait for action to finish
    while _action_flag and _run_flag:
        time.sleep(0.1)

    if not obj_confirmed:
        if maStatus != ActionStatus.Finished:
            message = "move cancelled" if maStatus == ActionStatus.Stopped else "move error"
        else:
            message = "move finished"
    else:
        message = "move finished"

    if location_name is not None and location_name != "custom" and location_name != "find_face" and location_name != "find_obj":
        if not obj_confirmed:
            reached_goal = is_close_to(location_name)
            message += ": arrived at " if reached_goal else ": did not arrive at "
            message += location_name

    return message, spatial_det


def moveActionMonitorWithPrompt(sdp=None, location_name=None, prompt="Describe the scene concisely.",
                                prompt_interval_seconds: int = 5, output_cb: Optional[Callable[[str], bool]] = None):
    """Monitors movement while periodically prompting the vlm about the scene.
    
    This is a reusable monitor function that can be used by any navigation tool
    that wants to prompt the vlm aboute the scene during movement.
    
    Args:
        sdp: The SDP client connection
        location_name: The intended destination (optional)
        prompt: The prompt to send to the VLM (default: "Describe the scene concisely.")
        prompt_interval_seconds: How often to prompt the VLM about the scene (default 5 seconds)
        output_cb: Optional callback function to handle the scene description output (e.g., for speech, or info) 
                   and return True to stop movement.
        
    Returns:
        result_message
    """
    global _action_flag, _interrupt_action, _nano_vlm
    
    # Wait for action flag to become true (movement started)
    if location_name is not None:
        while _action_flag == False and _run_flag:
            if _goal == "":  # Goal was cleared - error occurred
                return "unknown location"
            time.sleep(0.25)
    
    # Monitor movement while periodically describing scene
    last_narration_time = time.monotonic()
    maStatus = ActionStatus.Running
    
    if _nano_vlm is not None:
        _nano_vlm.set_prompts([prompt])

    while _run_flag and _interrupt_action == False:
        # Check if it's time for a narration
        current_time = time.monotonic()
        if current_time - last_narration_time >= prompt_interval_seconds:
            try:
                # Get scene description from VLM
                if _nano_vlm is not None:
                    output, _, _ = _nano_vlm.get_output(prompt_filter=prompt)
                    if output:
                        # call the cb with the output
                        if output_cb is not None:
                            should_stop = output_cb(output)
                            if should_stop:
                                cancelAction(interrupt=False, sdp=sdp)
                                maStatus = ActionStatus.Finished
                                break
            except Exception as e:
                print(f"Error getting scene vlm output: {e}")
            
            last_narration_time = current_time
        
        # Check movement status
        maStatus = getMoveActionStatus(sdp)
        if maStatus == ActionStatus.Stopped or \
            maStatus == ActionStatus.Error or \
            maStatus == ActionStatus.Finished:
            break
        
        time.sleep(0.1)
    
    # Wait for action to finish
    while _action_flag and _run_flag:
        time.sleep(0.1)
    
    # Determine result message
    if maStatus != ActionStatus.Finished:
        message = "move cancelled" if maStatus == ActionStatus.Stopped else "move error"
    else:
        message = "move finished"
    
    # Check if robot actually made it to the destination
    if location_name is not None and location_name != "custom" and location_name != "find_face" and location_name != "find_obj":
        reached_goal = is_close_to(location_name)
        message += ": arrived at " if reached_goal else ": did not arrive at "
        message += location_name

    return message


def move_by_deltas(sdp, deltas: List[Dict[str, float]], final_yaw: float, req_approval=False):
    """
    Function to move robot series of delta offsets from robot POV
    
    Args:
        deltas (list): List of dictionaries with 'dx' and 'dy' values for each delta offset.
        where +x axis is forward and +y axis is left from robot POV.
        Maximum number of deltas is defined by MAX_NUM_ROBOT_LOCATIONS. anymore will be ignored.

        final_yaw in degrees (float): The desired orientation after applying the final delta.

    """
    global _action_flag, _interrupt_action

    print("Moving by deltas with yaw {}:".format(final_yaw))
    scale = 100
    locs = LOCATIONS()
    locs.count = min(MAX_NUM_ROBOT_LOCATIONS, len(deltas))
    pose = sdp.pose()
    abs_points = [(pose.x * scale, pose.y * scale)]

    for i in range(0, locs.count):
        # Calculate world coordinates for each delta (dx, dy) in robot's local frame
        dx = deltas[i]['dx']
        dy = deltas[i]['dy']
        yaw_rad = math.radians(pose.yaw)
        # Transform (dx, dy) from robot frame to world frame using current yaw, a rotation
        locs.values[i].x = pose.x + dx
        locs.values[i].y = pose.y + dy
        abs_points.append((locs.values[i].x * scale, locs.values[i].y * scale))
        pose.x = locs.values[i].x
        pose.y = locs.values[i].y

    if req_approval:
        approved = post_alert(abs_points)
        if not approved:
            return "Movement cancelled by user because it does not match the intended shape."
    
    _interrupt_action = False
    _action_flag = True
    sdp.moveTosFloatWithYaw(locs, math.radians(final_yaw))

def move_by_deltas_tool_helper(sdp, deltas: List[Dict[str, float]], final_yaw: float):
    """Move through a series of deltas and await completion."""
    move_by_deltas(sdp, deltas, final_yaw, req_approval=True)
    return moveActionMonitor(sdp)
    
    
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

def stop_speaking():
    try:
        _voice.say("", tts.flags.SpeechVoiceSpeakFlags.PurgeBeforeSpeak.value or
            tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value)
    except Exception:
        print("Stop speaking has timed out.")
        pass

def wait_until_speech_done():
    global _voice
    while _voice.voice.WaitUntilDone(100) == False:
        pass

def speak(phrase, flag=tts.flags.SpeechVoiceSpeakFlags.Default.value, add_to_memory=True):
    global _last_phrase, _voice

    try:
        print("speaking: ", phrase)
        _pixel_ring.setSpeak()
        _voice.say(phrase, flag)
        # add robot response to memory
        if add_to_memory and _langgraph is not None:
            _langgraph.add_to_memory(robot_response=phrase)
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

def loadMap(filename, sdp):
    global _current_map_name
    sdp.wakeup()
    print("Loading map and its locations")
    filepath = os.path.join(_maps_dir, filename)
    res = sdp.loadSlamtecMap(str.encode(filepath) + b'.stcm')
    if (res == 0):
        # set update to false because we don't want to change the map when doing a demo with people standing around messing up the map!
        sdp.setMapUpdate(False)
        speak("Map and locations are loaded. Mapping is off.")
        # speak("Now let me get my bearings.")
        # result, errStr = recoverLocalization(_INIT_RECT)
        # if result == False:
        #    speak("I don't appear to be at the map starting location.")
        _current_map_name = filename
        load_locations(filepath)
        print("Done loading map")
    else:
        speak("Something is wrong. I could not load the map.")
    return res

def saveMap(filename, sdp):
    global _current_map_name
    filepath = os.path.join(_maps_dir, filename)
    print("saving map and its locations")
    res = sdp.saveSlamtecMap(str.encode(filepath) + b'.stcm')
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
    global _person, _mood

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
    answer = answer + str(manager.reader().battery()) + " percent"
    speak(answer)
    answer = "And I'm feeling " + _mood
    speak(answer)
    time.sleep(5)
        
# rotate 360 and stop if a person is spotted
def searchForPerson(sdp, is_clockwise=True):
    global _action_flag, _interrupt_action

    _interrupt_action = False
    aim_oakd(pitch=85) # aim up to see people better
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

def checkForObject(obj, max_tries=1):
    ps = None
    p = None
    for i in range(0,max_tries):
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

# rotate 360 and stop if a object is spotted
def searchForObject(sdp, obj, height="eye level", is_clockwise=True, checkForObject=checkForObject, rot_speed=0.1,
                    min_recheck=False, stream_mgr=None):
    global _action_flag, _interrupt_action

    _interrupt_action = False
    if height=="floor":
        aim_oakd(pitch=135) # aim down to see floor objects better
        eyes.setTargetPitchYaw(-50, 0)
    elif height == "up high":
        aim_oakd(pitch=85) # aim up to see high objects better
        eyes.setTargetPitchYaw(50, 0)
    else: # eye level
        _move_oak_d.allHome()
        eyes.setHome()

    p = None
    # First see if the obj is already in view and if so return
    found, p = checkForObject(obj)
    if found:
        _move_oak_d.allHome()
        return p
    
    # if object not in view, then slowly rotate 360 degrees and check every so often
    _action_flag = True
    
    oldyaw = sdp.heading() + 360
    yaw = oldyaw
    sweep = 0
    recheck_obj = False
    while (sweep < 380 and not _interrupt_action):
        sdp.rotate(-rot_speed if is_clockwise else rot_speed)
        found, p = checkForObject(obj)
        if found:
            recheck_obj = True
            break
        yaw = sdp.heading() + 360
        covered = abs(yaw - oldyaw)
        if covered > 180:
            covered = 360 - covered
        sweep += covered
        #print("yaw = ", yaw, " covered = ", covered, " sweep = ", sweep)
        
        oldyaw = yaw
        time.sleep(0.05)

    if recheck_obj:
        print(f"rechecking {obj}")
        sdp.cancelMoveAction()
        time.sleep(0.3)
        if stream_mgr is not None:
            stream_mgr.pause_streaming()
        deg = 5 if is_clockwise else -5

        for j in range(1,5):
            for i in range(1,5):
                if stream_mgr is not None:
                    stream_mgr.push_fresh_frame()
                found, p = checkForObject(obj)
                if found:
                    break
                time.sleep(0.12 if min_recheck else 0.05)
            if not found:
                # go back other way
                sdp.rotate(math.radians(deg))
                sdp.waitUntilMoveActionDone()
                deg = -deg
            else:
                break
        if stream_mgr is not None:
            stream_mgr.resume_streaming()

    _action_flag = False
    if not _keep_camera_orientation:
        _move_oak_d.allHome()
    return p

# rotate 360 and stop if the person's face is spotted
def searchForFace(sdp, name, is_clockwise=True):
    global _action_flag, _interrupt_action

    aim_oakd(pitch=85) # aim up to see people better
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

def getLocationOfObj(sdp, obj, p, cam_yaw=0, offset_dist=0.75, radians=True):
    yaw, xt, yt = getLocationNearObj(sdp, obj, p, cam_yaw, offset_dist)
    angle = yaw + cam_yaw + p.theta
    return (xt, yt, math.radians(angle) if radians else angle)

def setLocationOfObj(sdp, obj, p, cam_yaw=0, offset_dist=0.75):
    _locations[obj] = getLocationOfObj(sdp, obj, p, cam_yaw, offset_dist)

def setFoundObjAsGoal(obj, cam_yaw=0, offset_dist=0.75, sdp=None):
    if sdp is None:
        sdp = manager.reader()  # only reads pose (setLocationOfObj); safe shared
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
        sdp = manager.reader()  # only reads pose (setLocationOfObj); safe shared
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
    aim_oakd(pitch=85) 
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
    aim_oakd(yaw=move_oak_d._YAW_HOME_, pitch=75)
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

def findPersonWithRotate(sdp, yawDelta):
    # find person / deliveree
    ps = searchForPerson(sdp, yawDelta > 0)
    if len(ps) > 0:
        z = 9999.0
        # find closest person
        for p in ps:
            if p.z < z:
                z = p.z
        return p
    return None

def getLocOfPersonFromSound(doa, sdp):
    # look towards sound of voice and if person spotted, return it
    yawDelta = _mic_array.rotateToDoa(doa, sdp)
    p = findPersonWithRotate(sdp, yawDelta)
    if p is not None:
        return getLocationOfObj(sdp, "person", p, cam_yaw=0, offset_dist=1.0)
    else:
        return None

def findAndSetLocOfPersonFromSound(person, doa, sdp):
    # look towards sound of voice and if person spotted, record location
    yawDelta = _mic_array.rotateToDoa(doa, sdp)
    
    # Remove deliveree from locations
    if _locations.get(person):
        _locations.pop(person)

    p = findPersonWithRotate(sdp, yawDelta)
    if p is not None:
        setLocationOfObj(sdp, person, p, cam_yaw=0, offset_dist=1.0)
    else:
        return None
    return person

def come_here(doa):
    global _goal, _interrupt_action

    sdp = manager.dedicated('come_here')
    
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
            move_oak_d.pitchHome()
    manager.release(sdp)
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

def goToLocation(location):
    global _goal
    _goal = location

class GoDir:
    Forward = 0
    Backward = -180
    Right = -90
    Left = 90

# move in the given direction: forward, backward, right, or left from current pose for the given distance and unit
def moveInDirDist(dir, dist, unit, sdp):
    global _goal
    if dir == "forward":
        phi = GoDir.Forward
    elif dir == "backward":
        phi = GoDir.Backward
    elif dir == "right":
        phi = GoDir.Right
    elif dir == "left":
        phi = GoDir.Left

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
        return("unknown unit")

    #if unit not mentioned assume meters
    pose = sdp.pose()
    xt = pose.x + dist * math.cos(math.radians(pose.yaw + phi))
    yt = pose.y + dist * math.sin(math.radians(pose.yaw + phi))
    
    print("going to ", xt ,", ", yt)

    _locations["custom"] = (xt, yt)
    _goal = "custom"

def get_known_faces():
    return fr.get_known_faces()



# while crossing the room towards the furthest wall visible, look for the named person's face
def look_for_face(name, sdp):
    global _deliveree, _goal
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

def identify_visible_face():
    global _interrupt_action
    
    eyes.setTargetPitchYaw(-70, 0)
    shutdown_my_depthai()
    start_facial_recog()
    ided = False
    end = time.monotonic() + 5
    while time.monotonic() < end and not _interrupt_action:
        face = checkForFace(1)   
        if face is not None:
            ided = True
            break
    shutdown_facial_recog()
    start_depthai_thread()
    eyes.setHome()
    _move_oak_d.allHome()
    if not ided:
        return False, ""
    return True, face

def set_handling_response(value):
    global _handling_resp
    with _handling_resp_lock:
        _handling_resp = value

def handling_response():
    with _handling_resp_lock:
        return _handling_resp or _handle_resp_thread is not None and _handle_resp_thread.is_alive()

def handle_response_sync(sdp, phrase, doa, assist = False, listenResponseFn=None):
    if handling_response():
        print("already handling response, try again later.")
        return HandleResponseResult.NotHandledBusy
    set_handling_response(True)
    try:
        handled_result = handle_response(sdp, phrase, doa, listenResponseFn=listenResponseFn)
        if handled_result == HandleResponseResult.NotHandledUnknown:
            # add the human speech to memory
            speak("Sorry, I don't understand \"" + phrase.split(_hotword)[-1] + "\"?")
        elif handled_result != HandleResponseResult.HandledByLangGraph:
            _langgraph.add_to_memory(user_input=phrase)
    finally:
        set_handling_response(False)

def handle_response_async(sdp, phrase, doa):
    # issue the command in its own thread
    _handle_resp_thread = Thread(target = handle_response_sync, args=(sdp, phrase, doa), name = "handle_response_async", daemon=False)
    _handle_resp_thread.start()



###############################################################
# Command Handler
def handle_response(sdp, phrase, doa, listenResponseFn : typing.Union[typing.Callable[[object, int], str], None] = None):
    global _run_flag, _goal, _listen_flag, _last_phrase
    global _person, _mood, _time
    global _action_flag, _internet, _use_internet
    global _eyes_flag, _hotword, _sub_goal, _all_loaded
    global _deliveree, _map_proc

    # Warm up / Enable VLM 
    if _nano_vlm is not None:
        _nano_vlm.enable()

    if _nano_owl is not None:
        _nano_owl.enable()

    # class ImageCallback:
    #     def __init__(self):
    #         self._image = None
    
    #     def get_picture_cb(self, frame):
    #         # resize the image's larger dimension to 512 pixels while keeping the aspect ratio
    #         if frame.shape[0] > frame.shape[1]:
    #             frame = cv2.resize(frame, (int(512 * frame.shape[1] / frame.shape[0]), 512))
    #         else:
    #             frame = cv2.resize(frame, (512, int(512 * frame.shape[0] / frame.shape[1])))
    #         _, self._image = cv2.imencode(".jpg", frame)

    #     def get_image(self):
    #         return self._image

    tried_closest_cmd = False

    # max of twice through this loop. if first time command is not recognized then closest command is tried
    while True:
        # convert phrase to lower case for comparison
        phrase = phrase.lower().strip() 
        if phrase == "stop all":
            _langgraph.cancel_stream()

        if phrase == "stop" or phrase == "stop moving" or phrase == "stop motors" or phrase == "stop stop":
            retval = cancelAction(True, sdp)
            if retval == 1:
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
                    
        if phrase == "where are you going":
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
        
        if phrase == "status":
            statusReport()
            return HandleResponseResult.Handled

        if re.match(r"^list (people|persons|faces) you know", phrase):
            names = get_known_faces()
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
                deliveree = findAndSetLocOfPersonFromSound(_person, doa, sdp)
                if deliveree is None:
                    speak("sorry, i could not find you to bring the " + obj + " to.")
                    return HandleResponseResult.Handled
                else:
                    _deliveree = deliveree
            elif "and take it to" in phrase:
                if "in the" in phrase:
                    obj_p, _, loc = phrase.partition(op)[2].partition("in the")[0:3]
                    loc, deliveree_p = loc.split("and take it to")[0,3]
                else:
                    loc = ''
                    obj_p, _, deliveree_p= phrase.partition(op)[2].partition("and take it to")[0:3]
                obj = obj_p.split()[-1]
                deliveree = deliveree_p.split()[-1]
                names = fr.get_known_faces()
                if deliveree not in names:
                    speak("sorry, i have not met " + deliveree + ", and I don't know what they look like.")
                    return HandleResponseResult.Handled
                op += "_to_person"
                # assume deliveree is near user giving command
                if None == findAndSetLocOfPersonFromSound(deliveree, doa, sdp):
                    # if no person found then come back to this loc to look after retrieving object
                    return_to_loc, _, _ = where_am_i()
                    _locations[deliveree] = return_to_loc                     
                _deliveree = person
                
            else: # not "bring it to me" in phrase
                obj_p, _, loc = phrase.partition(op)[2].partition("in the")[0:3]
                obj = obj_p.split()[-1]
            loc = loc.strip()

            findOrRetrieveObject(loc, obj, op, person, orig_yaw, sdp)
            return HandleResponseResult.Handled

        # HBRC Floor Bot Challenge I
        if phrase == "go across the room and come back":
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
        # if phrase.startswith("go to"):
        #     if not _slamtec_on:
        #         speak("I'm sorry, movement is disabled.")
        #         return HandleResponseResult.Handled
        #     words = phrase.split()
        #     try:
        #         words.remove("the")
        #     except:
        #         None
        #     if len(words) > 2:
        #         cancelAction(True, sdp)
        #         goal = " ".join(words[2:]).lower()
        #         goToLocation(goal)
           
        #     return HandleResponseResult.Handled
                
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
            result, errStr = recoverLocalization(_WHOLE_MAP_RECT)
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
            print("shutting down jetson")
            os.system("ssh jim@192.168.55.1 \"sudo -S shutdown -h now\"")
            print("shutting down LattePanda in 10s")
            _run_flag = False
            # shutdown 
            os.system("shutdown /s /t 10")
            return HandleResponseResult.Handled
                
        if phrase == "battery" or phrase == "voltage":
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
            loadMap(name, sdp)
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
            saveMap(name, sdp)
            return HandleResponseResult.Handled
        
        if phrase == "clear map":
            speak("Ok. I will clear my map.")
            sdp.clearSlamtecMap()
            # after clearing make sure updating is on
            sdp.setMapUpdate(True)
            return HandleResponseResult.Handled
        
        if phrase == "clear locations":
            speak("Ok. I will clear the locations.")
            _locations.clear()
            return HandleResponseResult.Handled

        if phrase == "enable mapping":
            speak("Ok. I will enable map updating.")
            sdp.setMapUpdate(True)
            return HandleResponseResult.Handled                

        if phrase == "disable mapping":
            speak("Ok. I will disable map updating.")
            sdp.setMapUpdate(False)
            return HandleResponseResult.Handled

        if phrase == "show map" or phrase == "open map":
            # launch robostudio
            speak("ok.")
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
            speak("i opened up the map.")
            return HandleResponseResult.Handled

        if phrase == "close map" or phrase == "hide map":
            if _map_proc is not None:
                _map_proc.terminate()
                _map_proc = None
            return HandleResponseResult.Handled
        
        if phrase == "take a picture":
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

        if phrase == "take my picture":
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
        if phrase.startswith("set speed"):
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
                
        if phrase == "close pictures":
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
            _move_oak_d.setPitch(75) # pitch up to see person better
            speak("Ok. Let's dance.")
            sdp.setSpeed(3)
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
            sdp.setSpeed(_user_set_speed)
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
        
        if phrase.startswith("forget the face of"):
            name = phrase[18:]
            speak("Ok. I will forget the face of " + name)
            response = forget_a_face(name)
            speak(response)
            return HandleResponseResult.Handled
        
        if phrase == "list locations":
            speak("ok.")
            print("Locations I know:")
            for l in _locations.keys():
                print(l)
            return HandleResponseResult.Handled

        if phrase.startswith("rename location"):
            rest = phrase[16:].split(" to ")
            loc = rest[0]
            if len(rest) < 2:
                speak("Please say the new name.")
                return HandleResponseResult.Handled
            new_name = rest[1]
            speak("Ok. I will rename location " + loc + " to " + new_name)
            if loc in _locations:
                _locations[new_name] = _locations.pop(loc)
            else:
                speak("I don't know of location " + loc)
            return HandleResponseResult.Handled

        # parse var
        if phrase.startswith("delete location"):
            loc = phrase[16:]
            speak("Ok. I will delete location " + loc)
            if _locations.pop(loc, None) is None:
                speak("I don't know of location " + loc)
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

        # if "local speech" in phrase:
        #     switch_to_local_speech()
        #     return HandleResponseResult.Handled

        # if "cloud speech" in phrase:
        #     switch_to_cloud_speech()
        #     return HandleResponseResult.Handled

        if phrase == "enable radar":
            speak("ok, i've enabled radar.")
            start_radar()
            return HandleResponseResult.Handled

        if phrase == "disable radar":
            speak("ok, i've disabled radar.")
            stop_radar()
            return HandleResponseResult.Handled
            
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
        if (phrase.startswith("bring") or phrase.startswith("take")) and ("this" in phrase or "these" in phrase):
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
            turn(deg, sdp)
            return HandleResponseResult.Handled

        break # exit parse while loop

        # if chatbot command is recognized then skip to chatbot
        # if phrase == "reset memory" or phrase == "show chat log" or "you see" in phrase \
        #     or "describe this" in phrase or "identify this" in phrase or "what is this" in phrase:
        #     break

        # if tried_closest_cmd:
        #     print("error - closest command did not parse. check parser and command list")
        #     return HandleResponseResult.NotHandledUnknown
        
        #print("parser failed, looking up closest command")
        # if not understood, try to match the command using the embeddings manager
        #closest_command, dist = _cmdEmbedMgr.find_closest_command(phrase)
        #print("closest command is '{}' with distance: {}", closest_command, dist)
        #low_conf = dist < _closest_cmd_dist_thresh and dist > _closest_cmd_dist_low_conf
        #if dist >= _closest_cmd_dist_thresh:
        #    closest_command = None
        #elif low_conf:
            #ask user if correct
        #    speak("Did you mean, "+ closest_command + "?")
        #    if listenResponseFn is not None:
        #        response = listenResponseFn(sdp, 5)
        #        # if not yes let the agent handle it
        #        if response == "cancel":
        #            speak("ok")
        #            return HandleResponseResult.Handled
        #        elif response != "yes":
        #            break

        #tried_closest_cmd = True
        #if closest_command is not None:
        #    phrase = closest_command
        #    if not low_conf:
        #        speak("I assume you meant " + phrase + ".")
        #    print("trying again with closest command \"{}\", dist = {}".format(phrase, dist))
        #else:
        #    break
        # end of parse while(true)

    if _langgraph is None:
        print("Error: no langgraph agent")
        return HandleResponseResult.NotHandledUnknown

    # if not handled by old school parsing send it to the Langgraph Agent with tools 
    print("sending speech to langgraph agent")

    # Check if langgraph is already processing a request
    if _langgraph is not None and _langgraph.is_processing:
        print("langgraph agent is busy processing another request")
        return HandleResponseResult.NotHandledBusy
    
    image = None
    response = ""
    if len(phrase) > 0:
        if phrase == "reset memory":
            _langgraph.reset_memory()
            speak("I'm clearing my memory of this discussion.")
            return HandleResponseResult.Handled
        
        elif phrase == "show chat log":
            print(_langgraph.print_message_history())
            return HandleResponseResult.Handled

        # if not "album" in phrase and not re.search(r"that (photo|picture|image)", phrase) \
        #     and ("you see" in phrase or "describe this" in phrase or "identify this" in phrase \
        #     or "what is this" in phrase or "using your camera" in phrase):
            
        #     if _langgraph.has_vision:
        #         imageCallback = ImageCallback()
        #         _mdai.setGetPictureCb(imageCallback.get_picture_cb)
        #         timeout = time.monotonic() + 5
        #         if not _mdai.rgbWindowVisible():
        #             speak("I have to open the RGB window first. Hold on.")
        #             _mdai.showRgbWindow(True)
        #             _mdai.waitUntilChangeFinished()
        #             # wait for camera exposure to adjust
        #             time.sleep(1.5)                
        #         while imageCallback.get_image() is None and time.monotonic() < timeout:
        #             time.sleep(0.1)
        #         if imageCallback.get_image() is None:
        #             speak("I'm sorry, I can't see anything. Maybe you left the lens cap on or my rgb window is not open")
        #             return HandleResponseResult.Handled
        #         image = imageCallback.get_image()
        #     else:
        #         speak("I'm sorry, I can't currently interpret what I'm seeing.")
        #         return HandleResponseResult.Handled

        
        response = ""
        tools_log = ""    
        try:
            response, tools_log = _langgraph.send_input(phrase, image=image)
        except Exception as e:
            print("Error in getting response: ", e)
            traceback.print_exc()
            response = "Sorry, I could not get a response."

        # if len(response) > 0:
        #     speak(response, flag=tts.flags.SpeechVoiceSpeakFlags.FlagsAsync.value, add_to_memory=False)
        # else:
        #     speak("I got nothing on that.")

        #print("Tools Log: ", tools_log)
        
        return HandleResponseResult.HandledByLangGraph
    return HandleResponseResult.NotHandledUnknown # not handled

###############################################################
# Speech recognition using Google over internet connection
def listen():
    global _run_flag, _goal, _last_speech_heard
    global _internet, _use_internet, _hotword, _google_mode

    sdp = manager.dedicated('listen')
    
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
            with m as source: r.adjust_for_ambient_noise(source, duration=1, is_speech_cb=None if _mic_array.is_sim_mode() else _mic_array.getIsSpeech)
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

    #speak("Hello, My name is Orange. Pleased to be at your service.")

    HEY_ORANGE_KEYWORD_IDX = 0
    STOP_NOW_KEYWORD_IDX = 1
    GET_RESPONSE_IDX = 2

    def listenFromVoskSpeechRecog(r : sr.Recognizer, mic, sr,
                                  oww_config : typing.Union[sr.Recognizer.OpenWakeWordListener.Config, None],
                                  timeout=None) -> tuple[str, float]:
        global _last_speech_heard
      # obtain audio from the microphone
        try:
            with mic as source:
                print("Say something!")
                if oww_config is not None:
                    _pixel_ring.setOff() # turn off from trace mode so wake word volume effect is noticable
                audio = r.listen(source, timeout = timeout, phrase_time_limit = 10, oww_config = oww_config,
                                 is_speech_cb=None if _mic_array.is_sim_mode() else _mic_array.getIsSpeech)
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
            if e.__context__:
                print(e.__context__)
                raise e.__context__
            return "", 0

        # recognize speech using Vosk Speech Recognition
        try:
            result = r.recognize_vosk(audio, arg2=None, alts=3)
            print(result)
            phrase = json.loads(result)
            phrase = phrase["alternatives"][0]["text"].strip()          
            print("I heard: \"%s\" at %d degrees." % (phrase, manager.reader().heading() + _mic_array.doa2YawDelta(doa)))
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
                audio = r.listen(source, phrase_time_limit = 7, is_speech_cb=None if _mic_array.is_sim_mode() else _mic_array.getIsSpeech)
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
            print("I heard: \"%s\" at %d degrees." % (phrase, manager.reader().heading() + _mic_array.doa2YawDelta(doa)))
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
        global _awaiting_user_response
        on_detection(GET_RESPONSE_IDX)
        try:
            phrase, _ = listenFromVoskSpeechRecog(r, mic, sr, None, timeout=5)
        except:
            speak("sorry, i am having trouble understanding.")
        finally:
            _awaiting_user_response = False  # user-speech capture window closed
        return phrase

    def listenFromVosk(sdp, oww_config, finallyFunc=lambda:None):
        global _awaiting_user_response
        try:
            phrase, doa = listenFromVoskSpeechRecog(r, mic, sr, oww_config)
            _awaiting_user_response = False  # command capture done (wake-word wait is idle, not counted)
            if phrase != "stop moving":
                setPixelRingTrace()
            try:
                handle_response_sync(sdp, phrase, doa, listenResponseFn=listenFromVoskResponse)
            except Exception as e:
                print(str(e))
                traceback.print_exc()
                speak("sorry, i could not do what you wanted.")
        finally:
           finallyFunc()

    def listenFromGoogle(sdp, finallyFunc=lambda:None):
        try:
            phrase, doa = listenFromGoogleSpeechRecog(r, mic, sr)
            # slip in an ambient noise level adjustment here because some speech may have just
            # ended or a timeout occurred.
            #adj_spch_recog_ambient(r, mic)
            setPixelRingTrace()
            handle_response_sync(sdp, phrase, doa)
        except Exception as e:
            print(str(e))
            traceback.print_exc()
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
        print("I heard: \"%s\" at %d degrees" % (phrase, manager.reader().heading() + _mic_array.doa2YawDelta(doa)))
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
#        except:
#           speak("sorry, i could not do what you wanted.")
#        finally:
        listener.set_active(True)
    
    global _starting_up
    _starting_up = False
    _pixel_ring.setEndStartup() # restore pixel ring to default sound sensitive mode after boot up

    def on_detection(index):
        global _awaiting_user_response
        if index == HEY_ORANGE_KEYWORD_IDX:
            _awaiting_user_response = True  # user just triggered; about to speak a command
            stop_speaking()
            for i in range(1, 12):
                _pixel_ring.setColoredVolume(i)
                time.sleep(0.005)
        elif index == STOP_NOW_KEYWORD_IDX:
            stop_speaking()
            _pixel_ring.setRedVolume()
        elif index == GET_RESPONSE_IDX:
            _awaiting_user_response = True  # robot prompted; awaiting the user's reply
            for i in range(1, 12):
                _pixel_ring.setBlueVolume(i)
                time.sleep(0.0075)

    def on_listen_timeout(index):
        if index == HEY_ORANGE_KEYWORD_IDX:
            for i in range(11, -1, -1):
                _pixel_ring.setColoredVolume(i)
                time.sleep(0.005)

    models_dir = os.path.join(os.path.dirname(__file__), "models")
    wake_path = os.path.join(models_dir, "hey_orange.onnx")
    stop_path = os.path.join(models_dir, "stop_now.onnx")
    for p in (wake_path, stop_path):
        if not os.path.isfile(p):
            speak("I cannot find one of my wake word model files. Shutting down.")
            _run_flag = False
            break
    oww_config = r.OpenWakeWordListener.Config(
        model_paths=[wake_path, stop_path],
        keyword_types=[r.OpenWakeWordListener.KeywordType.LISTEN,
                       r.OpenWakeWordListener.KeywordType.IMMEDIATE],
        thresholds=[0.5, 0.5],
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
                listenFromVosk(sdp, oww_config=oww_config)
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
            traceback.print_exc()
            print(e)

        #while _run_flag and local_listener is not None and not _google_mode:
        #   time.sleep(2)
    
    # if no longer running, stop listening 
    #winspeech.stop_listening()
    manager.release(sdp)
    sdp = None

def recoverLocalization(sdp=None, rect=_WHOLE_MAP_RECT):
    if sdp is None:
        sdp = _sdp
    sdp.recoverLocalization(rect["left"],
                                      rect["bottom"],
                                      rect["width"],
                                      rect["height"])
    result = sdp.waitUntilMoveActionDone()
    time.sleep(0.05)
    result = sdp.getMoveActionStatus()
    print("Recovering localization result = ", result)
    if result == ActionStatus.Finished:
        location, distance, closeEnough = where_am_i()
        if not _langgraph_initiated_move:
            if closeEnough:
                speak("I appear to be at the " + location + " location.")
            else:
                speak("I appear to be near the "+ location + " location.")
        return True, ""
    elif result == ActionStatus.Error:
        errStr = sdp.getMoveActionError()
        print("Possibly an error occurred: ", errStr)
        return False, errStr
    return False, ""

#aim camera straight ahead and level
def home_oakd():
    _move_oak_d.allHome()

#aim camera. pitch of 75 is good for looking at faces, pitch of 135 is good for down at floor
def aim_oakd(yaw = None, pitch = None):
    print("Aiming OAK-D to yaw:", yaw, " pitch:", pitch)
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

def start_depthai_thread(model="yolo8nano", use_tracker=False, loc="TOP"):
    global _my_depthai_thread, _mdai

    if _my_depthai_thread is not None:
        return
    _mdai = my_depthai.MyDepthAI(model, use_tracker)

    _my_depthai_thread = Thread(target = _mdai.safe_startUp, args=(loc, _show_rgb_window, _show_depth_window), name="mdai", daemon=False)
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

    # A dedicated, thread-affine command client (msl.loadlib is not thread safe).
    sdp = manager.dedicated('follow_me')

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
            #     # tracking id will be a new one at this point so reset the tracker to take the closest person
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
    manager.release(sdp)
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
        return handle_response_async(sdp, arg1, 0)
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
             
def forget_a_face(name:str):
    file = f"databases/{name}.npz"
    if os.path.exists(file):
        os.remove(file)
        return(f"I will no longer recognize {name}'s face.")
    else:
        return(f"I don't have a memory of {name}'s face.")
    
# Tool helper functions for LangGraph
def list_locations_tool_helper():
    """List all known locations."""
    global _locations
    if not _locations:
        return "No locations saved."
    return f"Known locations: {', '.join(_locations.keys())}"

def search_for_face_tool_helper(sdp, name: str, rot_clockwise: bool = True):
    """Search for a face by rotating in place."""
    global _interrupt_action
    try:
        sdp.setSpeed(1) #slow speed
        shutdown_my_depthai()
        start_facial_recog(with_spatial=True, with_tracking=False)        

        p = searchForFace(sdp, name, rot_clockwise)
        if p is None:
            p = searchForFace(sdp, name, not rot_clockwise)
            
        if _interrupt_action:
            _interrupt_action = False
        if p is not None:
            loc = getLocationOfObj(sdp, name, p, cam_yaw=_move_oak_d.getYaw(), offset_dist=1.25, radians=False)

        shutdown_facial_recog()
        start_depthai_thread()
        sdp.setSpeed(_user_set_speed)

        if p is not None:
            return f"Found {name} at {loc}."
        return f"No instances of {name} found."
    except Exception as e:
        return f"Error searching for face {name}: {str(e)}"
    finally:
        _move_oak_d.allHome()

def while_go_to_location_find_face_tool_helper(sdp, name: str, loc: str):
    """While going to a location, look for a face by name and stop as soon as it is spotted and report its coords."""
    global _langgraph_initiated_move, _goal, _deliveree
    try:
        # Mark this as a LangGraph-initiated move
        _langgraph_initiated_move = True

        # Initiate movement
        sdp.setSpeed(1) #slow speed
        # switch to facial recognition with spatial location
        shutdown_my_depthai()
        start_facial_recog(with_spatial=True, with_tracking=False)

        # Set the deliveree to the person being searched for
        _deliveree = name

        if loc == "across the room":
            sdp.wakeup()
            time.sleep(5) #wait for LiDAR to spin up.  
            sdp.getLaserScan()  
            longest_dist, longest_angle = getFurthestLaserScan(sdp)
            longest_dist -= 1.75
            longest_dist = max(longest_dist, 0)

            print("furthest distance: angle = ", math.degrees(longest_angle), " distance = ",longest_dist)
            _locations["find_face"] = (getLocationFromAngleDist(longest_angle, longest_dist, sdp))
        else:
            _locations["find_face"] = _locations.get(loc, (0,0,0))
        _goal = "find_face"

        # Monitor the movement and search for the person
        result = moveActionMonitor(sdp, "find_face")

        shutdown_facial_recog()
        start_depthai_thread()
        sdp.setSpeed(_user_set_speed)

        # check if the person was found
        loc = _locations.get(_deliveree, None)
        if loc is not None:
            loc = (loc[0], loc[1], math.degrees(loc[2]))  # convert yaw to degrees for output
            result += f", {_deliveree} was found at {loc}"
            del _locations[_deliveree]
        else:
            result += f", {_deliveree} was not found"
        
        # Clear the flag after completion
        _langgraph_initiated_move = False

        return result
    except Exception as e:
        _langgraph_initiated_move = False
        return f"Error going to {location_name} while searching for {name}: {str(e)}"

def cancel_action_tool_helper(sdp):
    """Cancel current action."""
    cancelAction(True, sdp)
    return "Action cancelled."

def go_recharge_tool_helper(sdp):
    """Go to the recharge location."""
    global _goal
    cancelAction(True, sdp)
    _goal = "recharge"
    return "Going to recharge dock."

def go_to_location_by_coords_tool_helper(sdp, x: float, y: float, yaw: float = 0.0):
    """Go to a location given coordinates"""
    global _langgraph_initiated_move, _locations, _goal
    try:
        # Mark this as a LangGraph-initiated move
        _langgraph_initiated_move = True

        # Initiate movement
        _locations["custom"] = (x, y, math.radians(yaw))
        _goal = "custom"

        # Monitor the movement and return result (no streaming writer)
        result = moveActionMonitor(sdp, "custom")

        # Clear the flag after completion
        _langgraph_initiated_move = False

        return result
    except Exception as e:
        _langgraph_initiated_move = False
        return f"Error going to coordinates ({x}, {y}, {yaw}): {str(e)}"

def go_to_location_tool_helper(sdp, location_name: str):
    """Go to a specific named location using the existing goToLocation function."""
    global _langgraph_initiated_move
    try:
        # Mark this as a LangGraph-initiated move
        _langgraph_initiated_move = True

        # Initiate movement
        goToLocation(location_name)

        # Monitor the movement and return result (no streaming writer)
        result = moveActionMonitor(sdp, location_name)

        # Clear the flag after completion
        _langgraph_initiated_move = False

        return result
    except Exception as e:
        _langgraph_initiated_move = False
        return f"Error going to {location_name}: {str(e)}"

def go_to_location_with_narration_tool_helper(sdp, location_name: str, narration_interval_seconds: int = 5):
    """Go to a specific named location while periodically describing the scene."""
    global _langgraph_initiated_move

    guard = _require_camera_ai("vlm")
    if guard:
        return guard

    def narrate_scene_during_move(output):
        try:
            _mdai.drawText(output, 1, 14)
            speak(output)
            return False
        except Exception as e:
            print(f"Error during narration: {str(e)}")  
        finally:
            return False

    try:
        # Mark this as a LangGraph-initiated move
        _langgraph_initiated_move = True

        # Initiate movement
        goToLocation(location_name)

        # Monitor the movement with narration
        result = moveActionMonitorWithPrompt(sdp, location_name, "Describe the scene concisely.",
                                             narration_interval_seconds, narrate_scene_during_move)

        # Clear the flag after completion
        _langgraph_initiated_move = False

        return result
    except Exception as e:
        _langgraph_initiated_move = False
        return f"Error going to {location_name} with narration: {str(e)}"

def move_in_dir_dist_tool_helper(sdp, direction: str, distance: float, unit: str = "meters"):
    """Move in specified direction for specified distance."""
    global _langgraph_initiated_move
    try:
        # Mark this as a LangGraph-initiated move
        _langgraph_initiated_move = True

        moveInDirDist(direction, distance, unit, sdp)
        
        # Monitor the movement and return result (no streaming writer)
        result = moveActionMonitor(sdp, "custom")

        # Clear the flag after completion
        _langgraph_initiated_move = False

        return result
    except Exception as e:
        return f"Error moving {direction}: {str(e)}"

def get_loc_of_person_from_voice_tool_helper(sdp):
    """Find person based on voice direction."""
    try:
        doa = _mic_array.getDoa()
        return getLocOfPersonFromSound(doa, sdp)
    except Exception as e:
        return f"Error finding person from voice: {str(e)}"

def search_for_person_tool_helper(sdp):
    """Search for any person by rotating and scanning."""
    global _interrupt_action
    try:
        ps = searchForPerson(sdp)
        if _interrupt_action:
            _interrupt_action = False
        
        if ps and len(ps) > 0:
            # find closest person
            p = min(ps, key=lambda person: person.z)
            closest_person_loc = getLocationOfObj(sdp, "closest person", p, cam_yaw=_move_oak_d.getYaw(), offset_dist=1, radians=False)

            return f"Found {len(ps)} person(s), the closest one is at {closest_person_loc}."
        return "No person found."

    except Exception as e:
        return f"Error searching for person: {str(e)}"
    finally:
        _move_oak_d.allHome()    

def get_known_faces_tool_helper():
    """Get list of all known faces."""
    try:
        faces = get_known_faces()
        if faces:
            return f"Known faces: {', '.join(faces)}"
        return "No faces known."
    except Exception as e:
        return f"Error getting known faces: {str(e)}"

def identify_visible_face_tool_helper(name: str):
    """Identify face if any visible."""
    try:
        found, name = identify_visible_face()
        if found:
            return f"{name} is here"
        else:
            return "No known face is visible"
    except Exception as e:
        return f"Error identifying any face: {str(e)}"

def memorize_a_face(new_name):
    eyes.setTargetPitchYaw(-70, 0)
    shutdown_my_depthai()
    start_facial_recog(new_name=new_name)
    while not _facial_recog.was_face_added():
        time.sleep(0.2)
    shutdown_facial_recog()
    start_depthai_thread()
    eyes.setHome()
    _move_oak_d.allHome()

def memorize_a_face_tool_helper(name: str):
    """Memorize a new face with given name."""
    try:
        memorize_a_face(name)
        return f"Memorized face for {name}"
    except Exception as e:
        return f"Error memorizing face: {str(e)}"

# TBD
def get_object_from_person_tool_helper(sdp, obj: str):
    """Get object from person"""
    global _langgraph_initiated_move
    
    try:
        orig_yaw = sdp.pose().yaw
        _langgraph_initiated_move = True
        result = moveActionMonitor(sdp, "get_obj")
        _langgraph_initiated_move = False
        return result
    except Exception as e:
        return f"Error getting {obj} from user: {str(e)}"

# TBD
def deliver_object_to_person_tool_helper(sdp, obj: str, person: str, loc: str):
    """Deliver object to person at location."""
    global _langgraph_initiated_move
    
    try:
        orig_yaw = sdp.pose().yaw
        _langgraph_initiated_move = True
        result = moveActionMonitor(sdp, "deliver_obj")
        _langgraph_initiated_move = False
        return result
    except Exception as e:
        return f"Error delivering {obj} to {person}: {str(e)}"

def track_object_tool_helper(obj: str, height: str = "eye level", duration: int = 30):
    """Track object via NanoOWL at inference rate with no robot movement. Shows green bbox."""
    global _interrupt_action, _keep_camera_orientation

    guard = _require_camera_ai("owl")
    if guard:
        return guard

    if _nano_owl_mgr is None:
        return "Error: NanoOWL not available."

    if height == "floor":
        aim_oakd(pitch=135)
        eyes.setTargetPitchYaw(-50, 0)
    elif height == "up high":
        aim_oakd(pitch=85)
        eyes.setTargetPitchYaw(50, 0)
    else:
        _move_oak_d.allHome()
        eyes.setHome()

    _nano_owl_mgr.set_prompt(f"[{obj}]")
    _nano_owl_mgr.start_streaming(fps=16)
    _mdai.show_yolo_boxes = False
    _nano_owl_mgr.get_detections_nms()  # wakeup call

    found_count = 0
    total_count = 0
    deadline = time.time() + duration
    next_report = time.time() + 5
    _interrupt_action = False
    poll_interval = 1.0 / 16  # match inference fps

    try:
        _keep_camera_orientation = True
        while time.time() < deadline and not _interrupt_action:
            found, spatials = _nano_owl_mgr.check_for_all_objects(obj)
            total_count += 1
            if found and spatials:
                found_count += 1
                best = max(spatials, key=lambda s: s.z if s.z > 0 else 0)
                _mdai.drawText(f"{obj} x{len(spatials)} z={best.z:.2f}m", 1, 14)
            if time.time() >= next_report:
                elapsed = duration - (deadline - time.time())
                hit_rate = (found_count / total_count * 100) if total_count else 0
                print(f"[track] {elapsed:.0f}s: {found_count}/{total_count} ({hit_rate:.0f}%)")
                next_report += 5
            time.sleep(poll_interval)
    finally:
        _keep_camera_orientation = False
        _mdai.show_yolo_boxes = True
        _nano_owl_mgr.stop_streaming()
        _nano_owl_mgr.clear_prompt()
        _move_oak_d.allHome()
        if _interrupt_action:
            _interrupt_action = False

    elapsed = min(duration, time.time() - (deadline - duration))
    hit_rate = (found_count / total_count * 100) if total_count > 0 else 0
    return (f"Tracked '{obj}' for {elapsed:.0f}s: {found_count}/{total_count} frames detected "
            f"({hit_rate:.0f}%).")


def search_for_object_tool_helper(sdp, obj: str, height: str, rot_clockwise: bool = True):
    """Search for object by rotating in place."""
    global _interrupt_action, _keep_camera_orientation

    guard = _require_camera_ai("owl")
    if guard:
        return guard

    obj_loc = None
    yolo_obj = obj.replace(' ','')
    if yolo_obj in _mdai.labelMap:
        try:
            p = searchForObject(sdp, yolo_obj, height, rot_clockwise)
            if not _interrupt_action and p is None:
                p = searchForObject(sdp, yolo_obj, height, not rot_clockwise)
                
            if _interrupt_action:
                _interrupt_action = False
            if p is not None:
                obj_loc = getLocationOfObj(sdp, yolo_obj, p, cam_yaw=_move_oak_d.getYaw(), offset_dist=0.75, radians=False)

            return f"{obj} "+ (f"was found at {obj_loc}." if p else "was not found.")

        except Exception as e:
            return f"Error searching for object {obj}: {str(e)}"
        finally:
            _move_oak_d.allHome()            
    else:
        # object not handled by YOLO. Use NanoOWL-based object search
        if _nano_owl_mgr is None:
            return f"Error: NanoOWL not available to search for {obj}."

        print("processing with nano owl")
        _nano_owl_mgr.set_prompt(f"[{obj}]")
        _nano_owl_mgr.start_streaming(fps=16)
        _mdai.show_yolo_boxes = False
        _nano_owl_mgr.get_detections_nms()  # wakeup call

        last_spatial = [None]  # mutable container for closure access

        def is_object_found(obj_name):
            try:
                found, spatial = _nano_owl_mgr.check_for_object(obj_name)
                if found:
                    last_spatial[0] = spatial
                    _mdai.drawText(obj_name, 1, 14)
                    print(f"OWL found {obj_name}: z={spatial.z:.2f}m theta={spatial.theta:.1f}deg")
                    return True, spatial
            except Exception as e:
                print(f"Error during OWL object found check: {str(e)}")
            return False, None

        try:
            _keep_camera_orientation = True

            p = searchForObject(sdp, obj, height, rot_clockwise, is_object_found, rot_speed=0.05, min_recheck=True, stream_mgr=_nano_owl_mgr)
            if p is None and not _interrupt_action:
                p = searchForObject(sdp, obj, height, not rot_clockwise, is_object_found, rot_speed=0.05, min_recheck=True, stream_mgr=_nano_owl_mgr)

            if p is None and not _interrupt_action and last_spatial[0] is not None and last_spatial[0].z > 0:
                # Recheck failed but we saw it earlier — move halfway toward last known location and retry
                half_dist = last_spatial[0].z * 0.5
                pose = sdp.pose()
                cam_yaw = _move_oak_d.getYaw()
                angle = math.radians(pose.yaw + cam_yaw + last_spatial[0].theta)
                xt = pose.x + half_dist * math.cos(angle)
                yt = pose.y + half_dist * math.sin(angle)
                print(f"recheck failed, moving halfway ({half_dist:.2f}m) toward last known {obj} location")
                sdp.moveToFloat(xt, yt)
                sdp.waitUntilMoveActionDone()
                # Search again from new position
                p = searchForObject(sdp, obj, height, rot_clockwise, is_object_found, rot_speed=0.05, min_recheck=True, stream_mgr=_nano_owl_mgr)
                if p is None and not _interrupt_action:
                    p = searchForObject(sdp, obj, height, not rot_clockwise, is_object_found, rot_speed=0.05, min_recheck=True, stream_mgr=_nano_owl_mgr)

            if _interrupt_action:
                _interrupt_action = False

            obj_loc = None
            if p is not None and p.z > 0:
                obj_loc = getLocationOfObj(sdp, obj, p, cam_yaw=_move_oak_d.getYaw(), offset_dist=0.75, radians=False)

            if obj_loc:
                return f"{obj} was found at {obj_loc}."
            elif p is not None:
                return f"{obj} was found but depth unavailable, cannot determine location."
            else:
                return f"{obj} was not found."
        except Exception as e:
            return f"Error searching for object {obj}: {str(e)}"
        finally:
            _keep_camera_orientation = False
            _mdai.show_yolo_boxes = True
            _nano_owl_mgr.stop_streaming()
            _nano_owl_mgr.clear_prompt()
            _move_oak_d.allHome()

def while_go_to_loc_find_object_tool_helper(sdp, obj: str, loc: str, height: str = "eye level"):
    """While going to loc, look for object and stop as soon as it is seen and report its coords."""
    global _langgraph_initiated_move, _locations, _user_set_speed, _keep_camera_orientation

    guard = _require_camera_ai("owl")
    if guard:
        return guard

    # Set camera pitch based on height specification
    if height == "floor":
        aim_oakd(pitch=135)
        eyes.setTargetPitchYaw(-50, 0)
    elif height == "up high":
        aim_oakd(pitch=85)
        eyes.setTargetPitchYaw(50, 0)
    else:  # eye level
        _move_oak_d.allHome()
        eyes.setHome()

    sdp.setSpeed(1)  # slow speed while searching

    yolo_obj = obj.replace(' ','')
    if yolo_obj in _mdai.labelMap:
        try:
            orig_yaw = sdp.pose().yaw
            _langgraph_initiated_move = True
            findOrRetrieveObject(loc, yolo_obj, "", "", orig_yaw, sdp=sdp)
            result = moveActionMonitor(sdp, "find_obj")
            _langgraph_initiated_move = False

            obj_loc = _locations.get(yolo_obj, None)
            if obj_loc is not None:
                obj_loc = (obj_loc[0], obj_loc[1], math.degrees(obj_loc[2]))  # convert yaw to degrees for output
                result += f", {obj} is at {obj_loc}. I am not at {loc} or near the {obj}"
                del _locations[yolo_obj]
            else:
                result += f", {obj} was not found and I am now at {loc}."

            return result
        except Exception as e:
            return f"Error finding {obj}: {str(e)}"
        finally:
            _move_oak_d.allHome()
            sdp.setSpeed(_user_set_speed)
    else:
        # object not handled by YOLO. Use NanoOWL-based object search
        if _nano_owl_mgr is None:
            return f"Error: NanoOWL not available to search for {obj}."

        _nano_owl_mgr.set_prompt(f"[{obj}]")
        _nano_owl_mgr.start_streaming(fps=16)
        _mdai.show_yolo_boxes = False
        _nano_owl_mgr.get_detections_nms()  # wakeup call

        try:
            _langgraph_initiated_move = True
            _keep_camera_orientation = True
            goToLocation(loc)
            result, spatial_det = moveActionMonitorWithOwl(sdp, loc, obj)
            _langgraph_initiated_move = False

            if spatial_det is not None and spatial_det.z > 0:
                obj_loc = getLocationOfObj(sdp, obj, spatial_det, cam_yaw=_move_oak_d.getYaw(), offset_dist=0.75, radians=False)
                if obj_loc:
                    return result + f", {obj} is at {obj_loc}. I am not at {loc} or near the {obj}"
                return result + f", {obj} was found but depth unavailable, cannot determine location."
            elif spatial_det is not None:
                return result + f", {obj} was found but depth unavailable, cannot determine location."
            else:
                return result + f", {obj} was not found and I am now at {loc}."
        except Exception as e:
            return f"Error finding {obj}: {str(e)}"
        finally:
            _keep_camera_orientation = False
            _mdai.show_yolo_boxes = True
            _nano_owl_mgr.stop_streaming()
            _nano_owl_mgr.clear_prompt()
            _move_oak_d.allHome()
            sdp.setSpeed(_user_set_speed)

def aim_camera_tool_helper(yaw: int = None, pitch: int = None):
    """Aim camera to specific yaw and/or pitch angles."""
    try:
        aim_oakd(yaw=yaw, pitch=pitch)
        return f"Camera aimed to yaw={yaw}, pitch={pitch}"
    except Exception as e:
        return f"Error aiming camera: {str(e)}"

def home_camera_tool_helper():
    """Return camera to home position."""
    try:
        home_oakd()
        return "Camera returned to home position."
    except Exception as e:
        return f"Error homing camera: {str(e)}"

def take_picture_tool_helper():
    """Take a picture using the camera."""
    try:
        global _mdai
        if not _mdai.rgbWindowVisible():
            _mdai.showRgbWindow(True)
            _mdai.waitUntilChangeFinished()
            time.sleep(1.5)
        _mdai.takePicture()
        return "Picture taken and shown on screen."
    except Exception as e:
        return f"Error taking picture: {str(e)}"

def where_am_i_tool_helper(sdp):
    """Get current location information."""
    try:
        location, distance, close_enough = where_am_i(sdp)
        if close_enough:
            return f"I am at the {location} location."
        return f"I am closest to the {location} location, {distance:.2f} meters away."
    except Exception as e:
        return f"Error getting location: {str(e)}"

def recover_localization_tool_helper(sdp, rect: dict = _WHOLE_MAP_RECT):
    """Recover localization using a given rectangle."""
    try:
        if rect is None:
            rect = _WHOLE_MAP_RECT
        success, errStr = recoverLocalization(sdp, rect)
        if success:
            return "Localization recovered."
        else:
            return f"Error recovering localization: {errStr}"
    except Exception as e:
        return f"Error recovering localization: {str(e)}"

def follow_me_tool_helper():
    """Start following a person."""
    try:
        global _follow_thread
        if _follow_thread is None:
            start_following()
            return "Started following you."
        return "Already following."
    except Exception as e:
        return f"Error starting follow: {str(e)}"

def stop_following_tool_helper():
    """Stop following a person."""
    try:
        stop_following()
        return "Stopped following."
    except Exception as e:
        return f"Error stopping follow: {str(e)}"

def track_me_tool_helper():
    """Start tracking a person with camera."""
    try:
        start_tracking()
        return "Started tracking you."
    except Exception as e:
        return f"Error starting tracking: {str(e)}"

def stop_tracking_tool_helper():
    """Stop tracking a person."""
    try:
        stop_tracking()
        return "Stopped tracking."
    except Exception as e:
        return f"Error stopping tracking: {str(e)}"

def describe_scene_tool_helper():
    """Describe the scene concisely."""
    guard = _require_camera_ai("vlm")
    if guard:
        return guard
    if _nano_vlm is None:
        return "Error: NanoVLM client not available to describe the scene."
    try:
        prompt = "Describe the scene concisely."
        _nano_vlm.set_prompts([prompt])
        answer, _, _ = _nano_vlm.get_output(prompt_filter=prompt)
        _mdai.drawText(answer, 1, 14)
        return answer
    except Exception as e:
        return f"Error describing the scene: {str(e)}"    

def ask_question_about_scene_tool_helper(prompt: str):
    """Ask question about the scene."""
    guard = _require_camera_ai("vlm")
    if guard:
        return guard
    if _nano_vlm is None:
        return "Error: NanoVLM client not available to answer questions about the scene."
    try:
        _nano_vlm.set_prompts([prompt])
        answer, _, _ = _nano_vlm.get_output(prompt_filter=prompt)
        _mdai.drawText(answer, 1, 14)
        return answer
    except Exception as e:
        return f"Error asking about the scene: {str(e)}"

def get_yolo_detections_tool_helper(sdp: MyClient):
    """Get a list of objects and persons (with coordinates) visible in the scene using YOLO model."""
    try:
        detections = _mdai.getPersonDetections()
        detections += _mdai.getObjectDetections()

        results = []

        if len(detections) > 0:
            # list of dicts with 'label', 'confidence', xt, yt, yaw
            for det in detections:
                results.append({
                    'label': det.label,
                    'coords': getLocationOfObj(sdp, det.label, det, cam_yaw=_move_oak_d.getYaw(), offset_dist=0.75, radians=False)
                })
        return results 
    except Exception as e:
        return [{'error': f"Error getting YOLO detections: {str(e)}"}]

def _arm_problem(result: dict) -> str:
    """The readable half of a failed arm result. `output` is the ros2 node's
    console text, which is where a MoveIt refusal explains itself."""
    return (result.get('error') or (result.get('output') or '').strip()
            or "no detail")


def _ensure_arm_enabled() -> str:
    """Bring the ROS arm stack up if it is not already up. Returns "" when the
    arm is ready, otherwise the reason it is not.

    Motion commands deliberately do not auto-enable, so every path that moves
    the arm comes through here. Normally a no-op: initialize_robot enables the
    stack at startup and this only does work if it went away since.
    """
    status = _arm_client.get_status()
    if status is not None and status.get("arm_enabled"):
        return ""
    result = _arm_client.enable_arm()
    return "" if result.get("ok") else _arm_problem(result)


def wave_arm_tool_helper():
    """Wave the robot's arm using the arm client."""
    if _arm_client is None or not _arm_client.is_connected():
        return "Error: Arm client not available to wave the arm."
    if _held_object is not None:
        # The swings run well above picking speed and stow at `init`, so this
        # would both fling the can and leave the arm where the wrist camera
        # cannot be trusted about whether it is still there.
        return (f"I am holding a {_held_object}, so I will not wave -- I would "
                f"fling it.")
    try:
        problem = _ensure_arm_enabled()
        if problem:
            return f"Error: the arm could not be enabled to wave: {problem}"
        result = _arm_client.wave_arm()
        if result.get("ok"):
            return "Waved the arm."
        return f"Error: the arm failed to wave: {_arm_problem(result)}"
    except Exception as e:
        return f"Error waving the arm: {str(e)}"


# --- Pick and place ----------------------------------------------------------
# What the arm can actually do today. Both lists are short on purpose: a tool
# that claims more than pick_can/place_can support would have the planner build
# a plan the arm then refuses halfway through.

# Objects pick_up knows the size of (robot_frames.OBJECT_SIZES) AND the arm has
# a grasp for. Anything else is refused before the robot moves.
PICKABLE_OBJECTS = ("soda can",)

# Where put_down can be asked to deposit. The arm's release motion is the same
# for both -- place_can() drops at a fixed state in front of the robot -- so the
# destination is really a statement about which bin the robot must be parked at,
# and that is what put_down checks.
PLACE_DESTINATIONS = ("recycle bin", "trash bin")

# How close the robot has to be to a bin before dropping something is a place
# and not littering. Roughly the arm's forward span plus nav slop.
PLACE_ARRIVAL_DIST = 1.0

# Head tilt for looking at the floor close in. 145 is the pitch servo's limit;
# at that tilt the frame covers from about 0.4 m in front of the lens outwards,
# so both a can at arm's length and one across the room stay in view.
PICK_PITCH = 145

# Frames to sample before committing a grasp coordinate. One frame's depth ROI
# is noisy enough to miss a 66 mm can; the median of several is not.
PICK_SAMPLES = 7


def _normalize_place_name(name: str) -> str:
    return " ".join(str(name).replace("_", " ").lower().split())


def _find_location_key(name: str):
    """The key in _locations that the agent means by `name`, or None.

    Locations get named by a human ("recycle bin") and referred to by an LLM
    ("recycle_bin", "Recycle Bin"), so match on the normalized form.
    """
    wanted = _normalize_place_name(name)
    for key in _locations:
        if _normalize_place_name(key) == wanted:
            return key
    return None


def _look_for_floor_object(obj: str):
    """Find `obj` on the floor from where the robot stands, without moving.

    Returns (detection, cam_yaw, cam_pitch) for the NEAREST instance, or
    (None, 0, 0). The head is left tilted down; the caller homes it.

    Unlike search_for_object this never rotates the base: pick_up is called
    when the robot is already pointed at the thing, and a base rotation here
    would invalidate the coordinate it is about to hand the arm.
    """
    global _interrupt_action, _keep_camera_orientation

    aim_oakd(pitch=PICK_PITCH)
    eyes.setTargetPitchYaw(-50, 0)

    _nano_owl_mgr.set_prompt(f"[{obj}]")
    _nano_owl_mgr.start_streaming(fps=16)
    _mdai.show_yolo_boxes = False
    _nano_owl_mgr.get_detections_nms()  # wakeup call

    samples = []
    try:
        _keep_camera_orientation = True
        for _ in range(PICK_SAMPLES):
            if _interrupt_action:
                break
            found, spatials = _nano_owl_mgr.check_for_all_objects(obj)
            valid = [s for s in (spatials or []) if s.z > 0]
            if found and valid:
                # Nearest first: collecting cans means collecting this one.
                samples.append(min(valid, key=lambda s: s.z))
            time.sleep(1.0 / 16)
    finally:
        _keep_camera_orientation = False
        _mdai.show_yolo_boxes = True
        _nano_owl_mgr.stop_streaming()
        _nano_owl_mgr.clear_prompt()

    if not samples:
        return None, 0, 0

    # The median SAMPLE, not the median of each axis: with two cans in view the
    # nearest can flip between frames, and averaging would aim between them.
    samples.sort(key=lambda s: s.z)
    return (samples[len(samples) // 2],
            _move_oak_d.getYaw(), _move_oak_d.getPitch())


def _gripper_sees(obj: str):
    """Ask the wrist camera whether `obj` is actually in the jaws.

    Returns (held, why) with held True / False / None. None means the
    question could not be asked -- no detector, a dark frame, a view nobody
    calibrated -- and must NEVER be folded into False: that turns "the
    detector is busy elsewhere" into "you dropped it".

    ONLY ASK THIS WITH THE ARM AT `carry`. It classifies the whole frame as
    empty-gripper or holding-a-can rather than detecting the can, so from any
    pose that can see the floor a can lying there reads as held with high
    confidence -- a confident wrong answer, not a weak one. Carry points the
    tool up and shows no floor, which is what makes the question honest.

    `_held_object is not None` is exactly the window where the arm is at
    carry: it is set only by a completed pick and cleared by every place and
    reset. Every caller here is gated on it, and wave_arm refuses while it is
    set rather than stowing at `init` and quietly breaking that.

    It also goes to the same NanoOWL the scene-description skill takes the
    GPU from, so with the VLM active this answers None rather than lying.
    """
    if _arm_client is None or not _arm_client.is_connected():
        return None, "the arm is not reachable"
    try:
        result = _arm_client.is_holding(obj or "")
    except Exception as e:
        return None, "the look failed: %s" % e
    held = result.get("held")
    why = result.get("reason") or _arm_problem(result)
    if held not in (True, False):
        return None, why
    return held, why


def pick_up_tool_helper(sdp, object_name: str):
    """Look at the floor, and if the object is within reach, grasp and hold it."""
    global _held_object, _interrupt_action

    obj = _normalize_place_name(object_name)
    if obj not in PICKABLE_OBJECTS:
        return (f"I cannot pick up a {object_name}. I can only pick up: "
                f"{', '.join(PICKABLE_OBJECTS)}.")
    if _held_object is not None:
        return (f"I am already holding a {_held_object}, and I can only carry "
                f"one thing at a time.")
    if _arm_client is None or not _arm_client.is_connected():
        return "Error: the arm is not available to pick anything up."
    if _nano_owl_mgr is None:
        return f"Error: NanoOWL not available to locate the {obj}."

    guard = _require_camera_ai("owl")
    if guard:
        return guard

    problem = _ensure_arm_enabled()
    if problem:
        return f"Error: the arm could not be enabled to pick up the {obj}: {problem}"

    try:
        pose = sdp.pose()
        det, cam_yaw, cam_pitch = _look_for_floor_object(obj)

        if _interrupt_action:
            _interrupt_action = False
            return f"Stopped before picking up the {obj}."
        if det is None:
            return f"I do not see a {obj} on the floor in front of me."

        # The depth point is on the near face of the can partway up it; the
        # centre the arm wants comes from the can's own size and the floor.
        target = robot_frames.floor_object_to_arm(
            det.x, det.y, det.z, yaw_deg=cam_yaw, pitch_deg=cam_pitch, obj=obj)
        rng, arm_yaw, height = robot_frames.arm_reach(target)
        print("pick_up: %s at arm (%.3f, %.3f, %.3f), range %.3f m, yaw %.1f deg"
              % (obj, target[0], target[1], target[2], rng, arm_yaw))

        ok, reason = robot_frames.arm_can_reach(target)
        if not ok:
            gx, gy, gyaw = robot_frames.approach_pose(
                robot_frames.arm_to_robot(target), pose)
            return (f"The {obj} is out of reach: {reason}. Squared up on it at "
                    f"a comfortable distance, I would be standing at map "
                    f"x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.0f}.")

        # No is_holding() here: pick_can asks it itself, on approach and again
        # at carry, and aborts on a definite no. So an ok result has already
        # been confirmed through the same camera, and a failure after the
        # grasp is one of the things that check caught -- which is why the
        # failure below reports the arm as stuck rather than inviting a retry.
        result = _arm_client.pick_can(target[0], target[1], target[2], obj)
        if result.get("ok"):
            _held_object = obj
            return f"I picked up the {obj} and am holding it."
        return (f"The arm failed to pick up the {obj}: {_arm_problem(result)}. "
                f"It cannot plan another move until it is reset.")
    except Exception as e:
        return f"Error picking up the {object_name}: {str(e)}"
    finally:
        _move_oak_d.allHome()
        eyes.setHome()


def put_down_tool_helper(sdp, destination: str):
    """Release the held object into the bin the robot is parked at."""
    global _held_object

    if _held_object is None:
        return "I am not holding anything, so there is nothing to put down."

    dest = _normalize_place_name(destination)
    if dest not in PLACE_DESTINATIONS:
        return (f"I cannot put things in a {destination}. I can only put them "
                f"in: {', '.join(PLACE_DESTINATIONS)}.")
    if _arm_client is None or not _arm_client.is_connected():
        return f"Error: the arm is not available to put down the {_held_object}."

    key = _find_location_key(dest)
    if key is None:
        return (f"I have no saved location called '{dest}', so I do not know "
                f"where to take the {_held_object}.")

    # Release drops the object in front of the robot, so being at the bin is
    # the whole of "placing it in the bin" -- check it rather than litter.
    try:
        pose = sdp.pose()
        dist = distance_A_to_B(pose.x, pose.y, _locations[key][0], _locations[key][1])
    except Exception as e:
        return f"Error checking whether I am at the {dest}: {str(e)}"
    if dist > PLACE_ARRIVAL_DIST:
        return (f"I am {dist:.1f} m from the {dest}, too far to drop the "
                f"{_held_object} into it. I have to be within "
                f"{PLACE_ARRIVAL_DIST:.1f} m of it.")

    # A can can shake loose on the drive over, and the planning scene would
    # be none the wiser -- the wrist camera is the only thing that knows.
    # Ask before going through the motions of a place. Only a definite no
    # stops it; an unknown answer is not evidence of a drop.
    seen, why = _gripper_sees(_held_object)
    if seen is False:
        lost = _held_object
        _held_object = None
        return (f"The {lost} is not in the gripper any more -- {why}. It must "
                f"have come loose on the way to the {dest}, so there is "
                f"nothing to put down.")

    problem = _ensure_arm_enabled()
    if problem:
        return (f"Error: the arm could not be enabled to put down the "
                f"{_held_object}: {problem}")

    try:
        held = _held_object
        result = _arm_client.place_can()
        if result.get("ok"):
            _held_object = None
            return f"I put the {held} in the {dest}."
        return (f"The arm failed to put down the {held}: {_arm_problem(result)}. "
                f"I am still holding it, and the arm cannot plan another move "
                f"until it is reset.")
    except Exception as e:
        return f"Error putting down the {_held_object}: {str(e)}"


def get_held_object_tool_helper():
    """What the gripper is carrying, confirmed by looking at it."""
    global _held_object

    if _held_object is None:
        return "I am not holding anything."
    seen, why = _gripper_sees(_held_object)
    if seen is False:
        lost = _held_object
        _held_object = None
        return (f"I thought I was holding a {lost}, but the gripper is "
                f"empty -- {why}.")
    if seen is None:
        return (f"I am holding a {_held_object}, though I could not confirm "
                f"it by looking ({why}).")
    return f"I am holding a {_held_object}, and I can see it in the gripper."


def reset_arm_tool_helper():
    """Recover the arm after a failed pick or place."""
    global _held_object

    if _arm_client is None or not _arm_client.is_connected():
        return "Error: the arm is not available to reset."
    problem = _ensure_arm_enabled()
    if problem:
        return f"Error: the arm could not be enabled to reset: {problem}"
    try:
        held = _held_object
        result = _arm_client.reset_arm()
        # reset_arm opens the gripper before it moves, so whatever was carried
        # is on the floor now whether or not the move that followed succeeded.
        _held_object = None
        if result.get("ok"):
            return ("The arm is reset and back at rest."
                    + (f" It dropped the {held} where it was standing." if held else ""))
        return f"The arm could not be reset: {_arm_problem(result)}"
    except Exception as e:
        return f"Error resetting the arm: {str(e)}"

# --- Camera-AI service switching layer --------------------------------------
# The ONLY code that constructs/connects/disconnects the OWL/VLM clients. All of
# initialize_robot, the enable_* tools, and the inactive-service prompts funnel
# through _activate_camera_ai; the supervisor decides which single GPU service
# is up on the Jetson.

def _connect_nano_owl(max_retries: int = 3, retry_wait: float = 10.0):
    """Construct + connect (+ enable) the NanoOwl client. Sole owl constructor."""
    client = NanoOwlClient()
    for attempt in range(1, max_retries + 1):
        print(f"Attempting to connect to NanoOwl server (attempt {attempt}/{max_retries})...")
        if client.connect():
            print("NanoOwl connection established.")
            client.enable()
            return client
        if attempt < max_retries:
            print(f"Connection failed. Waiting {retry_wait} seconds before retry...")
            time.sleep(retry_wait)
    print("WARNING: No Owl connection available after all retries.")
    return None

def _connect_nano_vlm(max_retries: int = 3, retry_wait: float = 10.0):
    """Construct + connect (+ enable) the NanoVlm client. Sole vlm constructor."""
    client = NanoVlmClient()
    for attempt in range(1, max_retries + 1):
        print(f"Attempting to connect to NanoVLM server (attempt {attempt}/{max_retries})...")
        if client.connect():
            print("NanoVLM connection established.")
            client.enable()
            return client
        if attempt < max_retries:
            print(f"Connection failed. Waiting {retry_wait} seconds before retry...")
            time.sleep(retry_wait)
    print("WARNING: No VLM connection available after all retries.")
    return None

def _connect_camera_ai_client(target: str) -> bool:
    """Tear down the outgoing client and bring up `target`'s client. Sets the
    active-service globals. Returns True on success."""
    global _nano_owl, _nano_vlm, _nano_owl_mgr, _active_camera_ai, _pending_camera_ai

    if target == "vlm":
        # OWL is now stopped on the Jetson.
        _nano_owl_mgr = None
        if _nano_owl is not None:
            try:
                _nano_owl.disconnect()
            except Exception:
                pass
            _nano_owl = None
        _nano_vlm = _connect_nano_vlm()
        ok = _nano_vlm is not None
    else:  # owl
        if _nano_vlm is not None:
            try:
                _nano_vlm.disconnect()
            except Exception:
                pass
            _nano_vlm = None
        _nano_owl = _connect_nano_owl()
        ok = _nano_owl is not None
        if ok and _mdai is not None:
            _nano_owl_mgr = NanoOwlManager(_nano_owl, _mdai)
            print("NanoOwlManager created.")

    _pending_camera_ai = None
    _active_camera_ai = target if ok else None
    return ok

def _activate_camera_ai(target: str, announce: bool = True) -> str:
    """The one shared entry to bring up a camera-AI service. Non-blocking: if a
    real switch is needed it fires switch_to() and returns immediately while a
    background poller (_poll_switch) waits for it to settle, connects the client,
    and announces readiness. Used by initialize_robot and the enable_* tools."""
    global _active_camera_ai, _pending_camera_ai, _nano_owl, _nano_vlm, _nano_owl_mgr

    if target not in ("owl", "vlm"):
        return f"Error: unknown camera skill '{target}'."

    friendly = _CAMERA_AI_FRIENDLY[target]
    secs = _SWITCH_SECONDS.get(target, 45)

    # Already active and connected?
    if _active_camera_ai == target and (_nano_owl if target == "owl" else _nano_vlm) is not None:
        return f"The {friendly} skill is already active."

    # A switch is already in flight.
    if _pending_camera_ai is not None:
        pend = _CAMERA_AI_FRIENDLY.get(_pending_camera_ai, _pending_camera_ai)
        return f"Still switching to the {pend} skill; I'll let you know when it's ready."

    # If the requested service is already the one running on the Jetson, just
    # (re)connect its client synchronously -- no switch, no delay.
    if _supervisor is not None and _supervisor.is_connected() and _supervisor.current() == target:
        if _connect_camera_ai_client(target):
            return f"The {friendly} skill is now active."
        return f"Switched to {friendly} but could not connect its client."

    # No supervisor: we cannot orchestrate a switch. Try to connect the requested
    # client directly (legacy single-service setup); if its service isn't running
    # the connect simply fails.
    if _supervisor is None or not _supervisor.is_connected():
        if _connect_camera_ai_client(target):
            return f"The {friendly} skill is now active."
        return (f"The {friendly} skill is not available and the camera AI cannot be "
                f"switched right now (supervisor unavailable).")

    # Tear down the outgoing client now (its service is about to stop) and mark
    # the switch pending so tools report "not active yet".
    _pending_camera_ai = target
    _active_camera_ai = None
    _nano_owl_mgr = None
    if _nano_owl is not None:
        try:
            _nano_owl.disconnect()
        except Exception:
            pass
        _nano_owl = None
    if _nano_vlm is not None:
        try:
            _nano_vlm.disconnect()
        except Exception:
            pass
        _nano_vlm = None

    if announce:
        speak(f"Switching to the {friendly} skill. This takes about {secs} seconds "
              f"— you can keep talking to me; I'll tell you when it's ready.")

    result = _supervisor.switch_to(target)
    if not result or not result.get("accepted"):
        _pending_camera_ai = None
        return f"Error: the supervisor did not accept the switch to the {friendly} skill."

    # Wait for the switch to settle in the background so the listen loop stays
    # free; _poll_switch connects the client and announces readiness.
    Thread(target=_poll_switch, args=(target,), name="camera-ai-switch",
           daemon=True).start()
    return f"Switching to the {friendly} skill; I'll announce when it's ready."

def _tts_idle() -> bool:
    """True when the robot is not currently speaking."""
    try:
        return bool(_voice.voice.WaitUntilDone(0))
    except Exception:
        return True

def _safe_to_announce() -> bool:
    """Safe to voice a switch announcement when the robot isn't speaking and
    isn't in an interactive user-speech capture window. A silent long command
    run (movement/vision) counts as safe."""
    return _tts_idle() and not _awaiting_user_response

def _poll_switch(target: str):
    """Background poller (one short-lived thread per switch): wait for the fired
    switch to settle via get_status(), then connect the client and voice a
    queued announcement once it's safe to speak. Blocking here is fine -- it's
    off the listen loop -- so we avoid a persistent callback server."""
    global _pending_camera_ai, _active_camera_ai
    try:
        friendly = _CAMERA_AI_FRIENDLY.get(target, target)
        status = _supervisor.wait_until_settled() if _supervisor is not None else None
        phase = status.get("phase") if status else None

        if phase == "up":
            _connect_camera_ai_client(target)  # tools usable immediately
            msg = f"Pardon me, the {friendly} skill is now ready."
        elif phase == "rebooting":
            _pending_camera_ai = None
            _active_camera_ai = None
            msg = (f"Pardon me, the {friendly} skill failed to load and the Jetson "
                   f"is restarting. Please try again shortly.")
        else:
            _pending_camera_ai = None
            _active_camera_ai = None
            msg = (f"Pardon me, the {friendly} skill did not come up "
                   f"(status: {phase}). Please try again.")

        while _run_flag and not _safe_to_announce():
            time.sleep(0.25)
        speak(msg)
    except Exception as e:
        print(f"Switch poll error: {e}")
        _pending_camera_ai = None

def _require_camera_ai(target: str):
    """Return None if `target` is the active camera AI, else a message telling
    the agent to ask the user to switch (same switch the enable_* tools run)."""
    if _active_camera_ai == target:
        return None
    friendly = _CAMERA_AI_FRIENDLY[target]
    secs = _SWITCH_SECONDS.get(target, 45)
    enable_tool = "enable_scene_description_skill" if target == "vlm" else "enable_object_search_skill"
    if _pending_camera_ai == target:
        return f"The {friendly} skill is still starting up; please wait a moment and try again."
    if _supervisor is None or not _supervisor.is_connected():
        return (f"The {friendly} skill is not active and the camera AI cannot be "
                f"switched right now (supervisor unavailable).")
    return (f"The {friendly} skill is not active right now. Switching to it takes "
            f"about {secs} seconds, during which the other camera skill is "
            f"unavailable. Ask the user whether to switch; if they agree, call "
            f"{enable_tool} and then retry this request.")

def _disconnect_camera_ai():
    """Tear down all camera-AI resources. Used by shutdown_robot."""
    global _nano_owl, _nano_vlm, _nano_owl_mgr, _supervisor
    _nano_owl_mgr = None
    if _nano_owl is not None:
        print("disconnecting nano owl client")
        try:
            _nano_owl.disconnect()
        except Exception:
            pass
        _nano_owl = None
    if _nano_vlm is not None:
        print("disconnecting nano vlm client")
        try:
            _nano_vlm.disconnect()
        except Exception:
            pass
        _nano_vlm = None
    if _supervisor is not None:
        try:
            _supervisor.disconnect()
        except Exception:
            pass
        _supervisor = None

def enable_scene_description_tool_helper():
    """Switch the camera AI to scene description (NanoVLM)."""
    return _activate_camera_ai("vlm", announce=True)

def enable_object_search_tool_helper():
    """Switch the camera AI to open-vocabulary object search (NanoOWL)."""
    return _activate_camera_ai("owl", announce=True)

# Export dictionary for LangGraph tools
langgraph_tool_funcs = {
    # Location & Navigation
    "list_locations": list_locations_tool_helper,
    "go_to_location_by_coords": go_to_location_by_coords_tool_helper,
    "go_to_location": go_to_location_tool_helper,
    "where_am_i": where_am_i_tool_helper,
    "recover_localization": recover_localization_tool_helper,
    "move_in_dir_dist": move_in_dir_dist_tool_helper,
    "go_recharge": go_recharge_tool_helper,
    "cancel_action": cancel_action_tool_helper,
    
    # Object Detection & Retrieval
    "track_object": track_object_tool_helper,
    "while_go_to_loc_find_object": while_go_to_loc_find_object_tool_helper,
    "search_for_object": search_for_object_tool_helper,
    "go_to_location_with_narration": go_to_location_with_narration_tool_helper,
    "get_yolo_detections": get_yolo_detections_tool_helper,

    #"deliver_object_to_person": deliver_object_to_person_tool_helper,
    #"get_object_from_person": get_object_from_person_tool_helper,

    # Scene description & VLM
    "describe_scene": describe_scene_tool_helper,
    "ask_question_about_scene": ask_question_about_scene_tool_helper,

    # Camera-AI service switching (Jetson supervisor)
    "enable_scene_description": enable_scene_description_tool_helper,
    "enable_object_search": enable_object_search_tool_helper,

    # Person Detection & Recognition
    "identify_visible_face": identify_visible_face_tool_helper,
    "memorize_a_face": memorize_a_face_tool_helper,
    "search_for_person": search_for_person_tool_helper,
    "search_for_face": search_for_face_tool_helper,
    "get_known_faces": get_known_faces_tool_helper,
    "while_go_to_location_find_face": while_go_to_location_find_face_tool_helper,
    "get_loc_of_person_from_voice": get_loc_of_person_from_voice_tool_helper,

    # Camera Control
    "aim_camera": aim_camera_tool_helper,
    "home_camera": home_camera_tool_helper,
    "take_picture": take_picture_tool_helper,
    
    # Tracking & Following
    "follow_me": follow_me_tool_helper,
    "stop_following": stop_following_tool_helper,
    "track_me": track_me_tool_helper,
    "stop_tracking": stop_tracking_tool_helper,
    
    # Movement primitives
    "move_by_deltas": move_by_deltas_tool_helper,
    "forward": forward,
    "backup": backup,
    "turn": turn,

    # Arm control
    "wave_arm": wave_arm_tool_helper,
    "pick_up": pick_up_tool_helper,
    "put_down": put_down_tool_helper,
    "get_held_object": get_held_object_tool_helper,
    "reset_arm": reset_arm_tool_helper
}

################################################################   
# This is where data gets initialized from information stored on disk
# and threads get started
def initialize_robot():
    global _moods, _internet, _eyes_flag, _facial_recog
    global _listen_thread, _cmdEmbedMgr, _langgraph, _nano_vlm, _nano_owl, _nano_owl_mgr
    global _arm_client, _supervisor, _active_camera_ai

    _internet = True

    if _jetson_on:
        max_retries = 3
        retry_wait = 10  # seconds

        # Initialize Arm client with retry logic
        _arm_client = ArmClient()
        for attempt in range(1, max_retries + 1):
            print(f"Attempting to connect to Arm server (attempt {attempt}/{max_retries})...")
            if _arm_client.connect():
                print("Arm connection established.")
                # Reaching the server does not mean the arm can move: the ROS
                # stack is started explicitly. Do it now so the first wave is
                # not also paying for the launch.
                problem = _ensure_arm_enabled()
                if problem:
                    print(f"WARNING: arm reachable but its ROS stack would not start: {problem}")
                else:
                    print("Arm stack enabled.")
                break
            else:
                if attempt < max_retries:
                    print(f"Connection failed. Waiting {retry_wait} seconds before retry...")
                    time.sleep(retry_wait)
                else:
                    print("WARNING: No Arm connection available after all retries.")
                    _arm_client = None

        # Connect to the service supervisor. The camera-AI layer
        # (_activate_camera_ai) owns all owl/vlm client lifecycle from here on;
        # the actual client is brought up below, after _mdai is ready (needed for
        # the owl manager). Switch progress is tracked by polling (_poll_switch).
        _supervisor = SupervisorClient()
        if _supervisor.connect():
            current = _supervisor.current()
            _active_camera_ai = current if current in ("owl", "vlm") else "owl"
            print(f"Supervisor connected; active camera AI reported: {current}")
        else:
            print("WARNING: service supervisor not available; camera AI switching disabled.")
            _supervisor = None
            _active_camera_ai = "owl"

    start_button_pad_thread()

    start_depthai_thread()
    #start_blazepose_thread()

    # Bring up the camera-AI service now that _mdai exists. Boot default is owl;
    # if the Jetson is already on it this connects synchronously, otherwise a
    # switch is fired and readiness is announced via the callback.
    if _jetson_on:
        target = _active_camera_ai if _active_camera_ai in ("owl", "vlm") else "owl"
        print(_activate_camera_ai(target, announce=False))

    #_cmdEmbedMgr = CmdEmbedMgr()
    #_cmdEmbedMgr.load_cmds_embeddings()
    _listen_thread = Thread(target = listen, name = "Listen")
    _listen_thread.start()

    _langgraph = None
    while _langgraph is None:
        _langgraph = RobotPlannerGraph.create_langgraph(langgraph_tool_funcs, speak_function=speak, 
                                                        wait_until_speech_done = wait_until_speech_done)
        if _langgraph is None:
            print("LangGraph could not be created, retrying in 5 seconds.")
            time.sleep(5)
            print("retrying creating language graph")
    print("LangGraph created.")

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

################################################################
# This is where data gets saved to disk
# and by setting _run_flag to False, threads are told to terminate
def shutdown_robot():
    global _run_flag, _moods, _sdp, _grasper, _sdp, _lpArduino, _cmdEmbedMgr, _map_proc
    global _arm_client, _nano_owl, _nano_vlm

    cancelAction(True, _sdp)
    _run_flag = False

    # close map process if running
    if _map_proc is not None:
        _map_proc.terminate()
        _map_proc = None

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
    if _arm_client is not None:
        print("parking the arm and stopping its ROS stack")
        result = _arm_client.disable_arm("init")
        if not result.get("ok"):
            print(f"WARNING: the arm did not shut down cleanly: {_arm_problem(result)}")
        print("disconnecting arm client")
        _arm_client.disconnect()
        _arm_client = None
    print("shutting down camera AI (owl/vlm/supervisor)")
    _disconnect_camera_ai()
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
    if _langgraph is not None:
        print("shutting down langgraph agent")
        _langgraph.shutdown()   # cancels any run, joins stream thread, releases its client
    manager.shutdown_all()  # tears down 'main', the shared reader, and any stragglers
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
    """Microphone array interface with simulation fallback.
    
    If the USB microphone array device is not available, operates in
    simulation mode with default values.
    """
    
    def __init__(self):
        self._sim_mode = False
        self.dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
        if not self.dev:
            print("WARNING: Mic array not found. Running in simulation mode.")
            self._sim_mode = True
            self.tuning = None
        else:
            self.tuning = Tuning(self.dev)

    def is_sim_mode(self) -> bool:
        """Check if running in simulation mode."""
        return self._sim_mode

    def getDoa(self) -> int:
        """Get direction of arrival in degrees (0-359).
        
        In sim mode, returns 90 (straight ahead).
        """
        if self._sim_mode:
            return 90
        return self.tuning.direction
    
    def getIsSpeech(self) -> bool:
        """Check if speech is detected.
        
        In sim mode, always returns False.
        """
        if self._sim_mode:
            return False
        return self.tuning.is_speech()

    def doa2YawDelta(self, doa) -> int:
        """Convert DOA to yaw delta from forward direction."""
        yawDelta = doa - 90
        if yawDelta >= 180:
            yawDelta = yawDelta - 360
        return yawDelta

    def rotateToDoa(self, doa, sdp) -> int:
        """Rotate robot to face direction of arrival.
        
        In sim mode, does nothing and returns 0.
        """
        if self._sim_mode:
            print("[SIM] rotateToDoa called - no rotation in sim mode")
            return 0
        yawDelta = self.doa2YawDelta(doa)
        print("turning toward where heard person")
        turn(yawDelta, sdp)
        return yawDelta

    def close(self):
        """Release USB resources."""
        if not self._sim_mode and self.dev:
            usb.util.dispose_resources(self.dev)

# def init_local_speech_rec():
#     # Start an in-process edge recognizer using SAPI.
#     winspeech.initialize_recognizer(winspeech.INPROC_RECOGNIZER)
    
def save_locations(name):
    with open(name + ".pkl", "wb") as f:
        pickle.dump(_locations, f)

def load_locations(name):
    global _locations
    try:
        with open(name + ".pkl", "rb") as f:
            _locations = pickle.load(f)
    except Exception as e:
        speak("Could not load locations.")
        print(e)

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
            saveMap(_current_map_name, sdp)
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

def run(no_move=False):
    global _sdp, _slamtec_on, _move_oak_d, _mic_array, _pixel_ring, _lpArduino, _radar, _grasper, _grasper_sonar

    # Start 32 bit bridge server (main-thread command client; connect below, conditionally)
    _sdp = manager.dedicated('main', connect=False)
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

    if no_move:
        res = -1   
        sdp_comm.setSlamtecOn(False)    
    else:
        res = sdp_comm.connectToSdp(_sdp)

    if _execute:
        if (res == 0):
            _slamtec_on = True
            _sdp.setMapUpdate(True)
            pose = _sdp.pose()
            _locations["home"] = (pose.x, pose.y, math.radians(pose.yaw))
            loadMap(_default_map_name, _sdp)
            None
        else:
            speak("Movement is disabled.")
    
    
        robot()
    else:
        print("Program is in DEBUG mode.")
        #initialize_robot()

#import getopt

def main(argv):
    import getopt
    no_move = False
    try:
        opts, args = getopt.getopt(argv, "hnj", ["help", "no-move", "no-jetson"])
    except getopt.GetoptError:
        print('Usage: python main.py [-n|--no-move] [-j | --no-jetson]')
        sys.exit(2)
        
    for opt, arg in opts:
        if opt in ('-h', '--help'):
            print('Usage: python main.py [-n|--no-move] [-j | --no=jetson]')
            print('Options:')
            print('  -n, --no-move    Disable actual movement commands')
            print('  -j, --no-jetson  Disable Jetson-related features')
            sys.exit()
        elif opt in ("-n", "--no-move"):
            no_move = True
            print("Movement commands disabled")
        elif opt in ("-j", "--no-jetson"):
            global _jetson_on
            _jetson_on = False
            print("Jetson-related features disabled")
        
    while 1:
        run(no_move=no_move)
        if not _restart_flag:
           break 

if __name__ == "__main__":
   main(sys.argv[1:])
