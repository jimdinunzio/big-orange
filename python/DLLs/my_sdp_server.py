# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 16:14:21 2019

@author: bjwei
"""

# my_sdp_server.py

from msl.loadlib import Server32
from ctypes import *
from enum import Enum

class LASER_POINTS(Structure):
	"""Structure to hold laser points"""
	_fields_ = [("size", c_int),
                ("angle", c_float * 360), 
                ("distance", c_float * 360)]

class POSE(Structure):
    """Structure to hold x and y coords, and yaw angle in degrees"""
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("yaw", c_float)]

class MOVEOPTIONS(Structure):
    """Structure to hold Move Options for movement commands"""
    _fields_ = [("flag", c_int),
                ("speed_ratio", c_double)]
    
class MoveOptionFlag(Enum):
    """Enumerated type for Move Option flags field"""
    def __init__(self, number):
        self._as_parameter__ = number

    MoveOptionFlagNone = 0
    MoveOptionFlagAppending = 1
    MoveOptionFlagMilestone = 2
    MoveOptionFlagNoSmooth  = 4
    MoveOptionFlagKeyPoints = 8
    MoveOptionFlagPrecise   = 16
    MoveOptionFlagWithYaw   = 32
    MoveOptionFlagReturnUnreachableDirectly = 64
    MoveOptionFlagKeyPointsWithOA = 0x00000080

class ActionStatus(Enum):
    """Enumerated type for status of moving actions"""
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

class MyServer(Server32):
    """A wrapper around a 32-bit C++ library, 'SlamtecDll.dll', that has functions to control the robot."""

    def __init__(self, host, port, **kwargs):
        # Load the 'cpp_lib32' shared-library file using ctypes.CDLL.
        super(MyServer, self).__init__('SlamtecDll.dll', 'cdll', host, port)
        print("A Server was created.")
        #Set up C function input and output types

        self.lib.connectSlamtec.argtypes = c_char_p, c_int, c_char_p    
        self.lib.loadSlamtecMap.argtypes = c_char_p,
        self.lib.loadSlamtecMap.restype = c_int

        self.lib.waitUntilMoveActionDone.restype = ActionStatus
        self.lib.moveToFloatWithYaw.argtypes = c_float, c_float, c_float
        self.lib.moveToFloatWithYaw.restype = None
        self.lib.moveToFloat.argtypes = c_float, c_float
        self.lib.moveToFloat.restype = None
        self.lib.home.restype = None
        self.lib.disconnect.restype = None
        self.lib.getMoveActionStatus.restype = ActionStatus
        self.lib.rotateWithOpt.argtypes = c_float, MOVEOPTIONS
        self.lib.rotate.argtypes = c_float,
        self.lib.rotateToWithOpt.argtypes = c_float, MOVEOPTIONS
        self.lib.rotateTo.argtypes = c_float,
        self.lib.recoverLocalization.restype = ActionStatus
        self.lib.recoverLocalization.argtypes = c_float, c_float, c_float, c_float
        self.lib.getMoveActionError.restype = c_char_p
        self.lib.pose.restype = POSE
        self.lib.getLaserScan.restype = LASER_POINTS
        self.lib.freeIt.argtypes = c_void_p,
        self.lib.freeIt.restype = None
        self.lib.getBatteryIsCharging.restype = c_bool

    def connectSlamtec(self, ip_address, port, errStr, errStrLen):
        # The Server32 class has a 'lib' property that is a reference to the ctypes.CDLL object.
        retval = self.lib.connectSlamtec(ip_address, port, errStr, errStrLen)
        return retval

    def disconnect(self):
        self.lib.disconnect();
    
    def forward(self):
        self.lib.forward()

    def left(self):
        self.lib.left()
    
    def right(self):
        self.lib.right()

    def back(self):
        self.lib.back()

    def moveToFloat(self, x, y):
        self.lib.moveToFloat(x, y)

    def moveToFloatWithYaw(self, x, y, yaw):
        self.lib.moveToFloatWithYaw(x, y, yaw)

    def moveToInteger(self, x, y):
        self.lib.moveToInteger(x, y)

    def rotateToWithOpt(self, rads, moveOptions):
        self.lib.rotateToWithOpt(rads, moveOptions)

    def rotateTo(self, rads):
        self.lib.rotateTo(rads)

    def rotateWithOpt(self, rads, moveOptions):
        self.lib.rotateWithOpt(rads, moveOptions)

    def rotate(self, rads):
        self.lib.rotate(rads)

    def wakeup(self):
        self.lib.wakeup()

    def cancelMoveAction(self):
        self.lib.cancelMoveAction()

    def getMoveActionStatus(self):
        return self.lib.getMoveActionStatus()

    # returns ptr to new char array, must free with libc.free()
    def getMoveActionError(self):
        _str = self.lib.getMoveActionError()
        str = cast(_str, c_char_p).value
        self.lib.freeIt(_str)
        return str

    def waitUntilMoveActionDone(self):
        return self.lib.waitUntilMoveActionDone()
    
    def battery(self):
        return self.lib.battery()

    def getBatteryIsCharging(self):
        return self.lib.getBatteryIsCharging()

    def getBoardTemperature(self):
        return self.lib.getBoardTemperature()

    def getLocalizationQuality(self):
        return self.lib.getLocalizationQuality()
    
    def odometry(self):
        return self.lib.odometry()
        
    def pose(self):
        return self.lib.pose()
    
    def home(self):
        return self.lib.home()
    
    def getSpeed(self):
        return self.lib.getSpeed()

    def setSpeed(self, speed):
        return self.lib.setSpeed(speed)

    def getLaserScan(self):
        return self.lib.getLaserScan()
    
    def clearSlamtecMap(self):
        return self.lib.clearSlamtecMap()
    
    def loadSlamtecMap(self, filename):
        return self.lib.loadSlamtecMap(filename)
    
    def saveSlamtecMap(self, filename):
        return self.lib.saveSlamtecMap(filename)
    
    def recoverLocalization(self, left, bottom, width, height):
        return self.lib.recoverLocalization(left, bottom, width, height)

    def setUpdate(self, enable):
        return self.lib.setUpdate(enable)
    