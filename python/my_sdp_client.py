# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 16:23:18 2019

@author: bjwei
"""

# my_sdp_client.py

import os  # added this import
from msl.loadlib import Client64
MAX_ATTEMPTS = 3
from my_sdp_server import POSE, LASER_POINTS, SENSORVALUE, ActionStatus
import time

def get_decorator(errors=(Exception, ), default_value=''):
    
    def decorator(func):
    
        def new_func(*args, **kwargs):
            if args[0].connected:
                attempt = 1
                while attempt <= MAX_ATTEMPTS:
                    try:
                        return func(*args, **kwargs)
                    except errors as e:
                        print(e)
                        attempt += 1
                        time.sleep(0.1)
            return default_value
        return new_func
    
    return decorator

poseDecorator = get_decorator(default_value=POSE(x=0,y=0,yaw=0))
moveActionStatusDecorator = get_decorator(default_value=ActionStatus.Running)
noReturnDecorator = get_decorator(default_value=None)
batteryReturnDecorator = get_decorator(default_value=50)
getBatteryIsChargingDecorator = get_decorator(default_value = False)
getIntDecorator = get_decorator(default_value = 0)
getFloatDecorator = get_decorator(default_value = 0.0)
getStringDecorator = get_decorator(default_value = "")
getLaserPointsDecorator = get_decorator(default_value=LASER_POINTS(size=0))
getSensorValueDecorator = get_decorator(default_value=SENSORVALUE(time=0,value=0.0))

class MyClient(Client64):
    """Send a request to 'MyServer' to execute the methods and get the response."""

    def __init__(self):
        # Specify the name of the Python module to execute on the 32-bit server (i.e., 'my_sdp_server')
        super(MyClient, self).__init__(
            module32='my_sdp_server',
            timeout=20,
            append_environ_path=os.path.abspath(os.path.dirname(__file__))  # IMPORTANT!
        )
        self.connected = False

    def connectSlamtec(self, ip_address, port, errStr, errStrLen):
        # The Client64 class has a 'request32' method to send a request to the 32-bit server.
        result = self.request32('connectSlamtec', ip_address, port, errStr, errStrLen)
        if result == 0:
            self.connected = True
        return result

    @noReturnDecorator
    def disconnect(self):
        self.request32('disconnect');
        self.connected = False
        
    @noReturnDecorator
    def forward(self):
        self.request32('forward')

    @noReturnDecorator
    def left(self):
        self.request32('left')

    @noReturnDecorator
    def right(self):
        self.request32('right')

    @noReturnDecorator
    def back(self):
        self.request32('back')

    @noReturnDecorator
    def moveTosFloatWithYaw(self, locations, yaw):
        self.request32('moveTosFloatWithYaw', locations, yaw)

    @noReturnDecorator
    def moveTosFloat(self, locations):
        self.request32('moveTosFloat', locations)

    @noReturnDecorator
    def moveToFloatWithYaw(self, x, y, yaw):
        self.request32('moveToFloatWithYaw', x, y, yaw)

    @noReturnDecorator
    def moveToFloat(self, x, y):
        self.request32('moveToFloat', x, y)

    @noReturnDecorator
    def moveToInteger(self, x, y):
        self.request32('moveToInteger', x, y)

    @noReturnDecorator
    def rotateWithOpt(self, rads, moveOption):
        self.request32('rotateWithOpt', rads, moveOption)

    @noReturnDecorator
    def rotate(self, rads):
        self.request32('rotate', rads)

    @noReturnDecorator
    def rotateToWithOpt(self, rads, moveOption):
        self.request32('rotateToWithOpt', rads, moveOption)

    @noReturnDecorator
    def rotateTo(self, rads):
        self.request32('rotateTo', rads)

    @noReturnDecorator
    def wakeup(self):
        self.request32('wakeup')

    @noReturnDecorator
    def cancelMoveAction(self):
        self.request32('cancelMoveAction')

    @moveActionStatusDecorator
    def getMoveActionStatus(self):
        return self.request32('getMoveActionStatus')

    @getStringDecorator
    def getMoveActionError(self):
        return self.request32('getMoveActionError')
    
    @moveActionStatusDecorator
    def waitUntilMoveActionDone(self):
        return self.request32('waitUntilMoveActionDone')
    
    @getBatteryIsChargingDecorator
    def getBatteryIsCharging(self):
        return self.request32('getBatteryIsCharging')

    @getIntDecorator
    def getBoardTemperature(self):
        return self.request32('getBoardTemperature')

    @batteryReturnDecorator
    def battery(self):
        return self.request32('battery')

    def odometry(self):
        return self.request32('odometry')

    @getIntDecorator
    def getLocalizationQuality(self):
        return self.request32('getLocalizationQuality')

    @poseDecorator
    def pose(self):
        """Returns x, y, and yaw angle in degrees via POSE class""" 
        return self.request32('pose')

    @getFloatDecorator
    def heading(self):
        return self.request32('heading')

    @noReturnDecorator
    def home(self):
        self.request32('home')

    @getIntDecorator
    def getSpeed(self):
        return self.request32('getSpeed')

    @getIntDecorator
    def setSpeed(self, speed):
        return self.request32('setSpeed', speed)

    @getLaserPointsDecorator
    def getLaserScan(self):
        return self.request32('getLaserScan')

    @getIntDecorator
    def clearSlamtecMap(self):
        return self.request32('clearSlamtecMap')
    
    @getIntDecorator
    def loadSlamtecMap(self, filename):
        return self.request32('loadSlamtecMap', filename)

    @getIntDecorator
    def saveSlamtecMap(self, filename):
        return self.request32('saveSlamtecMap', filename)
    
    @moveActionStatusDecorator
    def recoverLocalization(self, left, bottom, width, height):
        return self.request32('recoverLocalization', left, bottom, width, height)
    
    @getIntDecorator
    def setMapUpdate(self, enable):
        return self.request32('setMapUpdate', enable)
    
    @noReturnDecorator
    def getMapUpdate(self):
        return self.request32('getMapUpdate')

    @getSensorValueDecorator
    def getSensorValue(self, id):
        return self.request32('getSensorValue', id)
"""
USAGE:
    First launch server and then client from a terminal window,
    then launch python while still in that terminal window,
    then in the REPL or code:
        from my_sdp_client import MyClient
        c = MyClient()
        c.connectSlamtec("192.168.11.1", 1445, None, 0)
        c.loadSlamtecMap() # Note: The map is C:/SlamMap/House.stcm
        c.forward()
        c.moveToInteger(1000, 0)
        c.rotate(45) # integer degrees
        c.cancel()
"""

if __name__ == '__main__':
    c = MyClient()
    c.shutdown_server32()
