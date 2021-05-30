# -*- coding: utf-8 -*-
"""
Created on Sun Jan  6 16:23:18 2019

@author: bjwei
"""

# my_sdp_client.py

import os  # added this import
from msl.loadlib import Client64
MAX_ATTEMPTS = 3
from my_sdp_server import POSE
import time

def get_decorator(errors=(Exception, ), default_value=''):
    
    def decorator(func):
    
        def new_func(*args, **kwargs):
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

class MyClient(Client64):
    """Send a request to 'MyServer' to execute the methods and get the response."""

    def __init__(self):
        # Specify the name of the Python module to execute on the 32-bit server (i.e., 'my_sdp_server')
        super(MyClient, self).__init__(
            module32='my_sdp_server',
            append_environ_path=os.path.abspath(os.path.dirname(__file__)),  # IMPORTANT!
            quiet=False,  # so that you see your print("A Server was created.") command, optional
        )
        
    def connectSlamtec(self, ip_address, port, errStr, errStrLen):
        # The Client64 class has a 'request32' method to send a request to the 32-bit server.
        return self.request32('connectSlamtec', ip_address, port, errStr, errStrLen)

    def disconnect(self):
        self.request32('disconnect');
        
    def forward(self):
        self.request32('forward')

    def left(self):
        self.request32('left')

    def right(self):
        self.request32('right')

    def back(self):
        self.request32('back')

    def moveToFloatWithYaw(self, x, y, yaw):
        self.request32('moveToFloatWithYaw', x, y, yaw)

    def moveToFloat(self, x, y):
        self.request32('moveToFloat', x, y)

    def moveToInteger(self, x, y):
        self.request32('moveToInteger', x, y)

    def rotateWithOpt(self, rads, moveOption):
        self.request32('rotateWithOpt', rads, moveOption)

    def rotate(self, rads):
        self.request32('rotate', rads)

    def rotateToWithOpt(self, rads, moveOption):
        self.request32('rotateToWithOpt', rads, moveOption)

    def rotateTo(self, rads):
        self.request32('rotateTo', rads)

    def wakeup(self):
        self.request32('wakeup')

    def cancelMoveAction(self):
        self.request32('cancelMoveAction')

    def getMoveActionStatus(self):
        return self.request32('getMoveActionStatus')

    def getMoveActionError(self):
        return self.request32('getMoveActionError')
    
    def waitUntilMoveActionDone(self):
        return self.request32('waitUntilMoveActionDone')
    
    def battery(self):
        return self.request32('battery')

    def odometry(self):
        return self.request32('odometry')

    @poseDecorator
    def pose(self):
        """Returns x, y, and yaw angle in degrees via POSE class""" 
        return self.request32('pose')

    def home(self):
        self.request32('home')

    def getSpeed(self):
        return self.request32('getSpeed')

    def setSpeed(self, speed):
        return self.request32('setSpeed', speed)

    def getLaserScan(self):
        return self.request32('getLaserScan')

    def clearSlamtecMap(self):
        return self.request32('clearSlamtecMap')
    
    def loadSlamtecMap(self, filename):
        return self.request32('loadSlamtecMap', filename)

    def saveSlamtecMap(self, filename):
        return self.request32('saveSlamtecMap', filename)
    
    def recoverLocalization(self, left, bottom, width, height):
        return self.request32('recoverLocalization', left, bottom, width, height)
    
    def setUpdate(self, enable):
        return self.request32('setUpdate', enable)


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
