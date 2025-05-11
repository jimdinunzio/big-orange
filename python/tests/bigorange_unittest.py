# -*- coding: utf-8 -*-
"""
Created on Sat Jan  2 19:02:06 2021

@author: LattePanda
"""
import time
from my_sdp_client import MyClient
from my_sdp_server import *
from ctypes import *

def getMoveActionStatus():
    status = c.getMoveActionStatus()
    print(status)
    if status == ActionStatus.Finished:
        print("move action finished.")
    elif status == ActionStatus.Error:
        errStr = c.getMoveActionError()
        print("move action error: ", errStr)
        if errStr != None:
            libc.free(errStr)        
    elif status == ActionStatus.Stopped:
        print("action has been cancelled.")
    return status

# clib free() on windows
libc = cdll.msvcrt
libc.free.argtypes = (c_void_p,)

errStr = create_string_buffer(255)
c = MyClient()
openErr = c.connectSlamtec(b'192.168.11.1', 1445, errStr.raw, 255)
if openErr != 0:
    print(err)
    print(errStr)

c.forward()
c.left()
c.right()
c.back()
c.moveToFloat(0.0, 0.0)
#c.moveToInteger(0,0)
while(1):
    maStatus = getMoveActionStatus()
    if maStatus == ActionStatus.Stopped or \
        maStatus == ActionStatus.Error or \
        maStatus == ActionStatus.Finished:
        break
    time.sleep(0.5)

pose = c.pose()
print(pose.x, pose.y, pose.yaw)
c.wakeup()
res = c.setSpeed(3)
print("setSpeed result: ", res)
print("speed = ", c.getSpeed())
#c.rotate(3.14)
#c.waitUntilMoveActionDone()
print("rotate done")
print("battery = ", c.battery())
print("odometry = ", c.odometry())
#c.getLaserScan()
res = c.saveSlamtecMap(b'test.stcm')
print("save map result: ", res)
res = c.clearSlamtecMap()
print("clear map result: ", res)
c.recoverLocalization(0.0, 0.0, 3.0, 3.0)
c.setUpdate(1)
if (openErr == 0):
    c.disconnect()

