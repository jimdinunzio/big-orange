import sys
from xml.etree.ElementTree import PI
sys.path.append('..')

from my_sdp_client import MyClient
from my_sdp_server import *
import sdp_comm
import math
import time
from latte_panda_arduino import LattePandaArduino
from pyfirmata import util as pyfirmata_util, Pin

_sonar_grasp_offset = -0.063 # distance to back of grasper
_last_grasper_sonar : float = 4.50
_grasper_sonar : Pin

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

def getGrasperSonar() -> float:
    global _last_grasper_sonar
    duration = _grasper_sonar.ping()
    if duration:
        _last_grasper_sonar = pyfirmata_util.ping_time_to_distance(duration)
    return _last_grasper_sonar

def getGraspDist() -> float:
    return getGrasperSonar() + _sonar_grasp_offset    

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

def moveForward(sdp, dist):
    print("move forward {} cm".format(dist * 100))
    pose = sdp.pose()
    xt = pose.x + dist * math.cos(math.radians(pose.yaw))
    yt = pose.y + dist * math.sin(math.radians(pose.yaw))
    sdp.moveToFloatWithYaw(xt, yt, math.radians(pose.yaw))

def rotateToPrecise(sdp, angle):
    pose = sdp.pose()
    for i in range(1):
        sdp.moveToFloatWithYaw(pose.x, pose.y, math.radians(angle))
        sdp.waitUntilMoveActionDone()

def main():
    global _grasper_sonar

    sdp = MyClient()
    #time.sleep(6)
    sdp_comm.connectToSdp(sdp)
    
    _lpArduino = LattePandaArduino()
    _lpArduino.initialize()
    _grasper_sonar = _lpArduino.board.get_pin('d:5:o')
    sdp_comm.connectToSdp(sdp)
    sdp.setSpeed(1)

    try:
        while True:
            input("Scan sonar for object")
            min_dist = getGraspDist()
            min_angle = sdp.heading()

            pose = sdp.pose()
            min_dist, min_angle = sonarSweep(sdp, pose.yaw + 25, min_dist, min_angle)
            min_dist, min_angle = sonarSweep(sdp, pose.yaw - 25, min_dist, min_angle)

            time.sleep(2)

            if min_dist > 0.30:
                continue
            print("turning to min_angle {}".format(min_angle))
            rotateToPrecise(sdp, min_angle)
            dist = getGraspDist()
            print("now dist = {} cm".format(dist*100))
            for i in range(3):
                if dist > min_dist + .01:
                    print("readjusting angle")
                    rotateToPrecise(sdp, min_angle)
                    dist = getGraspDist()
                else:
                    break
            print("final dist = {} cm".format(dist*100))
            input("try to capture object")
            moveForward(sdp, dist + 0.15)
            while True:
                dist = getGraspDist()
                #print("dist = {} cm".format(dist * 100))
                if dist < 0.05:
                    print("got it")
                    sdp.cancelMoveAction()
                    break
                maStatus = sdp.getMoveActionStatus()
                if maStatus == ActionStatus.Stopped or \
                    maStatus == ActionStatus.Error or \
                    maStatus == ActionStatus.Finished:
                    break
                time.sleep(0.1)
            print("done")
    except KeyboardInterrupt:
        sdp.disconnect()
        sdp.shutdown_server32()
    
if __name__ == "__main__":
    main()