import time
from threading import Thread
from enum import Enum
import math
from copy import deepcopy
from pyfirmata import INPUT, Board

class ServoAxis(Enum):
    """Enumerated type for Servo Axis"""
    def __init__(self, number):
        self._as_parameter__ = number

    Wrist = 0
    Grasp = 1

def clamp(num, min_value, max_value):
        num = max(min(num, max_value), min_value)
        return num

_WRIST_LIMITS = [0, 180]
_GRASP_LIMITS = [0, 140]

_WRIST_HOME_ = 95
_GRASP_HOME_ = 90

class RoboGripperServo(object):
    """RoboGripper Servo Class for wrist and grasp """
    def __init__(self, axis:ServoAxis, board :Board):
        self.axis = axis
        self.GRASP_FULL_OPEN = 0
        self.WRIST_HORIZ = 90
        self.WRIST_VERT = 0
        if axis == ServoAxis.Wrist:
            self.min_angle = _WRIST_LIMITS[0]
            self.max_angle = _WRIST_LIMITS[1]
            self.servo = board.get_pin('d:6:s')
            self.angle = 0
            self.home_angle = _WRIST_HOME_
        elif axis == ServoAxis.Grasp:
            self.min_angle = _GRASP_LIMITS[0]
            self.max_angle = _GRASP_LIMITS[1]
            self.servo = board.get_pin('d:8:s')
            self.angle = 0
            self.home_angle = _GRASP_HOME_

        self.setAngle(self.home_angle)

    def __del__(self):
        self.servo.mode = INPUT

    def _setAngle(self, angle, speed):
        curr = int(self.servo.read())
        intAngle = int(angle)
        r = range(curr, intAngle, speed if curr < intAngle else -speed)
        for pos in r:
            self.servo.write(pos)
            time.sleep(0.015)
    
    def getAngle(self, relToHome=True):
        if relToHome:
            return self.angle - self.home_angle
        else:
            return self.angle

    def setAngle(self, angle, speed=1):
        angle = clamp(angle, self.min_angle, self.max_angle)
        self.angle = angle
        if speed == 0:
            self.servo.write(int(angle))
        else:
            self._setAngle(angle, speed)

    def offsetAngle(self, delta, speed=0):
        self.setAngle(self.angle + delta, speed)

    def setHome(self):
        self.setAngle(self.home_angle)
    
class RoboGripper(object):
    def __init__(self):
        self.board : Board = None
        self.wristServo : RoboGripperServo = None
        self.graspServo : RoboGripperServo = None

    def __del__(self):
        self.shutdown()

    def initialize(self, board: Board):
        self.board = board
        self.wristServo = RoboGripperServo(ServoAxis.Wrist, self.board)
        self.graspServo = RoboGripperServo(ServoAxis.Grasp, self.board)

    def allHome(self):
        self.wristServo.setHome()
        self.graspServo.setHome()

    def wristHome(self):
        self.wristServo.setHome()

    def graspHome(self):
        self.graspServo.setHome()

    def getWrist(self):
        return self.wristServo.getAngle()

    def getGrasp(self):
        return self.graspServo.getAngle()

    def offsetWrist(self, delta):
        self.wristServo.offsetAngle(delta)

    def setWrist(self, angle):
        self.wristServo.setAngle(angle)

    # open with negative delta, close with positive delta
    def offsetGrasp(self, delta):
        self.graspServo.offsetAngle(delta)

    def setGrasp(self, angle):
        self.graspServo.setAngle(angle)

    def setGraspFullOpen(self):
        self.setGrasp(self.graspServo.GRASP_FULL_OPEN)

    def setWristVertOrient(self):
        self.setWrist(self.wristServo.WRIST_VERT)

    def setWristHorizOrient(self):
        self.setWrist(self.wristServo.WRIST_HORIZ)

    def shutdown(self):
        try:
            self.allHome()
            del(self.wristServo)
            del(self.graspServo)
        except:
            None

if __name__ == '__main__':
    from robo_gripper import RoboGripper
    from latte_panda_arduino import LattePandaArduino
    _lpArduino = LattePandaArduino()
    _lpArduino.initialize()
    m = RoboGripper()
    m.initialize(_lpArduino.board)
    try:
        while(1):
            m.wristServo.setAngle(_WRIST_LIMITS[0])
            time.sleep(.5)
            m.wristServo.setAngle(_WRIST_LIMITS[1])
            time.sleep(.5)        
            m.graspServo.setAngle(_GRASP_LIMITS[0])              # tell servo to go to position in variable 'pos'
            time.sleep(.5)
            m.graspServo.setAngle(_GRASP_LIMITS[1])              # tell servo to go to position in variable 'pos'
            time.sleep(.5)
    except (KeyboardInterrupt):
        print("KeyboardInterrupt, closing board.")
        m.shutdown()
        _lpArduino.shutdown()
    except Exception as e:
        print("exception: "  + str(e) + ", closing board")
        m.shutdown()
        _lpArduino.shutdown()
    