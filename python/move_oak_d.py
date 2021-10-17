from pyfirmata import Arduino, util
import time
from threading import Thread
import eyes

_YAW_HOME_ = 90
_PITCH_HOME_ = 100

#print("opening board: " + str(board))

_pitchServo = None
_yawServo = None
_board = None
_yaw = 0
_pitch = 0

def initialize(yaw=_YAW_HOME_, pitch=_PITCH_HOME_):
    global _board, _pitchServo, _yawServo
    global _pitch, _yaw
    _board = Arduino('COM6')
    iter = util.Iterator(_board)
    iter.start()
    _pitchServo = _board.get_pin('d:3:s')
    _yawServo = _board.get_pin('d:5:s')
    _pitchServo.write(pitch)
    _pitch = pitch
    _yawServo.write(yaw)
    _yaw = yaw

def setServo(servo, angle):
    curr = servo.read()
    r = range(curr, angle, 1 if curr < angle else -1)
    for pos in r:
        servo.write(pos)
        time.sleep(0.015)

def getPitch(relToHome=True):
    if relToHome:
        return _pitch - _PITCH_HOME_
    else:
        return _pitch

def setPitch(angle):
    global _pitch
    _pitch = angle
    setServo(_pitchServo, angle)

def getYaw(relToHome=True):
    if relToHome:
        return _yaw - _YAW_HOME_
    else:
        return _yaw

def setYaw(angle):
    global _yaw
    _yaw = angle
    setServo(_yawServo, angle)

def yawHome():
    setYaw(_YAW_HOME_) # straight ahead relative to robot heading

def pitchHome():
    setPitch(_PITCH_HOME_) # parallel to ground
    
def allHome():
    yawHome()
    pitchHome()

def isSweeping():
    return _sweeping

_sweeping = False
def stopSweepingBackAndForth():
    global _sweeping
    _sweeping = False

def startSweepingBackAndForth(count=0):
    global _sweeping
    _sweeping = True
    Thread(target = sweepYawBackAndForth, args=(count,)).start()
        
def sweepYawBackAndForth(count):
    global _sweeping
    _sweeping = True
    sweep_count = 0
    sweep_max = 125
    yawHome()
    if _pitch <= 100:
        sweep_min = 25
    else:
        sweep_min = 35
    eyes.set(0,0)
    while _sweeping and (count == 0 or sweep_count < count):
        sweepYaw(_YAW_HOME_, sweep_max)
        time.sleep(0.5)
        sweepYaw(sweep_max, _YAW_HOME_)
        time.sleep(0.5)
        sweepYaw(_YAW_HOME_, sweep_min)
        time.sleep(0.5)
        sweepYaw(sweep_min, _YAW_HOME_)
        time.sleep(0.5)
        sweep_count += 1
    _sweeping = False

def sweepYaw(begin, end):
    global _yaw
    if not _sweeping:
        return
    inc = 1 if begin <= end else -1
    for pos in range(begin, end, inc):
        _yawServo.write(pos)
        if eyes._going:
            eyes.setAngleOffset(targetOffset=(pos - 90) * (60.0/35.0))
        _yaw = pos
        if not _sweeping:
            return
        time.sleep(0.05)

def suspend():
    _pitchServo.disable_reporting()
    _yawServo.disable_reporting()

def resume():
    _pitchServo.enable_reporting()
    _yawServo.enable_reporting()
    
def shutdown():
    global _board
    allHome()
    if _board is not None:
        _board.exit()
        _board = None

if __name__ == '__main__':
    initialize()
    setYaw(45)
    try:
        while(1):
            for pos in range(45, 125): # goes from 0 degrees to 180 degrees in steps of 1 degree
                _yawServo.write(pos)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)
            for pos in range(125, 45, -1): # goes from 180 degrees to 0 degrees
                _yawServo.write(pos)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)        
            yawHome()
            setPitch(80)
            for pos in range(80, 135):
                _pitchServo.write(pos)              # tell servo to go to position in variable 'pos'
                time.sleep(0.015)
            time.sleep(.5)
            for pos in range(135, 80, -1): # goes from 180 degrees to 0 degrees
                _pitchServo.write(pos)              # tell servo to go to position in variable 'pos'
                time.sleep(0.015)
            time.sleep(.5)
            pitchHome()
            setYaw(45)
    except (KeyboardInterrupt):
        shutdown()
        print("KeyboardInterrupt, closing board.")
    except Exception as e:
        shutdown()
        print("exception: "  + e.str() + ", closing board")
    