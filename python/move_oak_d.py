from pyfirmata import Arduino, util
import time
from threading import Thread

_YAW_HOME_ = 81
_PITCH_HOME_ = 110

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
    Thread(target = sweepYawBackAndForth, args=(count,)).start()
        
def sweepYawBackAndForth(count):
    global _sweeping
    sweep_count = 0
    setYaw(35)
    _sweeping = True
    while _sweeping and (count == 0 or sweep_count < count):
        sweepYaw(41, 120) # 35 is limit on low end
        time.sleep(0.1)
        sweepYaw(120, 41)
        sweep_count += 1
    _sweeping = False

def sweepYaw(begin, end):
    global _yaw
    if not _sweeping:
        return
    inc = 1 if begin <= end else -1
    for pos in range(begin, end, inc):
        _yawServo.write(pos)
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
    #allHome()
    if _board is not None:
        _board.exit()
        _board = None

if __name__ == '__main__':
    initialize()
    setYaw(45)
    try:
        while(1):
            for pos in range(35, 120): # goes from 0 degrees to 180 degrees in steps of 1 degree
                _yawServo.write(pos)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)
            for pos in range(120, 35, -1): # goes from 180 degrees to 0 degrees
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
    