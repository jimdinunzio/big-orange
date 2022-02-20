from pyfirmata import Arduino, util
import time
from threading import Thread
import eyes
import my_depthai as dai
from enum import Enum
import math
from threading import Lock
from copy import deepcopy
import my_sdp_client
import sdp_comm

_YAW_LIMITS = [0, 180]
_PITCH_LIMITS = [0, 135]

_YAW_HOME_ = 90
_PITCH_HOME_ = 100

#print("opening board: " + str(board))

_pitchServo = None
_yawServo = None
_board = None
_yaw = 0
_pitch = 0
_min_track_confidence = 0.7
_oakd_sdp = None

class ServoAxis(Enum):
    """Enumerated type for Servo Axis"""
    def __init__(self, number):
        self._as_parameter__ = number

    Pitch = 0
    Yaw = 1

class TrackerMode(Enum):
    """Enumerated type for mode of tracker"""
    def __init__(self, number):
        self._as_parameter__ = number
    
    Idle = 0
    TrackScan = 1
    Track = 2

def clamp(num, min_value, max_value):
        num = max(min(num, max_value), min_value)
        return num

def initialize(yaw=_YAW_HOME_, pitch=_PITCH_HOME_):
    global _board, _pitchServo, _yawServo
    _board = Arduino('COM6')
    iter = util.Iterator(_board)
    iter.start()
    _pitchServo = _board.get_pin('d:3:s')
    _yawServo = _board.get_pin('d:5:s')
    setPitch(pitch, 0)
    setYaw(yaw, 0)

def setServo(servo, angle, speed):
    curr = int(servo.read())
    intAngle = int(angle)
    r = range(curr, intAngle, speed if curr < intAngle else -speed)
    for pos in r:
        servo.write(pos)
        time.sleep(0.015)

def getPitch(relToHome=True):
    if relToHome:
        return _pitch - _PITCH_HOME_
    else:
        return _pitch

def setPitch(angle, speed=1):
    global _pitch
    angle = clamp(angle, _PITCH_LIMITS[0], _PITCH_LIMITS[1])
    _pitch = angle
    if speed == 0:
        _pitchServo.write(int(angle))
    else:
        setServo(_pitchServo, angle, speed)

def getYaw(relToHome=True):
    if relToHome:
        return _yaw - _YAW_HOME_
    else:
        return _yaw

def setYaw(angle, speed=1):
    global _yaw
    angle = clamp(angle, _YAW_LIMITS[0], _YAW_LIMITS[1])
    _yaw = angle
    if speed == 0:
        _yawServo.write(int(angle))
    else:
        setServo(_yawServo, angle, speed)

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
_sweeping_thread = None

def servoToEyeYaw(yaw):
    return (yaw - 90) * (7.0 / 9.0)

def servoToEyePitch(pitch):
    return pitch - 100

def startSweepingBackAndForth(count=1, speed=2, min=45, max=135):
    global _sweeping, _sweeping_thread
    _sweeping = True
    _sweeping_thread = Thread(target = sweepYawBackAndForth, args=(count,speed,min,max))
    _sweeping_thread.start()        

def stopSweepingBackAndForth():
    global _sweeping, _sweeping_thread
    _sweeping = False
    if _sweeping_thread is not None:
        _sweeping_thread.join()
        _sweeping_thread = None

def sweepYawBackAndForth(count, speed=2, min=45, max=135):
    global _sweeping
    _sweeping = True
    sweep_count = 0
    yawHome()
    time.sleep(1.0)
    sweep_min = min
    sweep_max = max
    eyes.setHome()
    while _sweeping and (count == 0 or sweep_count < count):
        sweepYaw(_YAW_HOME_, sweep_max, speed)
        sweepYaw(sweep_max, _YAW_HOME_, speed)
        sweepYaw(_YAW_HOME_, sweep_min, speed)
        sweepYaw(sweep_min, _YAW_HOME_, speed)
        sweep_count += 1
    _sweeping = False

def sweepYaw(begin, end, speed):
    if not _sweeping:
        return
    inc = speed if begin <= end else -speed
    for pos in range(begin, end, inc):
        setYaw(pos, 0)
        if eyes._going:
            eyes.setTargetPitchYaw(targetYaw=servoToEyeYaw(pos))
        if not _sweeping:
            return
        time.sleep(0.05)
    time.sleep(0.50)

def suspend():
    _pitchServo.disable_reporting()
    _yawServo.disable_reporting()

def resume():
    _pitchServo.enable_reporting()
    _yawServo.enable_reporting()

pitch_auto_center_time = 0.0
pitch_target_pos = None
pitch_move_steps = 1.0
pitch_obj_ave = 0.0

def pitch_auto_center():
    global pitch_auto_center_time, pitch_target_pos
    pitch_auto_center_time = 0.0
    pitch_target_pos = _PITCH_HOME_

yaw_auto_center_time = 0.0
yaw_target_pos = None
yaw_move_steps = 1.0
yaw_obj_ave = 0.0

def yaw_auto_center():
    global yaw_auto_center_time, yaw_target_pos
    yaw_auto_center_time = 0.0
    yaw_target_pos = _YAW_HOME_

def servo_update(axis, obj):
    global yaw_move_steps, yaw_target_pos, yaw_auto_center_time, yaw_obj_ave, \
        pitch_move_steps, pitch_target_pos, pitch_auto_center_time, pitch_obj_ave
    
    if obj != None:
        #print('Axis: %s, Object to track: [xmin: %f, xmax: %f, ymin: %f]' % (axis, obj.xmin, obj.xmax, obj.ymin))

        if axis == ServoAxis.Yaw:
            yaw_obj_ave = yaw_obj_ave * 0.1 + obj.bboxCtr[0] * 0.9
            diff = 0.5 - yaw_obj_ave
            adj = diff * 7.5
            if abs(adj) < 1.0:
                adj = 0.0
            setYaw(_yaw + adj, 0)
            if eyes._going:
                eyes.setPitchYaw(yaw=servoToEyeYaw(_yaw + adj))

            yaw_target_pos = None
            yaw_auto_center_time = time.monotonic()

        else: # axis == ServoAxis.Pitch
            pitch_obj_ave = pitch_obj_ave * 0.1 + obj.ymin * 0.9
            diff = pitch_obj_ave - 0.25
            adj = diff * 7.5
            if abs(adj) < 1.0:
                adj = 0.0
            setPitch(_pitch + adj, 0)
            if eyes._going:
                eyes.setPitchYaw(pitch=servoToEyePitch(_pitch + adj))
            pitch_target_pos = None
            pitch_auto_center_time = time.monotonic()

    else: # obj == None
        if axis == ServoAxis.Yaw:
            if yaw_target_pos != None:
                diff = yaw_target_pos - _yaw
                if abs(diff) < yaw_move_steps:
                    yaw_move_steps = abs(diff)
                setYaw(_yaw + math.copysign(yaw_move_steps, diff), 0)
                if _yaw == yaw_target_pos or \
                        _yaw <= _YAW_LIMITS[0] or \
                        _yaw >= _YAW_LIMITS[1]:
                        yaw_target_pos = None
            else:
                # If wasn't tracking before,
                # then return to center after a timeout.
                if yaw_auto_center_time != 0.0 and time.monotonic() - yaw_auto_center_time > 6.0:
                    yaw_auto_center_time = 0.0
                    yaw_target_pos = _YAW_HOME_
                    yaw_move_steps = 3

        else: # axis == ServoAxis.Pitch
            if pitch_target_pos != None:
                diff = pitch_target_pos - _pitch
                if abs(diff) < pitch_move_steps:
                    pitch_move_steps = abs(diff)
                setPitch(_pitch + math.copysign(pitch_move_steps, diff), 0)
                if _pitch == pitch_target_pos or \
                        _pitch <= _PITCH_LIMITS[0] or \
                        _pitch >= _PITCH_LIMITS[1]:
                        pitch_target_pos = None
            else:
                # If wasn't tracking before,
                # then return to center after a timeout.
                if pitch_auto_center_time != 0.0 and time.monotonic() - pitch_auto_center_time > 6.0:
                    pitch_auto_center_time = 0.0
                    pitch_target_pos = _PITCH_HOME_
                    pitch_move_steps = 3
                    
_last_tracked_object = None
_last_detected_time = None
_detected_time = time.monotonic()
_tracked_duration = 0
_track_base_track_vel = 0.0
_track_base_track_pan_ave = None
_track_turn_base = True
_track_base_time = time.monotonic()
_track_base_turn_count = 0

class TrackingResult(Enum):
    """Enumerated type for tracking result"""
    def __init__(self, number):
        self._as_parameter__ = number

    Tracked = 0
    Not_Tracked = 1
    Lost = 2


class TrackStatus(object):
    def __init__(self, object=None, trackingRes=TrackingResult.Not_Tracked):
        self.object = object
        self.tracking = trackingRes

_track_status = TrackStatus()
_track_status_lock = Lock()

def update_base_pose_tracking():
    global _track_base_track_pan_ave, _track_base_track_vel, _track_base_time

    pan = getYaw()
    if _track_base_track_pan_ave == None:
        _track_base_track_pan_ave = pan
    else:
        _track_base_track_pan_ave = _track_base_track_pan_ave*0.5 + pan*0.5

    if abs(getYaw()) > 30.0:
        _track_base_track_vel = math.copysign(0.05, _track_base_track_pan_ave)
    else:
        if _track_base_track_vel == 0.0:
            return
        _track_base_track_vel = 0.0
    
    if _track_base_track_vel != 0.0:
        _oakd_sdp.rotate(_track_base_track_vel)

def get_track_status():
    with _track_status_lock:
        return deepcopy(_track_status)
    
def clearLastTrackedObj():
    global _last_tracked_object
    _last_tracked_object = None

def publish_tracked(detection, wasLost=False):
    global _tracked_duration
    with _track_status_lock:
        if detection != None:    
            _track_status.object = detection
            _track_status.tracking = TrackingResult.Tracked
        elif wasLost:
            _track_status.tracking = TrackingResult.Lost
        else:
            _track_status.tracking = TrackingResult.Not_Tracked
        
    # Duration tracked/not tracked
    _tracked_duration = time.monotonic() - _detected_time

def update_tracking(detections):
    global _last_tracked_object, _last_detected_time, _detected_time
    tracked_object = None
    publish = False

    if detections != None:
        for det in detections:
            if det.label != "person" or \
                det.confidence < _min_track_confidence or \
                det.status != dai.dai.Tracklet.TrackingStatus.TRACKED:
                continue

            if _last_tracked_object != None and \
                _last_tracked_object.id == det.id:

                    # Currently tracked object is still detected
                    tracked_object = _last_tracked_object = det
                    _last_detected_time = time.monotonic()
                    publish = True
                    break
                
                # Select the closest person
            if tracked_object == None or tracked_object.z > det.z:
                tracked_object = det

    # Delay a bit before switching away to different person
    if _last_tracked_object == None or \
        (time.monotonic() - _last_detected_time > 1.0):
        
        # Reset the start time of tracking/not tracking if
        # transitioning from not tracking to tracking or
        # vice versa (but not on each timeout while not
        # tracking)
        if _last_tracked_object != None:
            _detected_time = time.monotonic()

            _last_tracked_object = tracked_object
            _last_detected_time = time.monotonic()
        publish = True
        #print("Now tracking: %s" % ("none" if tracked_object == None else tracked_object.id))

    if publish:
        publish_tracked(tracked_object)

    return tracked_object

_tracking_thread = None
_tracking_run = False

def set_track_turn_base(value):
    global _track_turn_base
    _track_turn_base = value

def start_tracking(mdai, modeIn=TrackerMode.TrackScan, trackTurnBase=False):
    global _tracking_run, _tracking_thread, _track_turn_base
    _track_turn_base = trackTurnBase
    _tracking_thread = Thread(target = tracker_thread, args=(mdai, modeIn,))
    _tracking_run = True
    _tracking_thread.start()

def stop_tracking():
    global _tracking_run, _tracking_thread
    if _tracking_thread != None:
        _tracking_run = False
        _tracking_thread.join()
        _tracking_thread = None

def tracker_thread(mdai, mode):
    global _oakd_sdp
    print("Tracking thread started")
    # must establish a separate client and server and connection to SDP because msl loadlib is not thread safe
    _oakd_sdp = my_sdp_client.MyClient()
    sdp_comm.connectToSdp(_oakd_sdp)

    _oakd_sdp.wakeup()
    last_wakeup = time.monotonic()

    track_cnt = 0
    if mode == TrackerMode.TrackScan:
        startSweepingBackAndForth(count=3, speed=1)
    while _tracking_run:
        # keep lidar spinning for quick movement response
        if time.monotonic() - last_wakeup > 50:
            _oakd_sdp.wakeup()
            last_wakeup = time.monotonic()
        detections = mdai.getPersonDetections()
        tracked_object = update_tracking(detections)
        if mode == TrackerMode.TrackScan:
            if tracked_object == None:
                if not _sweeping:
                    break
            else: # found object
                stopSweepingBackAndForth()
                mode = TrackerMode.Track
        elif mode == TrackerMode.Track:
            servo_update(ServoAxis.Yaw, tracked_object)
            servo_update(ServoAxis.Pitch, tracked_object)

            if _track_turn_base:
                update_base_pose_tracking()

        if tracked_object == None:
            _last_detected_time == None
        
        time.sleep(0.050)
    _oakd_sdp.disconnect()
    del _oakd_sdp
    _oakd_sdp = None
    allHome()
    eyes.setHome()
    print("Tracking thread ending")

def shutdown():
    global _board
    stop_tracking()
    allHome()
    if _board is not None:
        _board.exit()
        _board = None

if __name__ == '__main__':
    initialize()
    setYaw(15)
    try:
        while(1):
            for pos in range(_YAW_LIMITS[0], _YAW_LIMITS[1]): # goes from 0 degrees to 180 degrees in steps of 1 degree
                _yawServo.write(pos)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)
            for pos in range(_YAW_LIMITS[1], _YAW_LIMITS[0], -1): # goes from 180 degrees to 0 degrees
                _yawServo.write(pos)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)        
            yawHome()
            setPitch(0)
            for pos in range(_PITCH_LIMITS[0], _PITCH_LIMITS[1]):
                _pitchServo.write(pos)              # tell servo to go to position in variable 'pos'
                time.sleep(0.015)
            time.sleep(.5)
            for pos in range(_PITCH_LIMITS[1], _PITCH_LIMITS[0], -1): # goes from 180 degrees to 0 degrees
                _pitchServo.write(pos)              # tell servo to go to position in variable 'pos'
                time.sleep(0.015)
            time.sleep(.5)
            pitchHome()
            setYaw(15)
    except (KeyboardInterrupt):
        shutdown()
        print("KeyboardInterrupt, closing board.")
    except Exception as e:
        shutdown()
        print("exception: "  + e.str() + ", closing board")
    