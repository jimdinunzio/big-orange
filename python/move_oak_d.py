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

def clamp(num, min_value, max_value):
        num = max(min(num, max_value), min_value)
        return num

_YAW_LIMITS = [0, 180]
_PITCH_LIMITS = [0, 135]

_YAW_HOME_ = 90
_PITCH_HOME_ = 100

def servoToEyeYaw(yaw):
    return (yaw - 90) * (7.0 / 9.0)

def servoToEyePitch(pitch):
    return pitch - 100

class OakDServo(object):
    """OakD Servo Class for Pitch and Yaw """
    def __init__(self, axis:ServoAxis, board):
        self.axis = axis
        if axis == ServoAxis.Pitch:
            self.min_angle = _PITCH_LIMITS[0]
            self.max_angle = _PITCH_LIMITS[1]
            self.servo = board.get_pin('d:3:s')
            self.angle = 0
            self.home_angle = _PITCH_HOME_
        elif axis == ServoAxis.Yaw:
            self.min_angle = _YAW_LIMITS[0]
            self.max_angle = _YAW_LIMITS[1]
            self.servo = board.get_pin('d:5:s')
            self.angle = 0
            self.home_angle = _YAW_HOME_

        self.lastBaseYaw = -1
        self.auto_center_time = 0.0
        self.target_pos = None
        self.move_steps = 1.0
        self.obj_ave = 0.0
        self.setAngle(self.home_angle, 0)

    def auto_center(self):
        self.auto_center_time = 0.0
        self.target_pos = self.home_angle

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

    def setHome(self):
        self.setAngle(self.home_angle)
    
    def resume(self):
        self.servo.disable_reporting()

    def resume(self):
        self.servo.enable_reporting()

    def update(self, obj, baseYaw):
        if obj != None:
            #print('Axis: %s, Object to track: [xmin: %f, xmax: %f, ymin: %f]' % (axis, obj.xmin, obj.xmax, obj.ymin))
            if self.axis == ServoAxis.Yaw:
                self.obj_ave = self.obj_ave * 0 + obj.bboxCtr[0] * 1
                diff = 0.5 - self.obj_ave
                adj = diff * 6
                if abs(adj) < 1.0:
                    adj = 0.0
                
                adjForNavBaseMov = 0
                if baseYaw != -1 and self.lastBaseYaw != -1:
                    adjForNavBaseMov = baseYaw - self.lastBaseYaw
                    delta = abs(adjForNavBaseMov)
                    if delta > 180:
                        adjForNavBaseMov = math.copysign(360 - delta, adjForNavBaseMov)
                    # if adjForNavBaseMov != 0:
                    #     print("Adjusting for base nav move: ", adjForNavBaseMov)
                    if adjForNavBaseMov < 0.1:
                        adjForNavBaseMov = 0
                
                self.setAngle(self.angle + adj - adjForNavBaseMov, 0)
                if eyes._going:
                    eyes.setPitchYaw(yaw = servoToEyeYaw(self.angle + adj + adjForNavBaseMov))
                # try to compensate if the robot is turning quickly and detections are lost

            else: # axis == ServoAxis.Pitch
                self.obj_ave = self.obj_ave * 0 + obj.ymin * 1
                diff = self.obj_ave - 0.25
                adj = diff * 6
                if abs(adj) < 1.0:
                    adj = 0.0
                self.setAngle(self.angle + adj, 0)
                if eyes._going:
                    eyes.setPitchYaw(pitch = servoToEyePitch(self.angle + adj))

            self.target_pos = None
            self.auto_center_time = time.monotonic()

        else: # obj == None
            if self.target_pos != None:
                diff = self.target_pos - self.angle
                if abs(diff) < self.move_steps:
                    self.move_steps = abs(diff)
                newAngle = self.angle + math.copysign(self.move_steps, diff)
                self.setAngle(newAngle, 0)
                if eyes._going:
                    if self.axis == ServoAxis.Yaw:
                        eyes.setPitchYaw(yaw = servoToEyeYaw(newAngle))
                    else: # self.axis == ServoAxis.Pitch
                        eyes.setPitchYaw(pitch = servoToEyePitch(newAngle))
                if self.angle == self.target_pos or \
                        self.angle <= self.min_angle or \
                        self.angle >= self.max_angle:
                        self.target_pos = None                
            else:
                # If wasn't tracking before,
                # then return to center after a timeout.
                if self.auto_center_time != 0.0 and time.monotonic() - self.auto_center_time > 6.0:
                    self.auto_center_time = 0.0
                    self.target_pos = self.home_angle
                    self.move_steps = 3
        self.lastBaseYaw = baseYaw

class MoveOakD(object):
    def __init__(self):
        self.pitchServo = None
        self.yawServo = None
        self.board = None
        self.min_track_confidence = 0.7
        self.oakd_sdp = None
        self.sweeping = False
        self.sweeping_thread = None
        self.last_tracked_object = None
        self.last_detected_time = None
        self.detected_time = time.monotonic()
        self.tracked_duration = 0
        self.track_base_track_vel = 0.0
        self.track_base_track_pan_ave = None
        self.track_turn_base = True
        self.track_base_time = time.monotonic()        
        self.track_status = TrackStatus()
        self.track_status_lock = Lock()
        self.tracking_thread = None
        self.tracking_run = False

    def initialize(self):
        self.board = Arduino('COM6')
        iter = util.Iterator(self.board)
        iter.start()
        self.pitchServo = OakDServo(ServoAxis.Pitch, self.board)
        self.yawServo = OakDServo(ServoAxis.Yaw, self.board)

    def allHome(self):
        self.pitchServo.setHome()
        self.yawServo.setHome()

    def isSweeping(self):
        return self.sweeping

    def getYaw(self):
        return self.yawServo.getAngle()

    def getPitch(self):
        return self.pitchServo.getAngle()

    def startSweepingBackAndForth(self, count=1, speed=2, min=45, max=135):
        self.sweeping = True
        self.sweeping_thread = Thread(target = self.sweepYawBackAndForth, args=(count,speed,min,max), name="oakd sweep", daemon=False)
        self.sweeping_thread.start()        

    def stopSweepingBackAndForth(self):
        self.sweeping = False
        if self.sweeping_thread is not None:
            self.sweeping_thread.join()
            self.sweeping_thread = None

    def sweepYawBackAndForth(self, count, speed=2, min=45, max=135):
        self.sweeping = True
        sweep_count = 0
        self.yawServo.setHome()
        time.sleep(1.0)
        sweep_min = min
        sweep_max = max
        eyes.setHome()
        while self.sweeping and (count == 0 or sweep_count < count):
            self.sweepYaw(_YAW_HOME_, sweep_max, speed)
            self.sweepYaw(sweep_max, _YAW_HOME_, speed)
            self.sweepYaw(_YAW_HOME_, sweep_min, speed)
            self.sweepYaw(sweep_min, _YAW_HOME_, speed)
            sweep_count += 1
        self.sweeping = False

    def sweepYaw(self, begin, end, speed):
        if not self.sweeping:
            return
        inc = speed if begin <= end else -speed
        for pos in range(begin, end, inc):
            self.yawServo.setAngle(pos, 0)
            if eyes._going:
                eyes.setTargetPitchYaw(targetYaw=servoToEyeYaw(pos))
            if not self.sweeping:
                return
            time.sleep(0.05)
        time.sleep(0.50)

    def suspend(self):
        self.pitchServo.suspend()
        self.yawServo.suspend()

    def resume(self):
        self.pitchServo.resume()
        self.yawServo.resume()

    def update_base_pose_tracking(self):
        pan = self.yawServo.getAngle()
        if self.track_base_track_pan_ave == None:
            self.track_base_track_pan_ave = pan
        else:
            self.track_base_track_pan_ave = self.track_base_track_pan_ave*0.5 + pan*0.5

        if abs(self.yawServo.getAngle()) > 30.0:
            self.track_base_track_vel = math.copysign(0.05, self.track_base_track_pan_ave)
        else:
            if self.track_base_track_vel == 0.0:
                return
            self.track_base_track_vel = 0.0
        
        if self.track_base_track_vel != 0.0:
            self.oakd_sdp.rotate(self.track_base_track_vel)

    def get_track_status(self):
        with self.track_status_lock:
            return deepcopy(self.track_status)
    
    def clearLastTrackedObj(self):
        self.last_tracked_object = None

    def publish_tracked(self, detection, wasLost=False):
        with self.track_status_lock:
            if detection != None:    
                self.track_status.object = detection
                self.track_status.tracking = TrackingResult.Tracked
            elif wasLost:
                self.track_status.tracking = TrackingResult.Lost
            else:
                self.track_status.tracking = TrackingResult.Not_Tracked
            
        # Duration tracked/not tracked
        self.tracked_duration = time.monotonic() - self.detected_time

    def update_tracking(self, detections):
        tracked_object = None
        publish = False

        if detections != None:
            for det in detections:
                if det.label != "person" or \
                    det.confidence < self.min_track_confidence or \
                    det.status != dai.dai.Tracklet.TrackingStatus.TRACKED:
                    continue

                if self.last_tracked_object != None and \
                    self.last_tracked_object.id == det.id:

                        # Currently tracked object is still detected
                        tracked_object = self.last_tracked_object = det
                        self.last_detected_time = time.monotonic()
                        publish = True
                        break
                    
                    # Select the closest person
                if tracked_object == None or tracked_object.z > det.z:
                    tracked_object = det

        # Delay a bit before switching away to different person
        if self.last_tracked_object == None or \
            (time.monotonic() - self.last_detected_time > 1.0):
            
            # Reset the start time of tracking/not tracking if
            # transitioning from not tracking to tracking or
            # vice versa (but not on each timeout while not
            # tracking)
            if self.last_tracked_object != None:
                self.detected_time = time.monotonic()

            self.last_tracked_object = tracked_object
            self.last_detected_time = time.monotonic()
        
            publish = True
            #print("Now tracking: %s" % ("none" if tracked_object == None else tracked_object.id))

        if publish:
            self.publish_tracked(tracked_object)

        return tracked_object

    def set_track_turn_base(self, value):
        self.track_turn_base = value

    def start_tracking(self, mdai, modeIn=TrackerMode.TrackScan, trackTurnBase=False):
        self.track_turn_base = trackTurnBase
        self.tracking_thread = Thread(target = self.tracker_thread, args=(mdai, modeIn,), name="tracker", daemon=False)
        self.tracking_run = True
        self.tracking_thread.start()

    def stop_tracking(self):
        if self.tracking_thread != None:
            self.tracking_run = False
            self.tracking_thread.join()
        self.tracking_thread = None

    def tracker_thread(self, mdai, mode):
        #print("Tracking thread started")
        # must establish a separate client and server and connection to SDP because msl loadlib is not thread safe
        self.oakd_sdp = my_sdp_client.MyClient()
        sdp_comm.connectToSdp(self.oakd_sdp)

        self.oakd_sdp.wakeup()
        last_wakeup = time.monotonic()
        last_heading_update = time.monotonic()

        if mode == TrackerMode.TrackScan:
            self.startSweepingBackAndForth(count=3, speed=2)
        while self.tracking_run:
        
            # keep lidar spinning for quick movement response
            if time.monotonic() - last_wakeup > 50:
                self.oakd_sdp.wakeup()
                last_wakeup = time.monotonic()
            detections = mdai.getPersonDetections()
            tracked_object = self.update_tracking(detections)
            if mode == TrackerMode.TrackScan:
                if tracked_object == None:
                    if not self.sweeping:
                        break
                else: # found object
                    self.stopSweepingBackAndForth()
                    mode = TrackerMode.Track
            elif mode == TrackerMode.Track:
                if time.monotonic() - last_heading_update >= 0.25:
                    if not self.track_turn_base:
                        baseYaw = self.oakd_sdp.heading() + 360
                    else:
                        baseYaw = -1
                    last_heading_update = time.monotonic()
                self.yawServo.update(tracked_object, baseYaw)
                self.pitchServo.update(tracked_object, baseYaw)

                if self.track_turn_base:
                    self.update_base_pose_tracking()

            if tracked_object == None:
                self.last_detected_time == None
            
            time.sleep(0.050)
            
        self.oakd_sdp.disconnect()
        self.oakd_sdp.shutdown_server32(kill_timeout=1)
        self.oakd_sdp = None
        self.allHome()
        eyes.setHome()
        #print("Tracking thread ending")

    def shutdown(self):
        self.stop_tracking()
        self.allHome()
        if self.board is not None:
            self.board.exit()
            self.board = None

if __name__ == '__main__':
    m = MoveOakD()
    m.initialize()
    m.yawServo.setAngle(15)
    try:
        while(1):
            for pos in range(_YAW_LIMITS[0], _YAW_LIMITS[1]): # goes from 0 degrees to 180 degrees in steps of 1 degree
                m.yawServo.setAngle(pos, 0)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)
            for pos in range(_YAW_LIMITS[1], _YAW_LIMITS[0], -1): # goes from 180 degrees to 0 degrees
                m.yawServo.setAngle(pos, 0)
                time.sleep(0.015)                       # waits 15ms for the servo to reach the position
            time.sleep(.5)        
            m.yawServo.setHome()
            m.pitchServo.setAngle(0)
            for pos in range(_PITCH_LIMITS[0], _PITCH_LIMITS[1]):
                m.pitchServo.setAngle(pos, 0)              # tell servo to go to position in variable 'pos'
                time.sleep(0.015)
            time.sleep(.5)
            for pos in range(_PITCH_LIMITS[1], _PITCH_LIMITS[0], -1): # goes from 180 degrees to 0 degrees
                m.pitchServo.setAngle(pos, 0)              # tell servo to go to position in variable 'pos'
                time.sleep(0.015)
            time.sleep(.5)
            m.pitchServo.setHome()
            m.yawServo.setAngle(15)
    except (KeyboardInterrupt):
        m.shutdown()
        print("KeyboardInterrupt, closing board.")
    except Exception as e:
        m.shutdown()
        print("exception: "  + str(e) + ", closing board")
    