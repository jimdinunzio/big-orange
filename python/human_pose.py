from array import ArrayType
import os
import sys

sys.path.append(os.path.join(os.path.dirname(__file__), 'depthai_blazepose'))
from BlazeposeDepthaiEdge import BlazeposeDepthai
from BlazeposeRenderer import BlazeposeRenderer
from mediapipe_utils import KEYPOINT_DICT
import time
from math import acos, degrees

#Based on http://geomalgorithms.com/a05-_intersect-1.html
import numpy as np
import sys
import os

import robot_frames

epsilon=1e-6

def rayPlaneIntersect(planeNormal, planePoint, rayDirection, rayPoint):
    ndotl = planeNormal.dot(rayDirection) 

    if abs(ndotl) < epsilon:
        #print ("no intersection or line is within plane")
        return None

    w = planePoint - rayPoint
    t = planeNormal.dot(w) / ndotl

    if t < 0:
        #print("no intersection because ray points away from plane")
        return None

    Pt = rayPoint + t * rayDirection

    return Pt

FLOOR_NORMAL = np.array([0.0, 0.0, 1.0])  # robot frame, z up

# get_target returns this when the arm ray never meets the floor.
POINTING_TOO_HIGH = "pointing too high"


def _to_robot(p, yaw_deg, pitch_deg):
    """
    Convert a camera-frame landmark point to the robot frame.

    landmarks_world and the depth translation share the camera's axes with
    y down; oakd_to_robot takes the Oak-D convention with y up.
    """
    return np.array(robot_frames.oakd_to_robot(
        p[0], -p[1], p[2], yaw_deg=yaw_deg, pitch_deg=pitch_deg))


#_next_print_time = 0
def recognize_gesture(body, head_angles=(0.0, 0.0)):
#    global _next_print_time

    if body.xyz_ref:
        """
        Beware, the y value of landmarks_world coordinates is negative for landmarks 
        above the mid hips (like shoulders) and negative for landmarks below (like feet).
        The y value of (x,y,z) coordinates given by depth sensor is negative in the lower part
        of the image and positive in the upper part.
        """
        translation = body.xyz / 1000
        translation[1] = -translation[1]
        if body.xyz_ref == "mid_hips":                   
            final_trans = translation
        elif body.xyz_ref == "mid_shoulders":
            mid_hips_to_mid_shoulders = np.mean([
                body.landmarks_world[KEYPOINT_DICT['right_shoulder']],
                body.landmarks_world[KEYPOINT_DICT['left_shoulder']]],
                axis=0) 
            final_trans = translation - mid_hips_to_mid_shoulders   
    else: # no gesture
        return None

    yaw_deg, pitch_deg = head_angles

    # Rotate the arm into the robot frame before meeting the floor: the
    # landmarks turn with the head, the floor does not.
    rw = _to_robot(body.landmarks_world[KEYPOINT_DICT['right_wrist']] + final_trans,
                   yaw_deg, pitch_deg)
    re = _to_robot(body.landmarks_world[KEYPOINT_DICT['right_shoulder']] + final_trans,
                   yaw_deg, pitch_deg)
    
    right_arm_dir = rw - re

    floor_point = np.array([0.0, 0.0, robot_frames.FLOOR_Z])
    r_res = rayPlaneIntersect(FLOOR_NORMAL, floor_point, right_arm_dir, rw)

    if r_res is not None:
        result = r_res
    else:
        result = POINTING_TOO_HIGH

    # if time.monotonic() > _next_print_time:
    #     if result is not None:
    #         arm_str = "right" if arm == 0 else "left"
    #         arm_dir = right_arm_dir if arm == 0 else left_arm_dir
    #         rw = body.landmarks_world[KEYPOINT_DICT['right_wrist']] + final_trans
    #         lw = body.landmarks_world[KEYPOINT_DICT['left_wrist']] + final_trans
    #         rs = body.landmarks_world[KEYPOINT_DICT['right_shoulder']] + final_trans
    #         ls = body.landmarks_world[KEYPOINT_DICT['left_shoulder']] + final_trans
    #         s = rs if arm == 0 else ls
    #         w = rw if arm == 0 else re

    #         print(arm_str, " arm: ", arm_dir, "angle to floor = ",
    #             degrees(acos(-floorNormal.dot(arm_dir) / (np.linalg.norm(floorNormal) * np.linalg.norm(arm_dir)))))
    #         print("shoulder = ", s)
    #         print("wrist = ", w)
    #         print("pointing to floor at ", result)
    #     else:
    #         None 
    #         print("not pointing at floor")
    #     _next_print_time = time.monotonic() + 1

    return result

class MyBlazePose:
    def __init__(self, device_id=None, get_head_angles=None):
        self.run_flag = False
        self.device_id_ = device_id
        # Returns (yaw_deg, pitch_deg) of the head, relative to home.
        self.get_head_angles_ = get_head_angles or (lambda: (0.0, 0.0))
        self.reset()

    def reset(self):
        self.target = None
        self.person_loc = None
        self.lm_score = 0.0
        self.rect_points = [[0,0],[0,0]]

    def shutdown(self):
        self.run_flag = False

    def get_target(self):
        return self.target

    def get_rect_points(self):
        return self.rect_points

    def get_lm_score(self):
        return self.lm_score

    def get_is_running(self):
        return self.run_flag
        
    def get_person_loc(self):
        """Robot-frame position of the person, metres, or None."""
        return self.person_loc

    def run(self):
        pose = BlazeposeDepthai(input_src='rgb', lm_model='lite', xyz=True, internal_frame_height=432, internal_fps=15, device_id=self.device_id_)
        renderer = BlazeposeRenderer(pose)

        self.target = None
        self.run_flag = True
        while self.run_flag:
            # Run blazepose on next frame
            frame, body = pose.next_frame()
            if frame is None:
                break

            # Draw 2d skeleton
            frame = renderer.draw(frame, body, self.target)
            
            # Gesture recognition
            if body:
                head_angles = self.get_head_angles_()
                self.target = recognize_gesture(body, head_angles)
                self.lm_score = body.lm_score
                # body.xyz is already the Oak-D convention, y up.
                self.person_loc = np.array(robot_frames.oakd_to_robot(
                    body.xyz[0] / 1000.0, body.xyz[1] / 1000.0, body.xyz[2] / 1000.0,
                    yaw_deg=head_angles[0], pitch_deg=head_angles[1]))
                self.rect_points = body.rect_points[1:3]
        #        if letter:
        #            cv2.putText(frame, letter, (frame.shape[1] // 2, 100), cv2.FONT_HERSHEY_PLAIN, 5, (0,190,255), 3)
            renderer.waitKey(45)

        renderer.exit()
        pose.exit()
        renderer = None
        pose = None
        print("exiting blazepose")    

if __name__ == "__main__":
    mbp = MyBlazePose()
    try:
        mbp.run()
    except KeyboardInterrupt:
        print("shutting down")
        