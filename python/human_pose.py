from array import ArrayType
from blazepose.BlazeposeDepthaiEdge import BlazeposeDepthai
from blazepose.BlazeposeRenderer import BlazeposeRenderer
from blazepose.mediapipe_utils import KEYPOINT_DICT
import time
from math import acos, degrees

#Based on http://geomalgorithms.com/a05-_intersect-1.html
import numpy as np

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

floorNormal = np.array([0, -1, 0])   # z up direction
floorPoint = np.array([0, 0.635, 0]) # camera space is 63.5cm above floor, the height of the camera

#_next_print_time = 0
def recognize_gesture(body):  
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

    rw = body.landmarks_world[KEYPOINT_DICT['right_wrist']] + final_trans
#    lw = body.landmarks_world[KEYPOINT_DICT['left_wrist']] + final_trans
    re = body.landmarks_world[KEYPOINT_DICT['right_shoulder']] + final_trans
#    le = body.landmarks_world[KEYPOINT_DICT['left_shoulder']] + final_trans
    
    right_arm_dir = rw - re
#    left_arm_dir  = lw - le

    r_res = rayPlaneIntersect(floorNormal, floorPoint, right_arm_dir, rw)
#    l_res = rayPlaneIntersect(floorNormal, floorPoint, left_arm_dir, lw)

    if r_res is not None:
#        arm = 0
        result = r_res
    # elif l_res is not None:
    #     arm = 1
    #     result = l_res
    else:
        result = [0, -1, 0] # pointing up to high

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
    def __init__(self, device_id=None):
        self.run_flag = False
        self.device_id_ = device_id
        self.reset()
        
    def reset(self):
        self.target = None
        self.person_loc = [0,0]
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
        return self.person_loc / 1000.0

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
                result = recognize_gesture(body)
                self.target = result
                self.lm_score = body.lm_score
                self.person_loc = body.xyz
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
        