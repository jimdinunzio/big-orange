import sys
import os
sys.path.append(os.path.abspath(os.path.join('..')))

# set the working dir to ..
os.chdir(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import time
import facial_recognize as fr
from threading import Thread
from latte_panda_arduino import LattePandaArduino
from move_oak_d import MoveOakD
import cv2

_lpArduino = LattePandaArduino()
_lpArduino.initialize()
m = MoveOakD()
m.initialize(_lpArduino.board)

f = fr.FacialRecognize(m.getPitch, m.offsetPitch, m.getYaw, m.offsetYaw)
t=Thread(target=f.run)
t.start()
try:
    while 1:
        detections = f.get_detections()
        for detection in detections:
            if detection.x != 0 or detection.y != 0 or detection.z != 0:
                print("{} is seen at location x = {}, y = {}, z = {}".format(detection.name, 
                                                                                detection.x,
                                                                                detection.y,
                                                                                detection.z))
            else:
                print("{} is seen.".format(detection.name))
        if cv2.waitKey(50) == ord('q'):
            break
        time.sleep(1)
except KeyboardInterrupt:
    f.shutdown()
    t.join()
    m.allHome()
    time.sleep(1)
    m.shutdown()
    time.sleep(1)
    _lpArduino.shutdown()
