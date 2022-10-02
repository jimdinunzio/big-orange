import time
import facial_recognize as fr
from threading import Thread
from latte_panda_arduino import LattePandaArduino
from move_oak_d import MoveOakD


_lpArduino = LattePandaArduino()
_lpArduino.initialize()
m = MoveOakD()
m.initialize(_lpArduino.board)

f = fr.FacialRecognize(m.getPitch, m.offsetPitch, m.getYaw, m.offsetYaw)
t=Thread(target=f.run)
t.start()
try:
    while 1:
        f.get_detected_names()
        time.sleep(0.250)
except KeyboardInterrupt:
    f.shutdown()
    t.join()
    m.allHome()
    time.sleep(1)
    m.shutdown()
    time.sleep(1)
    _lpArduino.shutdown()
