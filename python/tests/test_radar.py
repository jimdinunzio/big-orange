from latte_panda_arduino import LattePandaArduino
from radar import Radar
import time

def swCb(resp):
    print("SW Ver: ", bytearray(resp).decode())

_lpArduino = LattePandaArduino()
_lpArduino.initialize()

_radar = Radar()
_radar.initialize(_lpArduino.board)
_radar.start_sensing()
_radar.get_software_ver(swCb)

try:
    while True:
        if _radar.has_message():
            msg = _radar.pop_message()
            print(msg)
        time.sleep(0.250)
except(KeyboardInterrupt):
    print("shutting down")
    _radar.shutdown()
    _lpArduino.shutdown()
