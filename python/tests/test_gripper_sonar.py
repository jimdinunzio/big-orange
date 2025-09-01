import sys
import os

# allow importing from parent directory
sys.path.append(os.path.abspath(os.path.join('..')))

from latte_panda_arduino import LattePandaArduino
from pyFirmata.pyfirmata import util as pyfirmata_util, Pin

_sonar_grasp_offset = -0.063 # distance to back of grasper
_last_grasper_sonar : float = 4.50
_grasper_sonar : Pin

def getGrasperSonar() -> float:
    global _last_grasper_sonar
    duration = _grasper_sonar.ping()
    if duration:
        _last_grasper_sonar = pyfirmata_util.ping_time_to_distance(duration)
    return _last_grasper_sonar


def getGraspDist() -> float:
    return getGrasperSonar() + _sonar_grasp_offset 

def main():
    global _grasper_sonar

    _lpArduino = LattePandaArduino()
    _lpArduino.initialize()
    _grasper_sonar = _lpArduino.board.get_pin('d:13:o')

    try:
        while True:
            dist = getGraspDist()
            print("dist = {} cm".format(dist))
    except KeyboardInterrupt:
        print("Exiting sonar sweep test.")
        _lpArduino.shutdown()
        _grasper_sonar = None
    
if __name__ == "__main__":
    main()