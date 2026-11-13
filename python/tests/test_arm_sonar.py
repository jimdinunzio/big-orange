
import sys
import os

# allow importing from parent directory
sys.path.append(os.path.abspath(os.path.join('..')))

import time
from latte_panda_arduino import LattePandaArduino
from pyFirmata.pyfirmata import util as pyfirmata_util

SONAR_PIN = 12
PERIOD = 0.2  # 5 readings per second


def main():
    lpArduino = LattePandaArduino()
    lpArduino.initialize()
    sonar = lpArduino.board.get_pin('d:{}:o'.format(SONAR_PIN))

    try:
        while True:
            start = time.time()
            duration = sonar.ping()
            if duration:
                dist = pyfirmata_util.ping_time_to_distance(duration)
                print("sonar = {:.1f} cm".format(dist))
            else:
                print("sonar = no echo")
            time.sleep(max(0.0, PERIOD - (time.time() - start)))
    except KeyboardInterrupt:
        pass
    finally:
        lpArduino.board.exit()


if __name__ == "__main__":
    main()
