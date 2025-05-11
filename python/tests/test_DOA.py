import sys
import os
sys.path.append(os.path.abspath(os.path.join('..')))

from mic_array_tuning import Tuning
import usb.core
import usb.util
import time
 
dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
 
if dev:
    Mic_tuning = Tuning(dev)
    print(Mic_tuning.direction)
    while True:
        try:
            print(Mic_tuning.direction)
            time.sleep(1)
        except KeyboardInterrupt:
            break
