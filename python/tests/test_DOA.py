import sys
import os
sys.path.append(os.path.abspath(os.path.join('..')))

from mic_array_tuning import Tuning
import usb.core
import usb.util
import time

# DOA reading of a source straight ahead of the robot.  The front of the array
# faces robot left, and DOA increases counter-clockwise, the same sense as
# robot yaw.
DOA_FORWARD = 0


def doa2YawDelta(doa):
    """Convert DOA to yaw delta from forward direction, positive to the left."""
    yawDelta = (doa - DOA_FORWARD) % 360
    if yawDelta >= 180:
        yawDelta = yawDelta - 360
    return yawDelta


def label(yawDelta):
    """Name the robot-frame direction a yaw delta points at."""
    for limit, name in ((22.5, "front"), (67.5, "front left"), (112.5, "left"),
                        (157.5, "rear left"), (180.0, "rear")):
        if abs(yawDelta) <= limit:
            if limit == 22.5 or limit == 180.0:
                return name
            return name if yawDelta > 0 else name.replace("left", "right")
    return "rear"


dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)

if dev:
    Mic_tuning = Tuning(dev)
    print("speak from a known direction; yaw is positive to the robot's left")
    olddoa = -1
    while True:
        try:
            doa = Mic_tuning.direction
            yawDelta = doa2YawDelta(doa)
            if doa != olddoa:
                print("doa %3d  yaw %+4d  %s" % (doa, yawDelta, label(yawDelta)))
                olddoa = doa
            time.sleep(1)
        except KeyboardInterrupt:
            break
else:
    print("Error, could not find the mic array.")
