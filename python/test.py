import my_sdp_client as client
import sdp_comm
import time
from threading import Thread

_running = False

def poseit():
    global _running
    sdp = client.MyClient()
    sdp_comm.connectToSdp(sdp)
    while _running:
        pose = sdp.pose()
        if type(pose) == tuple:
            print(pose)
            time.sleep(0.1)
        print("yaw is: ", sdp.pose().yaw)
        time.sleep(0.1)
    sdp.disconnect()

def leftit():
    global _running
    sdp = client.MyClient()
    sdp_comm.connectToSdp(sdp)
    while _running:
        sdp.left()
        time.sleep(.2)
    sdp.disconnect()

def testit():
    global _running
    _running = True
    Thread(target=poseit).start()
    Thread(target=leftit).start()

def stopit():
    global _running
    _running = False


