from ctypes import create_string_buffer

_sdp_ip_address = b"192.168.11.1"
_sdp_port = 1445

def connectToSdp(sdp):
    errStr = create_string_buffer(255)
    res = sdp.connectSlamtec(_sdp_ip_address, _sdp_port, errStr.raw, 255)
    if res == 1 :
        print("Could not connect to SlamTec, time out.")#, errStr.value)
    elif res == 2:
        print("Could not connect to SlamTec, connection fail.")#, errStr.value)
    return res
