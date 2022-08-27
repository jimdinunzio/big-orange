from re import I
import time
from numpy import array, byte
from pyfirmata import Board, SW_SERIAL0
import struct
from threading import Lock

# output codes

BODYSIGN_OUT = 1
ENVIRONMENT_OUT = 2
NOBODY_OUT = 3
SOMEBODY_BE_OUT = 4
REPORT_RADAR_OUT = 5
SOMEBODY_MOVE_OUT = 6
SOMEBODY_STOP_OUT = 7
HEARTBEAT_OUT = 8
TOWARDS_AWAY_OUT = 9
NO_MOVE_OUT = 10
CA_TOWARDS_OUT = 11
CA_AWAY_OUT = 12
EXCEPTION_OUT = 13
REPORT_OTHER_OUT = 14
UNKNOWN_OUT = 15


# Radar byte codes
MESSAGE_HEAD=0x55         # Start byte of message

ACTIVE_REPORT = 0x04      # Proactive reporting

REPORT_RADAR = 0x03       # Report radar information
REPORT_OTHER = 0x05       # Report other information
REPORT_FALL = 0x06        # Report a fall

HEARTBEAT = 0x01          # Heartbeat Pack
ABNORMAL = 0x02           # Abnormal Reset
ENVIRONMENT = 0x05        # Environment
BODYSIGN = 0x06           # Physical parameters
TOWARDS_AWAY = 0x07       # Approach away

CA_BE = 0x01              # Approach away head frame
CA_TOWARDS = 0x02         # Someone approaches
CA_AWAY = 0x03            # Some people stay away
SOMEBODY_BE = 0x01        # Motion state header frame
SOMEBODY_MOVE = 0x01      # Somebody move
SOMEBODY_STOP = 0x00      # Somebody stop
NOBODY = 0x00             # No one here

class Radar:
    class Message:
        def __init__(self, report_type, value, status):
            self.report_type = report_type
            self.value = value
            self.status = status
        
        def __repr__(self):
            s = "Report Type:"
            if self.report_type == BODYSIGN_OUT:
                interp = self.evalBodySign(self.value, 25, 35)
                s += " Body sign, speed = " + str(self.value) + " interp: "
                if interp == SOMEBODY_STOP_OUT:
                    s += "Resting person"
                elif interp == SOMEBODY_MOVE_OUT:
                    s += "Moving person"
                elif interp == NOBODY_OUT:
                    s += "No body present"
                return s
            elif self.report_type == REPORT_RADAR_OUT:
                s = "Radar "
            elif self.report_type == REPORT_OTHER_OUT:
                s = "Other "
            s += "Sense Type: "
            if self.value == ENVIRONMENT_OUT:
                s += "Environ, "
            elif self.value == HEARTBEAT_OUT:
                s += "Heartbeat, "
            elif self.value == TOWARDS_AWAY_OUT:
                s += "Towards or Away, "
            s += "Status: "
            if self.status == NOBODY_OUT:
                s += "Nobody present"
            elif self.status == SOMEBODY_MOVE_OUT:
                s += "Body moving"
            elif self.status == SOMEBODY_STOP_OUT:
                s += "Body resting"
            elif self.status == NO_MOVE_OUT:
                s += "No movement"
            elif self.status == CA_TOWARDS_OUT:
                s += "Movement towards sensor"
            elif self.status == CA_AWAY_OUT:
                s += "Movement away from sensor"
            return s

        def evalBodySign(self, value, Move_min, Move_max):
            if(value >= Move_min and value < Move_max):
                return SOMEBODY_STOP_OUT
            elif(value < Move_min):
                return NOBODY_OUT
            elif(value >= Move_max):
                return SOMEBODY_MOVE_OUT

    def us_CalculateCrc16(self, data: bytes):
        luc_CRCHi = 0xFF
        luc_CRCLo = 0xFF
        li_Index = 0
        for c in data:
            li_Index = luc_CRCLo ^ c
            luc_CRCLo = luc_CRCHi ^ self.cuc_CRCHi[li_Index]
            luc_CRCHi = self.cuc_CRCLo[li_Index]
        return luc_CRCLo << 8 | luc_CRCHi                

    def __init__(self):
        self.MSG_LEN : byte = 12;
        self.board = None
        self.status = []
        self.new_data : bool = False
        self.msg = []
        self.data_len : byte = self.MSG_LEN
        self.recv_in_progress : bool = False
        self.last_was_head = False
        self.message_queue = []
        self.command_queue = []
        self.QUEUE_SIZE = 3
        self.msg_queue_lock = Lock()
        self.cmd_queue_lock = Lock()
        self.cuc_CRCHi = [
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
            0x00, 0xC1, 0x81, 0x40
        ]

        self.cuc_CRCLo = [
            0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
            0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
            0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
            0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
            0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
            0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
            0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
            0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
            0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
            0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
            0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
            0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
            0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB,
            0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
            0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
            0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
            0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
            0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
            0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
            0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
            0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
            0x41, 0x81, 0x80, 0x40
        ]

    def initialize(self, board : Board):
        self.board = board
        self.board.serial_config(SW_SERIAL0, 9600, 11, 12)

    def start_sensing(self):
        self.board.serial_read(SW_SERIAL0, self.MSG_LEN, self.read_callback)

    def stop_sensing(self):
        self.board.serial_flush(SW_SERIAL0)
        self.board.serial_stop(SW_SERIAL0)

    def shutdown(self):
        self.stop_sensing()
        self.board.serial_close(SW_SERIAL0)
        self.board = None

    def get_software_ver(self, callback=None):
        cmd = [0x55, 0x07, 0x00, 0x01, 0x01, 0x02]
        resp = [0x3, 0x1, 0x2]
        self.send_command(cmd, resp, callback)

    def has_message(self) -> bool:
        with self.msg_queue_lock:
            return len(self.message_queue) > 0

    # pop oldest message off so we read them in order
    def pop_message(self) -> Message:
        with self.msg_queue_lock:
            return self.message_queue.pop(0)

    def send_command(self, data, resp=None, callback=None):
        if resp is not None:
            with self.cmd_queue_lock:
                self.command_queue.append({"resp":resp, "cb": callback})
        crc_data = self.us_CalculateCrc16(data)
        data.append((crc_data & 0xff00) >> 8)
        data.append(crc_data & 0xff)

        print("Sending Msg: [", end='')
        for d in data:
            print(hex(d), end=', ')
        print(']')        
        self.board.serial_write(SW_SERIAL0, data)

    def read_callback(self, data):
        rb : byte # Each frame received
        ndx : byte = 0
        
        end = len(data)

        # handle stream of data that may include multiple messages including partial message 
        # for each full message store it in the queue
        while ndx < end:
            if not self.recv_in_progress:
                self.msg.clear()

            # while processing a single message
            while ndx < end and self.new_data == False:
                rb = data[ndx]
                if self.recv_in_progress == True:        # Received header frame
                    if self.data_len >= len(self.msg):   # Length in range
                        self.msg.append(rb)
                        if self.last_was_head:
                            if rb == MESSAGE_HEAD:
                                self.msg.pop()
                            else:
                                self.last_was_head = False
                                self.data_len = rb
                        ndx += 1
                    else:                           # Ending the information acquisition of a set of data
                        self.recv_in_progress = False
                        self.new_data = True
                        self.data_len = self.MSG_LEN
                elif rb == MESSAGE_HEAD:
                    self.msg.append(MESSAGE_HEAD)
                    self.recv_in_progress = True
                    ndx += 1
                    self.last_was_head = True

            # if complete message not yet received
            if not self.new_data:
                return

            # print("Msg: [", end='')
            # for d in self.msg:
            #     print(hex(d), end=', ')
            # print(']')

            # check if this is a response to a command sent earlier
            if len(self.msg) > 7:
                with self.cmd_queue_lock:
                    for c in self.command_queue:
                        if c['resp'] == self.msg[3:6]:
                            if c['cb'] is not None:
                                c['cb'](self.msg[6:-2])
                            self.command_queue.remove(c)
                            self.new_data = False
                            continue

            report_type, value, status = self.parseMsg(self.msg)
            self.new_data = False # Data has been used, next one can fill in
            
            if report_type != 0:
                msg = self.Message(report_type, value, status)
                with self.msg_queue_lock:
                    self.message_queue.append(msg)
                    #print(msg)
                    if len(self.message_queue) > self.QUEUE_SIZE:
                        self.message_queue.pop(0)

            #self.debug_print(report_type, value, status)
            # if report_type == BODYSIGN_OUT:
            #     result = self.evalBodySign(value, 2, 15)
            #     print(time.ctime(), ": movement = ", value, " eval = ", result)
            # elif report_type != 0:
            #     print(time.ctime(), ": report type = ", report_type, ", what = ", value, " status = ", status)

    def parseMsg(self, inf):
        #  Unpacking of physical parameters
        if(inf[3] == ACTIVE_REPORT):
            if(inf[5] == BODYSIGN):
                aa = bytearray([inf[6], inf[7], inf[8], inf[9]]) 
                value = struct.unpack('<f', aa)
                return BODYSIGN_OUT, value[0], 0
        #  Judgment of occupied and unoccupied, approach and distance
            if inf[4] == REPORT_RADAR:
                if inf[5] == ENVIRONMENT:
                    if inf[6] == NOBODY:
                        return REPORT_RADAR_OUT, ENVIRONMENT_OUT, NOBODY_OUT
                    elif inf[6] == SOMEBODY_BE:
                        if inf[7] == SOMEBODY_MOVE:
                            return REPORT_RADAR_OUT, ENVIRONMENT_OUT, SOMEBODY_MOVE_OUT
                        elif inf[7] == SOMEBODY_STOP:
                            return REPORT_RADAR_OUT, ENVIRONMENT_OUT, SOMEBODY_STOP_OUT     
                elif inf[5] == HEARTBEAT:
                    if inf[6] == NOBODY:
                        return REPORT_RADAR_OUT, HEARTBEAT_OUT, NOBODY_OUT
                    elif inf[6] == SOMEBODY_BE:
                        if inf[7] == SOMEBODY_MOVE:
                            return REPORT_RADAR_OUT, HEARTBEAT_OUT, SOMEBODY_MOVE_OUT
                        elif inf[7] == SOMEBODY_STOP:
                            return REPORT_RADAR_OUT, HEARTBEAT_OUT, SOMEBODY_STOP_OUT
                elif inf[5] == TOWARDS_AWAY:
                    if inf[6] == CA_BE:
                        if inf[7] == CA_BE:
                            if inf[8] == CA_BE:
                                return REPORT_RADAR_OUT, TOWARDS_AWAY_OUT, NO_MOVE_OUT
                            elif inf[8] == CA_TOWARDS:
                                return REPORT_RADAR_OUT, TOWARDS_AWAY_OUT, CA_TOWARDS_OUT
                            elif inf[8] == CA_AWAY:
                                return REPORT_RADAR_OUT, TOWARDS_AWAY_OUT, CA_AWAY_OUT
                elif inf[5] == ABNORMAL:
                    return EXCEPTION_OUT, 0, 0
            elif inf[4] == REPORT_OTHER:
                if inf[5] == ENVIRONMENT or inf[5] == HEARTBEAT:
                    if inf[6] == NOBODY:
                        return REPORT_OTHER_OUT, HEARTBEAT_OUT, NOBODY_OUT
                    elif inf[6] == SOMEBODY_BE:
                        if inf[7] == SOMEBODY_MOVE:
                            return REPORT_OTHER_OUT, HEARTBEAT_OUT, SOMEBODY_MOVE_OUT
                        elif inf[7] == SOMEBODY_STOP:
                            return REPORT_OTHER_OUT, HEARTBEAT_OUT, SOMEBODY_STOP_OUT
                elif inf[5] == TOWARDS_AWAY:
                    if inf[6] == CA_BE:
                        if inf[7] == CA_BE:
                            if inf[8] == CA_BE:
                                return REPORT_OTHER_OUT, TOWARDS_AWAY_OUT, NO_MOVE_OUT
                            elif inf[8] == CA_TOWARDS:
                                return REPORT_OTHER_OUT, TOWARDS_AWAY_OUT, CA_TOWARDS_OUT
                            elif inf[8] == CA_AWAY:
                                return REPORT_OTHER_OUT, TOWARDS_AWAY_OUT, CA_AWAY_OUT
                elif inf[5] == ABNORMAL:
                    return EXCEPTION_OUT, 0, 0
        return 0, 0, 0


