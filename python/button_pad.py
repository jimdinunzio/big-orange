from threading import Thread
from pyFirmata.pyfirmata import INPUT, Board
import time
import my_sdp_client
import sdp_comm

class Button4Pad(object):
    """Driver class for 4 button pad"""

    def __init__(self):
      self.board : Board = None
      self.buttons = []
      self.btn_state_mask = 0
      self.buttonEventCb = None

    def initialize(self, board: Board, buttonEventCb=None):
      self.board = board
      self.buttons.append(board.get_pin('d:8:ipu'))
      self.buttons.append(board.get_pin('d:7:ipu'))
      self.buttons.append(board.get_pin('d:2:ipu'))
      self.buttons.append(board.get_pin('d:4:ipu'))

      for btn in self.buttons:
          btn.enable_reporting()

      self.buttonEventCb = buttonEventCb

    def get_button(self, index):
      return not self.buttons[index].read()

    def get_btn_state_mask(self):
        btn_state_mask = 0
        for i in range(0, 4):
          btn_state_mask |= self.get_button(i) << i 
        return btn_state_mask

    def shutdown(self):
      self.run_flag = False
      for btn in self.buttons:
        btn.disable_reporting()
        btn.mode = INPUT

    def startUp(self, connect_sdp=True):
      if connect_sdp:
        sdp = my_sdp_client.MyClient()
        connected = sdp_comm.connectToSdp(sdp) == 0
      else:
        sdp = None
        connected = False

      self.run_flag = True
      time.sleep(0.25)
      while self.run_flag:
        new_btnState_mask = self.get_btn_state_mask()
        change_mask = self.btn_state_mask ^ new_btnState_mask
        self.btn_state_mask = new_btnState_mask
        if change_mask != 0 and self.buttonEventCb is not None:
          self.buttonEventCb(change_mask, self.btn_state_mask, sdp)
        time.sleep(0.1)     
      if sdp is not None:
        sdp.disconnect()
        sdp.shutdown_server32(kill_timeout=1)
        sdp = None     
        
if __name__ == '__main__':
  from button_pad import Button4Pad
  from latte_panda_arduino import LattePandaArduino

  def buttonEventCb(change_mask, button_state_mask, sdp):
      for i in range(0, 4):
        if change_mask & 1<<i != 0: 
          print("button {} {}.".format(i+1, button_state_mask & 1<<i != 0  and "pressed." or "released."))

  _lpArduino = LattePandaArduino()
  _lpArduino.initialize()
  print("initializing arduino")
  bp = Button4Pad()
  print("initializing Button4Pad")
  bp.initialize(_lpArduino.board, buttonEventCb)

  monitor = Thread(target=bp.startUp, args=(False,), name="button_pad")
  print("Starting monitor thread")
  monitor.start()

  try:
    while True:
      time.sleep(1)
  except KeyboardInterrupt:
    print("monitoring for button presses")
    bp.shutdown()
    monitor.join()
    _lpArduino.shutdown()
