from pyfirmata import ArduinoLeonardo, util

class LattePandaArduino:
    def __init__(self):
        self._board = None
    
    def __del__(self):
        if self._board is not None:
            self._board.exit()
            self._board = None
            
    def initialize(self):
        self._board = ArduinoLeonardo('COM6')
        iter = util.Iterator(self._board)
        iter.start()

    @property
    def board(self):
        return self._board
        
    def shutdown(self):
        if self._board is not None:
            self._board.exit()
            self._board = None
