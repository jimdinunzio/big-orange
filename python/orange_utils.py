from enum import Enum

class OrangeOpType(Enum):
    """Enumerated type for operation type"""
    def __init__(self, number):
        self._as_parameter__ = number

    TextCommand = 1
    BatteryPercent = 2
    LastSpeechHeard = 3
    LastSpeechSpoken = 4
    IpAddress = 5
    GoogleSpeech = 6
    ToggleGoogleSpeech = 7
    InternetStatus = 8
    BatteryIsCharging = 9
    BoardTemperature = 10
    LocalizationQuality = 11
    