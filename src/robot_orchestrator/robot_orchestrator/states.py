from enum import Enum

class RobotState(Enum):
    INIT = 0
    LOCALIZATION = 1
    NAVIGATION = 2
    MISSION_COMPLETE = 3
    STOP = 4
