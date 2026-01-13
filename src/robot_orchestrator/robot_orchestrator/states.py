from enum import Enum

class RobotState(Enum):
    INIT = 0
    LOCALIZATION = 1
    NAVIGATION = 2
    DETECTION = 3
    AVOID_OBSTACLE = 4
    OBJECT_FOUND = 5
    STOP = 6
