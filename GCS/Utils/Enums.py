from enum import Enum


class DRONE_STATE(Enum):
    LANDED = 0
    HOVER = 1
    FLYING = 2
    ERROR = 3
