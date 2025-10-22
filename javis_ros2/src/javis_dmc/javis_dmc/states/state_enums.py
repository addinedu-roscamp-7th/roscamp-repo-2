from enum import IntEnum


class RobotMode(IntEnum):
    '''DMC 운영 모드 정의.'''

    STANDBY = 0
    AUTONOMY = 1


class MainState(IntEnum):
    '''DMC 메인 상태 정의 (문서 v6.0 기준).'''

    INITIALIZING = 0
    CHARGING = 1
    IDLE = 2
    MOVING_TO_CHARGER = 3
    PICKING_UP_BOOK = 4
    RESHELVING_BOOK = 5
    GUIDING = 6
    CLEANING_DESK = 7
    SORTING_SHELVES = 8
    FORCE_MOVE_TO_CHARGER = 9
    LISTENING = 10
    ROAMING = 11
    EMERGENCYC_STOP = 98
    MAIN_ERROR = 99


class SubState(IntEnum):
    '''DMC 서브 상태 정의.'''

    NONE = 100

    MOVE_TO_PICKUP = 101
    PICKUP_BOOK = 102
    MOVE_TO_STORAGE = 103
    STOWING_BOOK = 104

    MOVE_TO_RETURN_DESK = 105
    COLLECT_RETURN_BOOKS = 106
    MOVE_TO_PLACE_SHELF = 107
    PLACE_RETURN_BOOK = 108

    SELECT_DEST = 109
    SCAN_USER = 110
    GUIDING_TO_DEST = 111
    FIND_USER = 112

    MOVE_TO_DESK = 113
    SCAN_DESK = 114
    CLEANING_TRASH = 115
    MOVE_TO_BIN = 116
    TIDYING_SHELVES = 117

    MOVE_TO_SHELF = 118
    SCAN_BOOK = 119
    SORT_BOOK = 120

    SUB_ERROR = 199
