# === Main State Constants ===
uint8 INITIALIZING = 0
uint8 CHARGING = 1
uint8 IDLE = 2
uint8 MOVING_TO_CHARGER = 3
uint8 PICKING_UP_BOOK = 4
uint8 RESHELVING_BOOK = 5
uint8 GUIDING = 6
uint8 CLEANING_DESK = 7
uint8 SORTING_SHELVES = 8
uint8 FORCE_MOVE_TO_CHARGER = 9
uint8 LISTENING = 10
uint8 ROAMING = 11
uint8 EMERGENCYC_STOP = 98
uint8 MAIN_ERROR = 99

# ----------------------------
uint8 main_state


# === Sub State Constants ===
# General States
uint8 NONE = 100

# Book Pickup Task 
uint8 MOVE_TO_PICKUP = 101
uint8 PICKUP_BOOK = 102
uint8 MOVE_TO_STORAGE = 103
uint8 STOWING_BOOK = 104

# Reshelve Book Task
uint8 MOVE_TO_RETURN_DESK = 105 
uint8 COLLECT_RETURN_BOOKS = 106
uint8 MOVE_TO_PLACE_SHELF = 107
uint8 PLACE_RETURN_BOOK = 108

# Guiding Task
uint8 SELECT_DEST = 109
uint8 SCAN_USER = 110
uint8 GUIDING_TO_DEST = 111
uint8 FIND_USER = 112

# Cleaning Desk Task
uint8 MOVE_TO_DESK = 113
uint8 SCAN_DESK = 114
uint8 CLEANING_TRASH = 115
uint8 MOVE_TO_BIN = 116
uint8 TIDYING_SHELVES = 117

# Sorting Shelf Task
uint8 MOVE_TO_SHELF = 118
uint8 SCAN_BOOK = 119
uint8 SORT_BOOK = 120
uint8 SUB_ERROR = 199
# ----------------------------
uint8 sub_state


# Error status
bool is_error
string error_message