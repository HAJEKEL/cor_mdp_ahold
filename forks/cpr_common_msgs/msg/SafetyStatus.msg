# Indications
uint8 E_STOP = 0
uint8 SAFETY_STOP = 1
uint8 WARNING = 2
uint8 SAFE_TO_LOAD = 3
uint8 NORMAL_TRAVEL = 4
uint8 ABOUT_TO_MOVE = 5
uint8 NARROW_HALLWAY = 6
uint8 DOCKING = 7
uint8 CHARGING = 8
uint8 ROBOT_LOST = 9
uint8 PARKED = 10
uint8 MANUAL = 11
uint8 POWER_ON = 12
uint8 APPLIANCE_ACTIVE = 13

#used to indicate that priority LED status can now
#be changed to a non-priority status
uint8 CLEAR_PRIORITY = 99

# Direction constants (deprecated, used only by legacy led_array)
uint8 FORWARD = 0
uint8 REVERSE = 1
uint8 LEFT_TURN = 2
uint8 RIGHT_TURN = 3

uint8 indication
uint8 direction
float64 angle
