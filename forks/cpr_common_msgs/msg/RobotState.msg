# See https://wiki.clearpathrobotics.com/display/AUT/Design%3A+Robot+State+API for more information.

# Bit constants (must have named constants for all bits between MIN_CONSTANT and MAX_CONSTANT, inclusive)
uint32 MIN_CONSTANT = 1
# Emergency stop - user intervention required.
uint32 STATE_E_STOP = 1
# Safety stop.
uint32 STATE_SAFETY_STOP = 2
# Robot is safe to approach and work may be done on payload.
uint32 STATE_SAFE_TO_LOAD = 4
# Robot is operating with reduced LIDAR fields and speed.
uint32 STATE_NARROW_HALLWAY = 8
# Robot is operating with greatly reduced LIDAR fields and speed. Caution should be exercised when approaching robot.
uint32 STATE_DOCKING = 16
# Robot is charging.
uint32 STATE_CHARGING = 32
# Robot is parked and has no intent of moving.
uint32 STATE_PARKED = 64
# Movement is indicated.
uint32 STATE_ABOUT_TO_MOVE = 128
# Robot is paused and cannot move.
# If in autonomous, this means robot autonomy or robot core is paused, if in manual, robot core is paused.
uint32 STATE_PAUSED = 256
# Robot is being controlled by teleoperation - cannot be driven autonomously.
uint32 STATE_MANUAL = 512
# Robot is autonomous.
uint32 STATE_AUTONOMY = 1024
# Robot is being controlled by the pendant - cannot be driven autonomously.
uint32 STATE_PENDANT = 2048
# Payload appliance is active.
uint32 STATE_APPLIANCE_ACTIVE = 4096
# Robot is creating a new map. Or retraining an existing map.
uint32 STATE_TRAINING = 8192
# Robot is attempting to localize.
uint32 STATE_LOCALIZING = 16384
# Robot has localized.
uint32 STATE_LOCALIZED = 32768
# Robot has lost localization and may need user intervention.
uint32 STATE_LOST = 65536
# Robot is actively executing a job issued by Nimbus.
uint32 STATE_EXECUTING_JOB = 131072
#
uint32 STATE_PLANNING = 262144
#
uint32 STATE_TRACKING = 524288
# Robot is blocked from moving and can't replan.
uint32 STATE_BLOCKED = 1048576
# Robot has lost connection to Nimbus.
uint32 STATE_NIMBUS_LOST = 2097152
uint32 MAX_CONSTANT = 2097152

# State will be some combination of the bit constants
uint32 state
