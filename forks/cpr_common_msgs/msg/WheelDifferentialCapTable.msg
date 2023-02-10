# Contains the Wheel-Differential Cap Table that is shared between a
# robot's Velocity Interface and the firmware to enforce a maximum
# allowed turning-rate for a given highest-wheel-speed.
# The table contents are defined in Velocity Interface's launch file.

std_msgs/Header header

# Core cap-table
WheelDifferentialCap[] rows

# Minimum speed both wheels must be moving before enforcement begins.
# Units MUST be consistant with the speed/velocity units used in the Cap Table!!!
float32 critical_speed

# Amount of time (in milliseconds) the cap table may be violated before throwing an E-stop
uint32 fault_confirmation_time_ms

# Salt value (pseudo-random number) added to checksum calculation
uint32 salt
