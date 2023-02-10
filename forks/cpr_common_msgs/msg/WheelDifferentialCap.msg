# Defines a single row in a shared Wheel-Differential Cap Table, used
# by the robot's Velocity Interface and platform firmware to enforce safe
# driving conditions while in Normal Mode.

# Represents the upper limit for a particular 'speed bucket', which
# is the larger of two wheel speeds (left and right) at a given
# moment.
# The absolute value of the speed is taken, values can be 0.0 or
# greater.  Future use cases may involve negative values for
# backwards movement.
float32 velocity  # rads/s of the fastest wheel

# The maximum allowed difference in speeds between the left and right
# wheels for the given 'speed bucket'.  Values  will always be positive
# while OTTO is in 'Normal Mode'.
# Values can be between 0.0 and 1.0 inclusive.
float32 wheel_differential  # % difference allowed in wheel speeds
