# Information about the sources of various assertions (e.g. states).

# Published by robot_state_aggregator to indicate which sources are asserting which states. Consumed primarily by rosbag
# for the purposes of understanding why a robot was doing what it was. It is primarily of value for robot states that
# can be asserted multiple ways. It can also be echoed as a quick visual check by name of which states are asserted.

Header header

# The list of possible assertions and corresponding sources (empty sources means not asserted).
Sources[] sources_list
