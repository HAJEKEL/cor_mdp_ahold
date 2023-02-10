# Represents a status to be returned from a component driving the
# LED array. Fields which are unknown for a particular implementation
# should be marked -1.

Header header

# Commit/version of firmware source.
string hardware_id

# Time since MCU power-on.
duration uptime

# Temperature in degrees celsius. Will vary with implementation,
# but may be temperature of the MCU, the DCDC, or an external probe.
float32 temperature

# Voltage of the ring LED's supply, in volts.
float32 supply_voltage

# Current consumed by the ring LED, averaged over the reporting
# window, in amps.
float32 supply_current

# Bitfield for various possible error modes.
uint8 FAULT_NONE=0
uint8 FAULT_COMMAND_STALE=1
uint8 FAULT_MISSING_SINGLE=2
uint8 FAULT_MISSING_MULTIPLE=4
uint8 FAULT_MISSING_ALL=8
uint8 FAULT_VOLTAGE=16
uint8 FAULT_UNKNOWN=128
uint8 fault
