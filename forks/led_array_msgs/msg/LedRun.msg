# Defines a run of same-colour LEDs on the array, for compact encoding purposes.
# This message is intended for use in the context of the LedState message.

# Colour values are from 0 (least intense) to 255 (most intense).
uint8 red
uint8 green
uint8 blue

# Number of LEDs to be this colour in a row.
uint8 run_length
