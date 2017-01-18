# Provides a cliff sensor event.
# This message is generated whenever a particular cliff sensor signals that the
# robot approaches or moves away from a cliff.
# Note that, despite cliff field on SensorState messages, state field is not a
# bitmask, but the new state of a single sensor.

# cliff sensor
uint8 LEFT   = 0
uint8 RIGHT  = 1

# cliff sensor state
uint8 FLOOR = 0
uint8 CLIFF = 1

uint8 leftSensor
uint8 leftState

# distance to floor when cliff was detected
float64 leftBottom

uint8 rightSensor
uint8 rightState

# distance to floor when cliff was detected
float64 rightBottom
