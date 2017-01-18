# Provides a ult sensor event.
# This message is generated whenever a particular cliff sensor signals that the
# robot approaches or moves away from a cliff.
# Note that, despite cliff field on SensorState messages, state field is not a
# bitmask, but the new state of a single sensor.

# ult sensor
uint8 LEFT   = 0
uint8 CENTER = 1
uint8 RIGHT  = 2

# ult sensor state
uint8 NORMAL = 0
uint8 NEAR = 1

uint8 leftSensor
uint8 leftState

# distance to floor when cliff was detected
float64 leftBottom

uint8 centerSensor
uint8 centerState

# distance to floor when cliff was detected
float64 centerBottom

uint8 rightSensor
uint8 rightState

# distance to floor when cliff was detected
float64 rightBottom
