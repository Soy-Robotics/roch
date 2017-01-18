# roch Sensor Data Messages
#
# For more direct simple interactions (buttons, leds, gyro, motor velocity
# etc) use the other topics. This provides detailed information about the
# entire state package that is transmitted at 50Hz from the robot.
#


###### CONSTANTS ######

# Wheel drop sensor states (states are combined, when both wheel drop sensors are triggered)
uint8 RAISED  = 0
uint8 DROPPED = 1

# Cliff sensor states (states are combined, when multiple cliff sensors are triggered)
uint8 FLOOR = 0
uint8 CLIFF = 1

# Ult/psd sensor states (states are combined, when multiple ult sensors are triggered)
uint8 NORMAL = 0
uint8 NEAR = 1

###### MESSAGE ######

Header header

###################
# Core Packet
###################
uint8  leftwheel_drop      # see wheel drop sensor states
uint8  rightwheel_drop      # see wheel drop sensor states
uint8  leftcliff           # see cliff sensor states
uint8  rightcliff           # see cliff sensor states
uint8  leftult             # see ult sensor states
uint8  centerult             # see ult sensor states
uint8  rightult             # see ult sensor states
uint8  leftpsd             # see psd sensor states
uint8  centerpsd             # see psd sensor states
uint8  rightpsd             # see psd sensor states

###################
# Cliff Packet
###################
float64[] cliffbottom        # ADC output of the right, centre, left cliff PSD sensor (0 - 0.2(meter), distance measure is non-linear)

###################
# Ult Packet
###################
float64[] ultbottom        # ADC output of the right, centre, left Ult sensor (0 - 0.6(meter), distance measure is non-linear)

###################
# PSD Packet
###################
float64[] psdbottom        # ADC output of the right, centre, left PSD sensor (0 - 0.2(meter), distance measure is non-linear)


