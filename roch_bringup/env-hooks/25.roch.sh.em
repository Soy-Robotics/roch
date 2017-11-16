# Set some sane defaults for the roch launch environment

##Documentation: 
#  The colon command simply has its arguments evaluated and then succeeds. 
#   It is the original shell comment notation (before '#' to end of line). For a long time, Bourne shell scripts had a colon as the first character. 
#   The C Shell would read a script and use the first character to determine whether it was for the C Shell (a '#' hash) or the Bourne shell (a ':' colon).
#   Then the kernel got in on the act and added support for '#!/path/to/program' and the Bourne shell got '#' comments, and the colon convention went by the wayside. 
#   But if you come across a script that starts with a colon (Like this one), now you will know why. ~ Jonathan Leffler

: ${ROCH_BASE:=roch}                             # roch
: ${ROCH_STACKS:=standard}                       # standard
: ${ROCH_3D_SENSOR:=astra}				 		 # kinect, asus_xtion_pro, asus_xtion_pro_offset, r200, astra
: ${ROCH_3D_SENSOR_ENABLE:=TRUE}             	 # enable 3d sensor
: ${ROCH_3D_SENSOR_NAV_ENABLE:=FALSE}			 # enable 3d sensor for navigation 
: ${ROCH_LASER:=rplidar}				 		 # ls01c, rplidar, sicklmsxx
: ${ROCH_LASER_ENABLE:=TRUE}				   	 # enable laser
: ${ROCH_SIMULATION:=false}
: ${ROCH_SERIAL_PORT:=/dev/roch}                 # /dev/ttyUSB0, /dev/ttyS0

: ${ROCH_NAME:=roch}
: ${ROCH_TYPE:=roch}
: ${ROCH_RAPP_PACKAGE_WHITELIST:=[rocon_apps, roch_rapps]}
: ${ROCH_RAPP_PACKAGE_BLACKLIST:=[]}
: ${ROCH_INTERACTIONS_LIST:=[roch_bringup/admin.interactions, roch_bringup/documentation.interactions, roch_bringup/pairing.interactions, roch_bringup/visualisation.interactions]}

# Exports
export ROCH_BASE
export ROCH_BATTERY
export ROCH_STACKS
export ROCH_3D_SENSOR
export ROCH_3D_SENSOR_ENABLE
export ROCH_3D_SENSOR_NAV_ENABLE
export ROCH_LASER
export ROCH_LASER_ENABLE
export ROCH_SIMULATION
export ROCH_SERIAL_PORT
export ROCH_NAME
export ROCH_TYPE
export ROCH_RAPP_PACKAGE_WHITELIST
export ROCH_RAPP_PACKAGE_BLACKLIST
export ROCH_INTERACTIONS_LIST

