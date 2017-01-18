cmake_minimum_required(VERSION 2.8.3)
project(roch_navigation)

find_package(catkin REQUIRED roslaunch tf roscpp sensor_msgs)

# What other packages will need to use this package
catkin_package(
    CATKIN_DEPENDS tf roscpp sensor_msgs
)

###########
## Build ##
###########
include_directories(${catkin_INCLUDE_DIRS})

roslaunch_add_file_check(launch)
catkin_add_env_hooks(25.roch-navigation
   SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
install(
  DIRECTORY param launch maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

