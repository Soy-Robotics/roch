cmake_minimum_required(VERSION 2.8.3)
project(roch_bringup)

find_package(catkin REQUIRED COMPONENTS roslaunch)

catkin_package()
catkin_add_env_hooks(25.roch
  SHELLS sh DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)


roslaunch_add_file_check(launch)

install(PROGRAMS scripts/install scripts/calibrate_compass scripts/compute_calibration 
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch  config param images interactions test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
