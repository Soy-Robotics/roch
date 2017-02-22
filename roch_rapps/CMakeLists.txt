cmake_minimum_required(VERSION 2.8.3)
project(roch_rapps)

find_package(catkin REQUIRED)

catkin_package()


 install(DIRECTORY rapps media
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
