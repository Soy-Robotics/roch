cmake_minimum_required(VERSION 2.8.3)
project(roch_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  sensor_msgs
  std_srvs
  actionlib_msgs
)  
    
add_message_files(
  FILES
  RochStatus.msg
  PanoramaImg.msg
  CliffEvent.msg
  UltEvent.msg
  PSDEvent.msg
  SensorState.msg
)
  
add_service_files(
  DIRECTORY srv
  FILES 
  TakePanorama.srv
  SetFollowState.srv
)

add_action_files(
   DIRECTORY action
   FILES 
   AutoDocking.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  actionlib_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs std_srvs message_runtime actionlib_msgs)
