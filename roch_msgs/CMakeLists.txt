cmake_minimum_required(VERSION 2.8.3)
project(roch_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  sensor_msgs
  std_srvs
)  
    
add_message_files(
  FILES
  RochStatus.msg
  PanoramaImg.msg
)
  
add_service_files(
  DIRECTORY srv
  FILES 
  TakePanorama.srv
  SetFollowState.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs sensor_msgs std_srvs message_runtime)
