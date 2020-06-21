cmake_minimum_required(VERSION 2.8.3)
project(proto_unit2)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation 
)

## Generate messages in the 'msg' folder
 #add_message_files(
   #FILES
 #)

## Generate services in the 'srv' folder
 #add_service_files(
  #FILES
 #)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime  #Agregado!!
)

## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


###TOPICOS####

add_executable(Distance_Control ~/tiago_public_ws/src/proto_unit2/src/Distance_Control.cpp)
target_link_libraries(Distance_Control ${catkin_LIBRARIES})
add_dependencies(Distance_Control proto_unit2_generate_messages_cpp)


