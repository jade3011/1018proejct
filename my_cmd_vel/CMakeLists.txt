cmake_minimum_required(VERSION 3.0.2)
project(my_cmd_vel)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_runtime
  sensor_msgs
  std_msgs
  nav_msgs
  roscpp
  pcl_ros
  tf
)

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#  cmd_vel_msg.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  geometry_msgs
#  nav_msgs
#)


catkin_package(
# INCLUDE_DIRS include
# LIBRARIES my_cmd_vel
 CATKIN_DEPENDS geometry_msgs roscpp nav_msgs

# DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS})
#include_directories(include/my_cmd_vel/)

## Declare a C++ library

add_executable(cmd_vel_pub
    src/cmd_vel_pub.cpp
    include/my_cmd_vel/cmd_vel_pub.h
    )
target_link_libraries(cmd_vel_pub ${catkin_LIBRARIES})

add_dependencies(cmd_vel_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#add_dependencies(cmd_vel_pub ${catkin_EXPORTED_TARGETS)
## Specify libraries to link a library or executable target against

