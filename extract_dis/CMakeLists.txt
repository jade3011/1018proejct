cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++14)

project(extract_dis)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  laser_geometry
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  darknet_ros_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES targetless
    CATKIN_DEPENDS cv_bridge pcl_ros roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE project_HEADERS
    "*.h"
    "*.hpp"
)

add_executable(
${PROJECT_NAME}
${project_HEADERS}
src/main.cpp
src/TrackAssociation.cpp
src/extractDis.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

