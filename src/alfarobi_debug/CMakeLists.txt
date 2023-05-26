cmake_minimum_required(VERSION 3.0.2)
project(alfarobi_debug)

# List all the dependencies
find_package(catkin REQUIRED COMPONENTS
  alfarobi_lib
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
)
find_package(yaml-cpp REQUIRED)

# This is basically used in every packages
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    message_runtime 
    alfarobi_lib
)

# Include header files from alfarobi_lib package
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${alfarobi_lib_INCLUDE_DIRS}
)

# Compile executables
add_executable(plotter_test src/plotter_test.cpp)
target_link_libraries(plotter_test ${catkin_LIBRARIES})