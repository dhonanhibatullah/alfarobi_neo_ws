cmake_minimum_required(VERSION 3.0.2)
project(alfarobi_lib)

# List all the dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  message_generation
)

# Add custom messages
add_message_files(
  FILES
    JointTargetVal.msg
    JointCurrentVal.msg
)
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Add as a package
catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS
    message_runtime
)
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# Add library
add_library(linear_algebra src/alfarobi_lib/LinearAlgebra.cpp)
add_library(plotter src/alfarobi_lib/Plotter.cpp)