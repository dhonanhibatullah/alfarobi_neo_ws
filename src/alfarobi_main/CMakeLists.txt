cmake_minimum_required(VERSION 3.0.2)
project(alfarobi_main)

# List all the dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  alfarobi_lib
)

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