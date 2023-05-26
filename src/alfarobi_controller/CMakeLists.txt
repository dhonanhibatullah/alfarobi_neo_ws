cmake_minimum_required(VERSION 3.0.2)
project(alfarobi_controller)

# List all the dependencies
find_package(catkin REQUIRED COMPONENTS
  alfarobi_lib
  roscpp
  rospy
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

# Compile libraries
add_library(StableWalk src/alfarobi_controller/dynamics/StableWalk.cpp)
add_library(BodyPose src/alfarobi_controller/kinematics/BodyPose.cpp)