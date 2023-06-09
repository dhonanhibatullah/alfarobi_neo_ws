cmake_minimum_required(VERSION 3.0.2)
project(alfarobi_controller)

# List all the dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  alfarobi_lib
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
add_executable(controller_main_node 
  src/controller_main.cpp
  src/alfarobi_controller/dynamics/StableWalk.cpp
  src/alfarobi_controller/kinematics/PosePlayer.cpp
)
target_link_libraries(controller_main_node linear_algebra plotter yaml-cpp ${catkin_LIBRARIES})