#!/usr/bin/env python3
import os.path
import rospy, rospkg
import yaml
import pybullet as pb
import pybullet_data
import pybullet_controller.pybullet_controller as pbc



# Path to this package
PKG_PATH    = rospkg.RosPack().get_path('alfarobi_main')

# Paths to the .urdf files
PLANE_PATH = os.path.join(pybullet_data.getDataPath(), 'samurai.urdf')
ROBOT_PATH = PKG_PATH + '/src/simulation/robot_description/urdf/robotis_op3.urdf'



# Parse robot_information.yaml and simulation_settings.ymal
robot_information_yaml      = open(PKG_PATH + '/config/robot_information.yaml', 'r')
robot_information_config    = yaml.load(robot_information_yaml, Loader=yaml.SafeLoader)

simulation_settings_yaml    = open(PKG_PATH + '/config/simulation_settings.yaml', 'r')
simulation_settings_config  = yaml.load(simulation_settings_yaml, Loader=yaml.SafeLoader)



# Start ROS node
rospy.init_node(
    name        = robot_information_config['name'] + '_python_publisher_node', 
    anonymous   = False
)
ros_rate = rospy.Rate(simulation_settings_config['simulation_fps'])



# Start physics client and load .urdf files
physics_client  = pb.connect(pb.GUI)
pb.setGravity(0, 0, simulation_settings_config['gravity'])
plane_id    = pb.loadURDF(PLANE_PATH)
robot_id    = pb.loadURDF(
   ROBOT_PATH, 
   [0, 0, 0.27],                          # starting position
   pb.getQuaternionFromEuler([0, 0, 0])   # starting orientation
)
robot_inst = pbc.PybulletController(
    name            = robot_information_config['name'], 
    plane_id        = plane_id, 
    robot_id        = robot_id, 
    joint_config    = simulation_settings_config['joint_config']
)



# Start simulation
if __name__ == '__main__':


    try:
        while not rospy.is_shutdown():
            


            # Read and write the joints' state
            # Increment the simulation
            # Sleep within the fps
            robot_inst.readWrite()
            pb.stepSimulation()
            ros_rate.sleep()


    except rospy.ROSInterruptException:
        pb.disconnect()