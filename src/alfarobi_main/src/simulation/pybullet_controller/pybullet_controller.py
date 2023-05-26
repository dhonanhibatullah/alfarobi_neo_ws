import rospy
import pybullet as pb
import alfarobi_lib.msg as alfarobi_lib 



class PybulletController:



    def __init__(self, name, plane_id, robot_id, joint_config:dict, maximum_force:float=6.4) -> None:
        # Controller parameters
        self.MAXIMUM_FORCE  = maximum_force
        self.JOINT_CONFIG   = joint_config


        # Get the plane and robot id
        self.robot_name = name
        self.plane_id   = plane_id
        self.robot_id   = robot_id


        # Get the robot joints
        self.num_joints     = pb.getNumJoints(self.robot_id) - 2
        self.joints_name    = []
        self.joints_idx     = {}
        self.joints_val     = []
        for joint_idx in range(self.num_joints):
            # Save joints' info
            joint_name = pb.getJointInfo(self.robot_id, joint_idx)[1].decode()
            self.joints_name.append(joint_name)
            self.joints_idx[joint_name] = joint_idx

            # Get state values (theta, time, current theta, multiplier)
            if joint_name != 'cam' and joint_name != 'cam_gazebo':
                self.joints_val.append([0, 0, 0, joint_config[joint_name]])


        # Configure all the joints for initialization
        pb.setJointMotorControlArray(
            self.robot_id,
            range(self.num_joints),
            controlMode         = pb.POSITION_CONTROL,
            targetPositions     = [0]*self.num_joints,
            forces              = [self.MAXIMUM_FORCE]*self.num_joints
        )


        # ROS publisher and subscribers
        self.joint_current_val_pub  = rospy.Publisher(name + '/joint_current_val', alfarobi_lib.JointVal, queue_size=1000)
        self.joint_target_val_sub   = rospy.Subscriber(name + '/joint_target_val', alfarobi_lib.JointVal, self.jointTargetValCallback)


        # ROS messages
        self.joint_current_val_msg = alfarobi_lib.JointVal()



    def getJointNameByIdx(self, joint_idx:int) -> str:
        return self.joints_name[joint_idx]



    def getJointIdxByName(self, joint_name:str) -> int:
        return self.joints_idx[joint_name]



    def writeJointValue(self) -> None:
        # Set value to joint servos
        pb.setJointMotorControlArray(
            self.robot_id,
            range(self.num_joints),
            controlMode         = pb.POSITION_CONTROL,
            targetPositions     = [row[0]*row[3] for row in self.joints_val],
            forces              = [self.MAXIMUM_FORCE]*self.num_joints
        )



    def readJointValue(self) -> None:
        # Read joints' states
        for joint_idx in range(self.num_joints-2):
            joint_state = pb.getJointState(self.robot_id, joint_idx)
            self.joints_val[joint_idx][2] = joint_state[0]*self.joints_val[joint_idx][3]

        # Publish the data
        self.joint_current_val_msg.l_hip_yaw.data   = self.joints_val[0][0]
        self.joint_current_val_msg.l_hip_roll.data  = self.joints_val[1][0]
        self.joint_current_val_msg.l_hip_pitch.data = self.joints_val[2][0]
        self.joint_current_val_msg.l_knee.data      = self.joints_val[3][0]
        self.joint_current_val_msg.l_ank_pitch.data = self.joints_val[4][0]
        self.joint_current_val_msg.l_ank_roll.data  = self.joints_val[5][0]
        self.joint_current_val_msg.r_hip_yaw.data   = self.joints_val[6][0]
        self.joint_current_val_msg.r_hip_roll.data  = self.joints_val[7][0]
        self.joint_current_val_msg.r_hip_pitch.data = self.joints_val[8][0]
        self.joint_current_val_msg.r_knee.data      = self.joints_val[9][0]
        self.joint_current_val_msg.r_ank_pitch.data = self.joints_val[10][0]
        self.joint_current_val_msg.r_ank_roll.data  = self.joints_val[11][0]
        self.joint_current_val_msg.l_sho_pitch.data = self.joints_val[12][0]
        self.joint_current_val_msg.l_sho_roll.data  = self.joints_val[13][0]
        self.joint_current_val_msg.l_el.data        = self.joints_val[14][0]
        self.joint_current_val_msg.r_sho_pitch.data = self.joints_val[15][0]
        self.joint_current_val_msg.r_sho_roll.data  = self.joints_val[16][0]
        self.joint_current_val_msg.r_el.data        = self.joints_val[17][0]
        self.joint_current_val_msg.head_pan.data    = self.joints_val[18][0]
        self.joint_current_val_msg.head_tilt.data   = self.joints_val[19][0]
        self.joint_current_val_pub.publish(self.joint_current_val_msg)



    def readWrite(self) -> None:
        self.writeJointValue()
        self.readJointValue()



    def jointTargetValCallback(self, data):
        self.joints_val[0][0]   = data.l_hip_yaw.data
        self.joints_val[1][0]   = data.l_hip_roll.data
        self.joints_val[2][0]   = data.l_hip_pitch.data
        self.joints_val[3][0]   = data.l_knee.data
        self.joints_val[4][0]   = data.l_ank_pitch.data
        self.joints_val[5][0]   = data.l_ank_roll.data
        self.joints_val[6][0]   = data.r_hip_yaw.data
        self.joints_val[7][0]   = data.r_hip_roll.data
        self.joints_val[8][0]   = data.r_hip_pitch.data
        self.joints_val[9][0]   = data.r_knee.data
        self.joints_val[10][0]  = data.r_ank_pitch.data
        self.joints_val[11][0]  = data.r_ank_roll.data
        self.joints_val[12][0]  = data.l_sho_pitch.data
        self.joints_val[13][0]  = data.l_sho_roll.data
        self.joints_val[14][0]  = data.l_el.data
        self.joints_val[15][0]  = data.r_sho_pitch.data
        self.joints_val[16][0]  = data.r_sho_roll.data
        self.joints_val[17][0]  = data.r_el.data
        self.joints_val[18][0]  = data.head_pan.data
        self.joints_val[19][0]  = data.head_tilt.data

