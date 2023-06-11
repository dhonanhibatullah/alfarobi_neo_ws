import math
import rospy
import pybullet as pb
import alfarobi_lib.msg as alfarobi_lib 



class PybulletController:



    def __init__(self, name, plane_id, robot_id, joint_config:dict, time_config, maximum_force:float=6.2) -> None:
        # Controller parameters
        self.MAXIMUM_FORCE  = maximum_force
        self.JOINT_CONFIG   = joint_config
        self.STEP_TIME      = 1./time_config
        self.JOINT_ERROR    = 0.007


        # Get the plane and robot id
        self.robot_name = name
        self.plane_id   = plane_id
        self.robot_id   = robot_id


        # Variables
        self.update_vel = False
        self.update_pos = False


        # Get the robot joints
        self.num_joints     = pb.getNumJoints(self.robot_id) - 2
        self.joints_name    = []
        self.joints_idx     = {}
        self.joints_val     = []
        self.joints_dxl_val = []
        self.joints_inc     = []
        for joint_idx in range(self.num_joints):
            # Save joints' info
            joint_name = pb.getJointInfo(self.robot_id, joint_idx)[1].decode()
            self.joints_name.append(joint_name)
            self.joints_idx[joint_name] = joint_idx

            # Get state values (theta, time, current theta, multiplier)
            if joint_name != 'cam' and joint_name != 'cam_gazebo':
                self.joints_val.append([0, 0, 0, joint_config[joint_name]])
                self.joints_dxl_val.append(0)
                self.joints_inc.append(0)


        # Configure all the joints for initialization
        pb.setJointMotorControlArray(
            self.robot_id,
            range(self.num_joints),
            controlMode         = pb.POSITION_CONTROL,
            targetPositions     = [0]*self.num_joints,
            forces              = [self.MAXIMUM_FORCE]*self.num_joints
        )


        # ROS publisher and subscribers
        self.joint_current_val_pub  = rospy.Publisher(name + '/joint_current_val', alfarobi_lib.JointCurrentVal, queue_size=1000)
        self.joint_target_val_sub   = rospy.Subscriber(name + '/joint_target_val', alfarobi_lib.JointTargetVal, self.jointTargetValCallback)


        # ROS messages
        self.joint_current_val_msg = alfarobi_lib.JointCurrentVal()



    def getJointNameByIdx(self, joint_idx:int) -> str:
        return self.joints_name[joint_idx]



    def getJointIdxByName(self, joint_name:str) -> int:
        return self.joints_idx[joint_name]



    def writeJointValue(self) -> None:
        for joint_idx in range(self.num_joints):
            des  = self.dxl2Radian(self.joints_dxl_val[joint_idx])
            diff = des - self.joints_val[joint_idx][0]

            if math.fabs(diff) < self.JOINT_ERROR:
                self.joints_val[joint_idx][0] = des
            
            else:
                self.joints_val[joint_idx][0] += self.joints_inc[joint_idx]

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
        conv_const = 651.739492
        converse = lambda theta : theta if theta > 0 else 2*math.pi + theta

        self.joint_current_val_msg.l_hip_yaw.data   = int(converse(self.joints_val[0][2])*conv_const)
        self.joint_current_val_msg.l_hip_roll.data  = int(converse(self.joints_val[1][2])*conv_const)
        self.joint_current_val_msg.l_hip_pitch.data = int(converse(self.joints_val[2][2])*conv_const)
        self.joint_current_val_msg.l_knee.data      = int(converse(self.joints_val[3][2])*conv_const)
        self.joint_current_val_msg.l_ank_pitch.data = int(converse(self.joints_val[4][2])*conv_const)
        self.joint_current_val_msg.l_ank_roll.data  = int(converse(self.joints_val[5][2])*conv_const)
        self.joint_current_val_msg.r_hip_yaw.data   = int(converse(self.joints_val[6][2])*conv_const)
        self.joint_current_val_msg.r_hip_roll.data  = int(converse(self.joints_val[7][2])*conv_const)
        self.joint_current_val_msg.r_hip_pitch.data = int(converse(self.joints_val[8][2])*conv_const)
        self.joint_current_val_msg.r_knee.data      = int(converse(self.joints_val[9][2])*conv_const)
        self.joint_current_val_msg.r_ank_pitch.data = int(converse(self.joints_val[10][2])*conv_const)
        self.joint_current_val_msg.r_ank_roll.data  = int(converse(self.joints_val[11][2])*conv_const)
        self.joint_current_val_msg.l_sho_pitch.data = int(converse(self.joints_val[12][2])*conv_const)
        self.joint_current_val_msg.l_sho_roll.data  = int(converse(self.joints_val[13][2])*conv_const)
        self.joint_current_val_msg.l_el.data        = int(converse(self.joints_val[14][2])*conv_const)
        self.joint_current_val_msg.r_sho_pitch.data = int(converse(self.joints_val[15][2])*conv_const)
        self.joint_current_val_msg.r_sho_roll.data  = int(converse(self.joints_val[16][2])*conv_const)
        self.joint_current_val_msg.r_el.data        = int(converse(self.joints_val[17][2])*conv_const)
        self.joint_current_val_msg.head_pan.data    = int(converse(self.joints_val[18][2])*conv_const)
        self.joint_current_val_msg.head_tilt.data   = int(converse(self.joints_val[19][2])*conv_const)
        self.joint_current_val_pub.publish(self.joint_current_val_msg)



    def readWrite(self) -> None:
        self.readJointValue()
        self.writeJointValue()



    def jointTargetValCallback(self, data) -> None:
        if (
            self.joints_dxl_val[0]  != data.l_hip_yaw.data or
            self.joints_dxl_val[1]  != data.l_hip_roll.data or
            self.joints_dxl_val[2]  != data.l_hip_pitch.data or
            self.joints_dxl_val[3]  != data.l_knee.data or
            self.joints_dxl_val[4]  != data.l_ank_pitch.data or
            self.joints_dxl_val[5]  != data.l_ank_roll.data or
            self.joints_dxl_val[6]  != data.r_hip_yaw.data or
            self.joints_dxl_val[7]  != data.r_hip_roll.data or
            self.joints_dxl_val[8]  != data.r_hip_pitch.data or
            self.joints_dxl_val[9]  != data.r_knee.data or
            self.joints_dxl_val[10] != data.r_ank_pitch.data or
            self.joints_dxl_val[11] != data.r_ank_roll.data or
            self.joints_dxl_val[12] != data.l_sho_pitch.data or
            self.joints_dxl_val[13] != data.l_sho_roll.data or
            self.joints_dxl_val[14] != data.l_el.data or
            self.joints_dxl_val[15] != data.r_sho_pitch.data or
            self.joints_dxl_val[16] != data.r_sho_roll.data or
            self.joints_dxl_val[17] != data.r_el.data or
            self.joints_dxl_val[18] != data.head_pan.data or
            self.joints_dxl_val[19] != data.head_tilt.data
        ):
            self.update_pos         = True
            self.joints_dxl_val[0]  = data.l_hip_yaw.data
            self.joints_dxl_val[1]  = data.l_hip_roll.data
            self.joints_dxl_val[2]  = data.l_hip_pitch.data
            self.joints_dxl_val[3]  = data.l_knee.data
            self.joints_dxl_val[4]  = data.l_ank_pitch.data
            self.joints_dxl_val[5]  = data.l_ank_roll.data
            self.joints_dxl_val[6]  = data.r_hip_yaw.data
            self.joints_dxl_val[7]  = data.r_hip_roll.data
            self.joints_dxl_val[8]  = data.r_hip_pitch.data
            self.joints_dxl_val[9]  = data.r_knee.data
            self.joints_dxl_val[10] = data.r_ank_pitch.data
            self.joints_dxl_val[11] = data.r_ank_roll.data
            self.joints_dxl_val[12] = data.l_sho_pitch.data
            self.joints_dxl_val[13] = data.l_sho_roll.data
            self.joints_dxl_val[14] = data.l_el.data
            self.joints_dxl_val[15] = data.r_sho_pitch.data
            self.joints_dxl_val[16] = data.r_sho_roll.data
            self.joints_dxl_val[17] = data.r_el.data
            self.joints_dxl_val[18] = data.head_pan.data
            self.joints_dxl_val[19] = data.head_tilt.data


        if (
            self.joints_val[0][1]   != data.l_hip_yaw_t.data or
            self.joints_val[1][1]   != data.l_hip_roll_t.data or
            self.joints_val[2][1]   != data.l_hip_pitch_t.data or
            self.joints_val[3][1]   != data.l_knee_t.data or
            self.joints_val[4][1]   != data.l_ank_pitch_t.data or
            self.joints_val[5][1]   != data.l_ank_roll_t.data or
            self.joints_val[6][1]   != data.r_hip_yaw_t.data or
            self.joints_val[7][1]   != data.r_hip_roll_t.data or
            self.joints_val[8][1]   != data.r_hip_pitch_t.data or
            self.joints_val[9][1]   != data.r_knee_t.data or
            self.joints_val[10][1]  != data.r_ank_pitch_t.data or
            self.joints_val[11][1]  != data.r_ank_roll_t.data or
            self.joints_val[12][1]  != data.l_sho_pitch_t.data or
            self.joints_val[13][1]  != data.l_sho_roll_t.data or
            self.joints_val[14][1]  != data.l_el_t.data or
            self.joints_val[15][1]  != data.r_sho_pitch_t.data or
            self.joints_val[16][1]  != data.r_sho_roll_t.data or
            self.joints_val[17][1]  != data.r_el_t.data or
            self.joints_val[18][1]  != data.head_pan_t.data or
            self.joints_val[19][1]  != data.head_tilt_t.data
        ):
            self.update_vel         = True
            self.joints_val[0][1]   = data.l_hip_yaw_t.data
            self.joints_val[1][1]   = data.l_hip_roll_t.data
            self.joints_val[2][1]   = data.l_hip_pitch_t.data
            self.joints_val[3][1]   = data.l_knee_t.data
            self.joints_val[4][1]   = data.l_ank_pitch_t.data
            self.joints_val[5][1]   = data.l_ank_roll_t.data
            self.joints_val[6][1]   = data.r_hip_yaw_t.data
            self.joints_val[7][1]   = data.r_hip_roll_t.data
            self.joints_val[8][1]   = data.r_hip_pitch_t.data
            self.joints_val[9][1]   = data.r_knee_t.data
            self.joints_val[10][1]  = data.r_ank_pitch_t.data
            self.joints_val[11][1]  = data.r_ank_roll_t.data
            self.joints_val[12][1]  = data.l_sho_pitch_t.data
            self.joints_val[13][1]  = data.l_sho_roll_t.data
            self.joints_val[14][1]  = data.l_el_t.data
            self.joints_val[15][1]  = data.r_sho_pitch_t.data
            self.joints_val[16][1]  = data.r_sho_roll_t.data
            self.joints_val[17][1]  = data.r_el_t.data
            self.joints_val[18][1]  = data.head_pan_t.data
            self.joints_val[19][1]  = data.head_tilt_t.data


        if self.update_pos or self.update_vel:
            
            for joint_idx in range(self.num_joints):
                self.joints_inc[joint_idx] = (self.dxl2Radian(self.joints_dxl_val[joint_idx]) - self.joints_val[joint_idx][0])*self.STEP_TIME*1000/self.joints_val[joint_idx][1]
            
            self.update_pos = False
            self.update_vel = False


        
    def dxl2Radian(self, dxl_val) -> float:
        conv_const = 0.001534355386
        if dxl_val < 2047:
            return dxl_val*conv_const
        else:
            return (dxl_val-4096)*conv_const