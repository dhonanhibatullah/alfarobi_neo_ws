#include <alfarobi_controller/kinematics/PosePlayer.h>




alfarobi::PosePlayer::PosePlayer(ros::NodeHandle* nh, std::string name) {
    this->nh    = nh;
    this->name  = name;
    ROS_WARN("PosePlayer started!");

    /* Publishers and subscribers definiton */
    joint_target_val_pub = this->nh->advertise<alfarobi_lib::JointTargetVal>(this->name + (std::string)"/joint_target_val", 1000);
}




alfarobi::PosePlayer::~PosePlayer() {
    ROS_WARN("PosePlayer terminated!");
}




void alfarobi::PosePlayer::fetchBodyPoseFromYaml(YAML::Node config) {
    joint_target_val_msg.l_hip_yaw.data     = config["l_hip_yaw"].as<uint32_t>();
    joint_target_val_msg.l_hip_roll.data    = config["l_hip_roll"].as<uint32_t>();
    joint_target_val_msg.l_hip_pitch.data   = config["l_hip_pitch"].as<uint32_t>();
    joint_target_val_msg.l_knee.data        = config["l_knee"].as<uint32_t>();
    joint_target_val_msg.l_ank_pitch.data   = config["l_ank_pitch"].as<uint32_t>();
    joint_target_val_msg.l_ank_roll.data    = config["l_ank_roll"].as<uint32_t>();
    joint_target_val_msg.r_hip_yaw.data     = config["r_hip_yaw"].as<uint32_t>();
    joint_target_val_msg.r_hip_roll.data    = config["r_hip_roll"].as<uint32_t>();
    joint_target_val_msg.r_hip_pitch.data   = config["r_hip_pitch"].as<uint32_t>();
    joint_target_val_msg.r_knee.data        = config["r_knee"].as<uint32_t>();
    joint_target_val_msg.r_ank_pitch.data   = config["r_ank_pitch"].as<uint32_t>();
    joint_target_val_msg.r_ank_roll.data    = config["r_ank_roll"].as<uint32_t>();
    joint_target_val_msg.l_sho_pitch.data   = config["l_sho_pitch"].as<uint32_t>();
    joint_target_val_msg.l_sho_roll.data    = config["l_sho_roll"].as<uint32_t>();
    joint_target_val_msg.l_el.data          = config["l_el"].as<uint32_t>();
    joint_target_val_msg.r_sho_pitch.data   = config["r_sho_pitch"].as<uint32_t>();
    joint_target_val_msg.r_sho_roll.data    = config["r_sho_roll"].as<uint32_t>();
    joint_target_val_msg.r_el.data          = config["r_el"].as<uint32_t>();
    joint_target_val_msg.head_pan.data      = config["head_pan"].as<uint32_t>();
    joint_target_val_msg.head_tilt.data     = config["head_tilt"].as<uint32_t>();
    joint_target_val_msg.l_hip_yaw_t.data   = body_pose_t;
    joint_target_val_msg.l_hip_roll_t.data  = body_pose_t;
    joint_target_val_msg.l_hip_pitch_t.data = body_pose_t;
    joint_target_val_msg.l_knee_t.data      = body_pose_t;
    joint_target_val_msg.l_ank_pitch_t.data = body_pose_t;
    joint_target_val_msg.l_ank_roll_t.data  = body_pose_t;
    joint_target_val_msg.r_hip_yaw_t.data   = body_pose_t;
    joint_target_val_msg.r_hip_roll_t.data  = body_pose_t;
    joint_target_val_msg.r_hip_pitch_t.data = body_pose_t;
    joint_target_val_msg.r_knee_t.data      = body_pose_t;
    joint_target_val_msg.r_ank_pitch_t.data = body_pose_t;
    joint_target_val_msg.r_ank_roll_t.data  = body_pose_t;
    joint_target_val_msg.l_sho_pitch_t.data = body_pose_t;
    joint_target_val_msg.l_sho_roll_t.data  = body_pose_t;
    joint_target_val_msg.l_el_t.data        = body_pose_t;
    joint_target_val_msg.r_sho_pitch_t.data = body_pose_t;
    joint_target_val_msg.r_sho_roll_t.data  = body_pose_t;
    joint_target_val_msg.r_el_t.data        = body_pose_t;
    joint_target_val_msg.head_pan_t.data    = body_pose_t;
    joint_target_val_msg.head_tilt_t.data   = body_pose_t;
}




bool alfarobi::PosePlayer::setBodyPose(PosePlayer::BodyPoseEnum body_pose) {
    /* Timer for returning value */
    static uint64_t timer_start = 0;
    static bool     is_init     = true;
    

    /* Send the joint values based on .yaml file */
    switch(body_pose) {

        case alfarobi::PosePlayer::OFFSET:
            fetchBodyPoseFromYaml(body_pose_config["offset"]);
            if(is_init) ROS_INFO("PosePlayer plays 'offset' pose!");
            break;


        case alfarobi::PosePlayer::WALKING:
            fetchBodyPoseFromYaml(body_pose_config["walking"]);
            if(is_init) ROS_INFO("PosePlayer plays 'walking' pose!");
            break;


        case alfarobi::PosePlayer::KICKING:
            fetchBodyPoseFromYaml(body_pose_config["kicking"]);
            if(is_init) ROS_INFO("PosePlayer plays 'kicking' pose!");
            break;
    }


    if(is_init) {
        timer_start = ros::Time::now().toSec()*1000;
        is_init     = false;
    }


    if(ros::Time::now().toSec()*1000 - timer_start > body_pose_t + COMMUNICATION_TIME_ERROR) {
        is_init = true;
        return true;
    }
    else {
        joint_target_val_pub.publish(joint_target_val_msg);
        return false;
    }
}