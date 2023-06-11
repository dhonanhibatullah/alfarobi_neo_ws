/* PosePlayer.h
    * Created by Dhonan Nabil Hibatullah, Alfarobi v12
    * dhonan.hibatullah@gmail.com, open for any questions
*/    

#ifndef POSE_PLAYER_H
#define POSE_PLAYER_H

/* Libraries */
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <alfarobi_lib/Eigen/Dense>
#include <alfarobi_lib/JointTargetVal.h>

#define COMMUNICATION_TIME_ERROR    50


/* Inside alfarobi namespace */
namespace alfarobi{



    /* PosePlayer */
    class PosePlayer {



    private:

        ros::NodeHandle*    nh;

        std::string         pkg_path            = ros::package::getPath("alfarobi_controller"),
                            name;

        YAML::Node          body_pose_config    = YAML::LoadFile(pkg_path + (std::string)"/config/pose_body.yaml"),
                            seq_move_config     = YAML::LoadFile(pkg_path + (std::string)"/config/pose_sequential.yaml");

        /* Change body_pose_t to set body posing time in miliseconds */
        uint32_t            body_pose_t         = body_pose_config["body_pose_time"].as<uint32_t>();

        /* ROS msgs, publishers, and subscribers */
        alfarobi_lib::JointTargetVal    joint_target_val_msg;

        ros::Publisher                  joint_target_val_pub;



    protected:

        void fetchBodyPoseFromYaml(YAML::Node config);



    public:

        enum BodyPoseEnum {
            OFFSET,
            WALKING,
            KICKING
        };

        PosePlayer(ros::NodeHandle* nh, std::string name);
        ~PosePlayer();

        bool setBodyPose(BodyPoseEnum body_pose);
    };
}


#endif