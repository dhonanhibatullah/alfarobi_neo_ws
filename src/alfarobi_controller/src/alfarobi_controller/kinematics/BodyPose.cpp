#include <alfarobi_controller/kinematics/BodyPose.h>




void alfarobi::setBodyPose(BodyPose body_pose) {
    /* Get the package directory */
    std::string pkg_path = ros::package::getPath("alfarobi_controller");
    YAML::Node body_pose_config = YAML::LoadFile(pkg_path + (std::string)"/config/initial_pose.yaml");


    /* Send the joint values based on .yaml file */
    switch(body_pose) {
        case OFFSET:
            break;

        case WALKING:
            break;

        case KICKING:
            break;
    }


    
}