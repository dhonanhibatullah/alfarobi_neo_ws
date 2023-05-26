/* Libraries */
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <alfarobi_lib/Eigen/Dense>




/* Inside alfarobi namespace */
namespace alfarobi{



    /* Pass the value below */
    enum BodyPose {
        OFFSET,
        WALKING,
        KICKING
    };



    /* setBodyPose() function
        * Use this function to set the body pose of the robot
        * the input is based on the enumeration given
    */    
    void setBodyPose(BodyPose body_pose);
}