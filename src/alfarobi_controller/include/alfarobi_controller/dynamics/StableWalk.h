/* StableWalk.h
    * Created by Dhonan Nabil Hibatullah, Alfarobi v12
    * dhonan.hibatullah@gmail.com, open for any questions
*/    

#ifndef STABLE_WALK_H
#define STABLE_WALK_H

/* Definitions */
#define NEGLIGIBLE_VAL      0.01
#define _USE_MATH_DEFINES

/* Libraries */
#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

/* Alfarobi libraries */
#include <alfarobi_lib/Eigen/Dense>
#include <alfarobi_lib/Math/LinearAlgebra.h>
#include <alfarobi_lib/Math/Plotter.h>




/* Inside alfarobi namespace */
namespace alfarobi{




    class StableWalkController {
    
    
    
    private:

        ros::NodeHandle* nh;

        /* pkg_path is holding this package directory */
        std::string pkg_path        = ros::package::getPath("alfarobi_controller"),
                    name;


        /* parse the needed .yaml files */
        YAML::Node  control_param   = YAML::LoadFile(pkg_path + (std::string)"/config/stable_walk_param.yaml");


        /* Behavioral parameters
            * The following variables will determine the behaviour of the stable walk. You may change
            * these values from config/stable_walk_param.yaml.
            *   sampling_freq   : controller sampling frequency
            *   preview_step    : number of step being previewed
            *   swing_damp      : damping const. between [0, 1] for reducing the hip swing while walking
            *   foot_width      : distance between the center of two foot (in m)
            *   step_dist_x     : distance between step in x-direction
            *   step_dist_y     : distance between step in y-direction
            *   com_height      : called z_c in the formula. This will change the way how the robot stands
            *   Q_e, Q_x, and R : weight for the optimalization objective function
        */
        double      swing_damp,
                    foot_width,
                    step_height,
                    com_height,
                    max_step_dist_x,
                    nor_step_dist_x,
                    max_step_dist_y,
                    nor_step_dist_y,
                    max_omega,
                    max_step_freq,
                    nor_step_freq,
                    Q_ex, Q_xx, R_x,
                    Q_ey, Q_xy, R_y,
                    sampling_t;
        
        uint64_t    sampling_freq,
                    preview_step,
                    time_t = 0;


        /* Input variables
            * vel_x, vel_y, and omega are used for input
            * this means, the robot can accept input for once and latch the input
            * The robot will remain walking as long as is_walking is True
            * walking_mode set the walking mode to velocity-based or spline-based
        */
        double  vel_x,
                vel_y,
                omega;

        bool    is_walking;

        enum    WalkingModeEnum {
                    NONE,
                    VELOCITY,
                    SPLINE,
                    STOPPING
                };

        WalkingModeEnum walking_mode = NONE;


        /* Control states
            * x_st, x_dot_st, y_st, and y_dot_st are the control state variables
            * com_pos is the position of CoM with respect to robot's starting position
            * foot_pos is the position of lifting foot
            * l_joint_val consists of left hip, knee, and ankle joints' value in radian and so r_joint_val
            * foot_placed will tell which foot currently on the ground
        */
        Eigen::VectorXd x_st        = Eigen::VectorXd(3),
                        x_dot_st    = Eigen::VectorXd(3),
                        y_st        = Eigen::VectorXd(3),
                        y_dot_st    = Eigen::VectorXd(3);

        enum    PlacedFootEnum {
                    BOTH_FEET,
                    LEFT_FOOT,
                    RIGHT_FOOT
                };

        PlacedFootEnum placed_foot = BOTH_FEET;


        /* Output values
            * zmp and com trajectories are the output of this controller
            * this output can be turned into robot joint values via inverse kinematics
        */
        std::vector<std::vector<double>>    zmp_trajectory,
                                            com_trajectory;


        /* Controller variables
            * The following variables is used to control the stable walk
            * and not meant to be edited or changed, but one may improve it
        */
        // Eigen::MatrixXd A_mat       = Eigen::MatrixXd(3, 3),
        //                 B_mat       = Eigen::MatrixXd(3, 1),
        //                 C_mat       = Eigen::MatrixXd(1, 3),
        //                 Ad_mat      = Eigen::MatrixXd(3, 3),
        //                 Bd_mat      = Eigen::MatrixXd(3, 1),
        //                 Cd_mat      = Eigen::MatrixXd(1, 3),
        //                 B_tilde     = Eigen::MatrixXd(4, 1),
        //                 I_tilde     = Eigen::MatrixXd(4, 1),
        //                 F_tilde     = Eigen::MatrixXd(4, 3),
        //                 A_tilde     = Eigen::MatrixXd(4, 4),
        //                 Qx_tilde    = Eigen::MatrixXd(4, 4),
        //                 Qy_tilde    = Eigen::MatrixXd(4, 4),
        //                 Rx          = Eigen::MatrixXd(1, 1),
        //                 Ry          = Eigen::MatrixXd(1, 1),
        //                 Kx_tilde    = Eigen::MatrixXd(4, 4),
        //                 Ky_tilde    = Eigen::MatrixXd(4, 4),
        //                 Ax_c_tilde  = Eigen::MatrixXd(4, 4),
        //                 Ay_c_tilde  = Eigen::MatrixXd(4, 4);

        double          x_err       = 0.,
                        y_err       = 0.,
                        Gx_i        = 0.,
                        Gx_x        = 0.,
                        Gy_i        = 0.,
                        Gy_x        = 0.;

        std::vector<double> Gx_d,
                            Gy_d;



        /* Holder variables */
        double      zmp_step_period,
                    zmp_step_freq,
                    zmp_step_dist_x,
                    zmp_step_dist_y,
                    zmp_omega;


        /* ROS publishers, subscribers, and messages */



    protected:

        void generateZmpFromVel();
        void generateZmpFromSpline();



    public:

        StableWalkController(ros::NodeHandle* nh, std::string name);
        ~StableWalkController();

        void setVelocityStableWalk(double vel_x, double vel_y, double omega);
        void setSplineStableWalk();
        void playStableWalk();
        void stopStableWalk();

        void plot(std::vector<double>& nval, std::vector<double>& xval, std::vector<double>& yval);
        uint64_t stoppppphhh();
    };
}


#endif