#include <alfarobi_controller/dynamics/StableWalk.h>




alfarobi::StableWalkController::StableWalkController(ros::NodeHandle* nh, std::string name) {
    this->nh    = nh;
    this->name  = name;
    ROS_WARN("StableWalkController started!");

    /* Parse values from config/control_param.yaml */
    sampling_freq   = control_param["sampling_freq"].as<uint64_t>();
    preview_step    = control_param["preview_step"].as<uint64_t>();
    swing_damp      = control_param["swing_damp"].as<double>();
    foot_width      = control_param["foot_width"].as<double>();
    step_height     = control_param["step_height"].as<double>();
    com_height      = control_param["com_height"].as<double>();
    max_step_dist_x = control_param["max_step_dist_x"].as<double>();
    nor_step_dist_x = control_param["nor_step_dist_x"].as<double>();
    max_step_dist_y = control_param["max_step_dist_y"].as<double>();
    nor_step_dist_y = control_param["nor_step_dist_y"].as<double>();
    max_omega       = control_param["max_omega"].as<double>();
    max_step_freq   = control_param["max_step_freq"].as<double>();
    nor_step_freq   = control_param["nor_step_freq"].as<double>();
    Q_ex            = control_param["Q_ex"].as<double>();
    Q_xx            = control_param["Q_xx"].as<double>();
    R_x             = control_param["R_x"].as<double>();
    Q_ey            = control_param["Q_ey"].as<double>();
    Q_xy            = control_param["Q_xy"].as<double>();
    R_y             = control_param["R_y"].as<double>();
    sampling_t      = 1./((double)sampling_freq);

    /* The following values are remain fixed and not meant to be changed */
    ROS_INFO("Evaluating parameters for StableWalkController...");

    /* Continuous-time matrices */
    // A_mat <<    0, 1, 0,
    //             0, 0, 1,
    //             0, 0, 0;

    // B_mat <<    0,
    //             0,
    //             1;

    // C_mat <<    1, 0, com_height;

    // /* Discrete-time matrices */
    // Ad_mat <<   1, sampling_t, pow(sampling_t, 2.),
    //             0, 1, sampling_t,
    //             0, 0, 1;

    // Bd_mat <<   pow(sampling_t, 3.)/6., 
    //             pow(sampling_t, 2.)/2.,
    //             sampling_t;


    // Cd_mat <<   1, 0, com_height;

    // /* Preview control values */
    // B_tilde     <<  Cd_mat*Bd_mat,
    //                 Bd_mat;

    // I_tilde     <<  1,
    //                 0,
    //                 0,
    //                 0;

    // F_tilde     <<  Cd_mat*Ad_mat,
    //                 Ad_mat;

    // Qx_tilde    <<  Q_ex*Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 2),
    //                 Eigen::MatrixXd::Zero(2, 2), Q_xx*Eigen::MatrixXd::Identity(2, 2);

    // Qy_tilde    <<  Q_ey*Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 2),
    //                 Eigen::MatrixXd::Zero(2, 2), Q_xy*Eigen::MatrixXd::Identity(2, 2);

    // Rx          <<  R_x;

    // Ry          <<  R_y;

    // alfarobi::solveDare(Kx_tilde, A_tilde, B_tilde, Rx, Qx_tilde);

    // alfarobi::solveDare(Ky_tilde, A_tilde, B_tilde, Ry, Qy_tilde);

    // Gx_i        = ((Rx + B_tilde.transpose()*Kx_tilde*B_tilde).inverse()*B_tilde.transpose()*Kx_tilde*I_tilde)(0, 0);

    // Gx_x        = ((Rx + B_tilde.transpose()*Kx_tilde*B_tilde).inverse()*B_tilde.transpose()*Kx_tilde*F_tilde)(0, 0);

    // Gy_i        = ((Ry + B_tilde.transpose()*Ky_tilde*B_tilde).inverse()*B_tilde.transpose()*Ky_tilde*I_tilde)(0, 0);

    // Gy_x        = ((Ry + B_tilde.transpose()*Ky_tilde*B_tilde).inverse()*B_tilde.transpose()*Ky_tilde*F_tilde)(0, 0);

    ROS_INFO("Evaluating parameters has been completed successfully!");
}




alfarobi::StableWalkController::~StableWalkController() {
    ROS_WARN("StableWalkController terminated!");
}




void alfarobi::StableWalkController::generateZmpFromVel() {
    /* Initialization */
    if(zmp_trajectory.size() == 0) {

        /* Generate first ZMP previewed step */
        for(uint16_t n = 0; n < preview_step; ++n) {
            std::vector<double> point;
            bool step_inc = fmod(((double)n)*sampling_t, zmp_step_period) < sampling_t;
            

            /* x-direction */
            if(n == 0) point.push_back(0.);
            else point.push_back(
                step_inc ? 
                zmp_trajectory.back()[0] + zmp_step_dist_x :
                zmp_trajectory.back()[0]
            );


            /* y-direction */
            if(n == 0) point.push_back(foot_width/2.);
            else {
                if(sin(((double)n)*sampling_t*M_PI*zmp_step_freq) > 0.) {
                    placed_foot = alfarobi::StableWalkController::LEFT_FOOT;
                    point.push_back(
                        step_inc ? 
                        zmp_trajectory.back()[1] + foot_width/2. + zmp_step_dist_y :
                        zmp_trajectory.back()[1]
                    );
                }
                else {
                    placed_foot = alfarobi::StableWalkController::RIGHT_FOOT;
                    point.push_back(
                        step_inc ? 
                        zmp_trajectory.back()[1] - foot_width/2. + zmp_step_dist_y :
                        zmp_trajectory.back()[1]
                    );
                }
            }

            zmp_trajectory.push_back(point);
        }
    }

    /* If not init:
        * Remove the first zmp_trajectory value
        * Add a new one
    */
    else {

        zmp_trajectory.erase(zmp_trajectory.begin());
        std::vector<double> point;
        bool step_inc = fmod(((double)(time_t + preview_step))*sampling_t, zmp_step_period) < sampling_t;


        /* x-direction */
        point.push_back(
            step_inc ? 
            zmp_trajectory.back()[0] + zmp_step_dist_x :
            zmp_trajectory.back()[0]
        );


        /* y-direction */
        if(sin(((double)(time_t + preview_step))*sampling_t*M_PI*zmp_step_freq) > 0.) {
            placed_foot = alfarobi::StableWalkController::LEFT_FOOT;
            point.push_back(
                step_inc ? 
                zmp_trajectory.back()[1] + foot_width/2. + zmp_step_dist_y :
                zmp_trajectory.back()[1]
            );
        }
        else {
            placed_foot = alfarobi::StableWalkController::RIGHT_FOOT;
            point.push_back(
                step_inc ? 
                zmp_trajectory.back()[1] - foot_width/2. + zmp_step_dist_y :
                zmp_trajectory.back()[1]
            );
        }

        zmp_trajectory.push_back(point);
    }
}




void alfarobi::StableWalkController::generateZmpFromSpline() {

}




void alfarobi::StableWalkController::setVelocityStableWalk(double vel_x, double vel_y, double omega) {
    /* Set walking mode to VELOCITY*/
    walking_mode = alfarobi::StableWalkController::VELOCITY;


    /* Calculate step frequency and period */
    zmp_step_freq = (pow(vel_x, 2) + pow(vel_y, 2))/(fabs(vel_x*nor_step_dist_x) + fabs(vel_y*nor_step_dist_y));
    if(zmp_step_freq > max_step_freq) {
        zmp_step_freq = max_step_freq;
        ROS_ERROR("Maximum step frequency exceeded, set to maximum!");
    }
    zmp_step_period = 1./zmp_step_freq;


    /* Calculate distance x and y*/
    zmp_step_dist_x = vel_x*zmp_step_period;
    if(fabs(zmp_step_dist_x) > max_step_dist_x) {
        zmp_step_dist_x = (zmp_step_dist_x > 0) ? max_step_dist_x : -max_step_dist_x;
        ROS_ERROR("Maximum step distance x exceeded, set to maximum!");
    }

    zmp_step_dist_y = vel_y*zmp_step_period;
    if(fabs(zmp_step_dist_y) > max_step_dist_y) {
        zmp_step_dist_y = (zmp_step_dist_y > 0) ? max_step_dist_y : -max_step_dist_y;
        ROS_ERROR("Maximum step distance y exceeded, set to maximum!");
    }

    zmp_omega = omega;
    if(fabs(zmp_omega) > max_omega) {
        zmp_omega = (zmp_omega > 0) ? max_omega : -max_omega;
        ROS_ERROR("Maximum omega exceeded, set to maximum!");
    }


    /* Save the value */
    this->vel_x     = zmp_step_dist_x*zmp_step_freq;
    this->vel_y     = zmp_step_dist_y*zmp_step_freq;
    this->omega     = zmp_omega;

    ROS_INFO("zmp_step_dist_x = %f", zmp_step_dist_x);
    ROS_INFO("zmp_step_dist_y = %f", zmp_step_dist_y);
    ROS_INFO("zmp_step_period = %f", zmp_step_period);
    ROS_INFO("sampling_time   = %f", sampling_t);
}




void alfarobi::StableWalkController::setSplineStableWalk() {
    /* Set walking mode to SPLINE*/
    walking_mode = alfarobi::StableWalkController::SPLINE;

}




void alfarobi::StableWalkController::playStableWalk() {
    /* Set is_walking to true to start moving */
    is_walking = true;


    /* Move based on walking_mode */
    switch(walking_mode) {

        case alfarobi::StableWalkController::VELOCITY:
            generateZmpFromVel();
            break;


        case alfarobi::StableWalkController::SPLINE:
            generateZmpFromSpline();
            break;

        
        case alfarobi::StableWalkController::STOPPING:
            break;
    }


    /* Increment time*/
    ++time_t;
}




void alfarobi::StableWalkController::stopStableWalk() {
    is_walking      = false;
    walking_mode    = alfarobi::StableWalkController::STOPPING; 
}




void alfarobi::StableWalkController::plot(std::vector<double>& nval, std::vector<double>& xval, std::vector<double>& yval) {
    for(uint16_t i = 0; i < zmp_trajectory.size(); ++i) {
        nval.push_back((double)i);
        xval.push_back(zmp_trajectory[i][0]);
        yval.push_back(zmp_trajectory[i][1]);
    }
}


uint64_t alfarobi::StableWalkController::stoppppphhh() {
    return time_t;
}