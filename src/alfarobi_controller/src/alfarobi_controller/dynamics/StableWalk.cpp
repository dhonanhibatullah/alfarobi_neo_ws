#include <alfarobi_controller/dynamics/StableWalk.h>




alfarobi::StableWalkController::StableWalkController(std::string name) {
    ROS_WARN("StableWalkController started!");

    /* This name variable is used to define the robot name for environment developing */
    this->name  = name;

    /* Parse values from config/control_param.yaml */
    sampling_freq   = control_param["sampling_freq"].as<uint64_t>();
    preview_step    = control_param["preview_step"].as<uint64_t>();
    swing_damp      = control_param["swing_damp"].as<double>();
    foot_width      = control_param["foot_width"].as<double>();
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
    A_mat <<    0, 1, 0,
                0, 0, 1,
                0, 0, 0;

    B_mat <<    0,
                0,
                1;

    C_mat <<    1, 0, com_height;

    /* Discrete-time matrices */
    Ad_mat <<   1, sampling_t, pow(sampling_t, 2.),
                0, 1, sampling_t,
                0, 0, 1;

    Bd_mat <<   pow(sampling_t, 3.)/6., 
                pow(sampling_t, 2.)/2.,
                sampling_t;


    Cd_mat <<   1, 0, com_height;

    /* Preview control values */
    B_tilde     <<  Cd_mat*Bd_mat,
                    Bd_mat;

    I_tilde     <<  1,
                    0,
                    0,
                    0;

    F_tilde     <<  Cd_mat*Ad_mat,
                    Ad_mat;

    Qx_tilde    <<  Q_ex*Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 2),
                    Eigen::MatrixXd::Zero(2, 2), Q_xx*Eigen::MatrixXd::Identity(2, 2);

    Qy_tilde    <<  Q_ey*Eigen::MatrixXd::Identity(2, 2), Eigen::MatrixXd::Zero(2, 2),
                    Eigen::MatrixXd::Zero(2, 2), Q_xy*Eigen::MatrixXd::Identity(2, 2);

    Rx          <<  R_x;

    Ry          <<  R_y;

    alfarobi::solveDare(Kx_tilde, A_tilde, B_tilde, Rx, Qx_tilde);

    alfarobi::solveDare(Ky_tilde, A_tilde, B_tilde, Ry, Qy_tilde);

    Gx_i        = ((Rx + B_tilde.transpose()*Kx_tilde*B_tilde).inverse()*B_tilde.transpose()*Kx_tilde*I_tilde)(0, 0);

    Gx_x        = ((Rx + B_tilde.transpose()*Kx_tilde*B_tilde).inverse()*B_tilde.transpose()*Kx_tilde*F_tilde)(0, 0);

    Gy_i        = ((Ry + B_tilde.transpose()*Ky_tilde*B_tilde).inverse()*B_tilde.transpose()*Ky_tilde*I_tilde)(0, 0);

    Gy_x        = ((Ry + B_tilde.transpose()*Ky_tilde*B_tilde).inverse()*B_tilde.transpose()*Ky_tilde*F_tilde)(0, 0);

    ROS_INFO("Evaluating parameters has been completed successfully!");
}




alfarobi::StableWalkController::~StableWalkController() {
    ROS_WARN("StableWalkController terminated!");
}




void alfarobi::StableWalkController::generateZmpFromVel(bool is_init=false) {
    if(is_init) {
        /* Clear the zmp_trajectory */
        zmp_trajectory.clear();

        /* Generate first ZMP previewed step */
        for(uint16_t n = 0; n < preview_step; ++n) {
            std::vector<double> point;
            
            /* x-direction */
            if(n == 0) point.push_back(0.);
            else point.push_back(
                (fmod(((double)time_t)*sampling_t, sampling_t) > sampling_t) ? 
                point.back() + zmp_step_dist_x :
                point.back()
            );

            /* y-direction */
            
        }
    }


    else if(is_walking) {

    }


    else if(!is_walking) {

    }
}




void alfarobi::StableWalkController::generateZmpFromSpline() {

}




void alfarobi::StableWalkController::velocityStableWalk(double vel_x, double vel_y, double omega) {
    /* Initialization for calculating optimal step distance and period */
    static bool is_init = true;     // Declared only once

    if(is_init) {
        /* Calculate step frequency and period */
        double zmp_step_freq = (pow(vel_x, 2) + pow(vel_y, 2))/(vel_x*nor_step_dist_x + vel_y*nor_step_dist_y);
        if(zmp_step_freq > max_step_freq) {
            zmp_step_freq = max_step_freq;
            ROS_ERROR("Maximum step frequency exceeded, set to maximum!");
        }
        zmp_step_period = 1./zmp_step_freq;

        /* Calculate distance x and y*/
        zmp_step_dist_x = vel_x*zmp_step_period;
        if(zmp_step_dist_x > max_step_dist_x) {
            zmp_step_dist_x = max_step_dist_x;
            ROS_ERROR("Maximum step distance x exceeded, set to maximum!");
        }

        zmp_step_dist_y = vel_y*zmp_step_period;
        if(zmp_step_dist_y > max_step_dist_y) {
            zmp_step_dist_y = max_step_dist_y;
            ROS_ERROR("Maximum step distance y exceeded, set to maximum!");
        }

        zmp_omega = omega;
        if(zmp_omega > max_omega) {
            zmp_omega = max_omega;
            ROS_ERROR("Maximum omega exceeded, set to maximum!");
        }

        /* Save the value */
        this->vel_x     = zmp_step_dist_x*zmp_step_freq;
        this->vel_y     = zmp_step_dist_y*zmp_step_freq;
        this->omega     = zmp_omega;
    }



    /* Start the walk if start_walk == true */
    if(is_walking) {


        /* Walk in place */
        if(fabs(vel_x) <= NEGLIGIBLE_VAL and fabs(vel_y) <= NEGLIGIBLE_VAL and fabs(omega) <= NEGLIGIBLE_VAL) {
            generateZmpFromVel(is_init);
        }


        /* Rotate */
        else if(fabs(vel_x) <= NEGLIGIBLE_VAL and fabs(vel_y) <= NEGLIGIBLE_VAL and fabs(omega) > NEGLIGIBLE_VAL) {
            generateZmpFromVel(is_init);
        }


        /* Move straight */
        else if((fabs(vel_x) > NEGLIGIBLE_VAL or fabs(vel_y) > NEGLIGIBLE_VAL) and fabs(omega) <= NEGLIGIBLE_VAL) {
            generateZmpFromVel(is_init);
        }


        /* Revolute */
        else if((fabs(vel_x) > NEGLIGIBLE_VAL or fabs(vel_y) > NEGLIGIBLE_VAL) and fabs(omega) > NEGLIGIBLE_VAL) {
            generateZmpFromVel(is_init);
        }


        /* Stop initializing */
        is_init = false;
    }


    else {
        /* Reset */
        this->vel_x = 0.;
        this->vel_y = 0.;
        this->omega = 0.;
        is_init = true;

        /* Stop the movement */
        generateZmpFromVel();
    }


    /* Increment time */
    ++time_t;
}




void alfarobi::StableWalkController::splineStableWalk() {

}




void alfarobi::StableWalkController::startStableWalk() {
    is_walking = true;
}




void alfarobi::StableWalkController::stopStableWalk() {
    is_walking = false;
}