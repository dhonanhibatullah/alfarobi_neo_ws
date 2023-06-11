#include <ros/ros.h>
#include <ros/package.h>
#include <yaml-cpp/yaml.h>

#include <alfarobi_controller/dynamics/StableWalk.h>
#include <alfarobi_controller/kinematics/PosePlayer.h>
#include <alfarobi_lib/Math/Plotter.h>



int main(int argc, char **argv) {

    /* Parse robot information from alfarobi_main/config/robot_information.yaml */
    YAML::Node  robot_info  = YAML::LoadFile(ros::package::getPath("alfarobi_main") + (std::string)"/config/robot_information.yaml");
    std::string robot_name  = robot_info["name"].as<std::string>();


    /* Initialise ROS node */
    ros::init(argc, argv, robot_name + (std::string)"_controller_main_node");
    ros::NodeHandle nh;
    ros::Rate loop_rate(30);


    /* Classes */
    alfarobi::StableWalkController  stable_walk_controller(&nh, robot_name);
    alfarobi::PosePlayer            pose_player(&nh, robot_name);
    
    stable_walk_controller.setVelocityStableWalk(0.3, 0.3, 0);

    /* Loop */
    while(ros::ok()) {

        // pose_player.setBodyPose(alfarobi::PosePlayer::WALKING);
        stable_walk_controller.playStableWalk();


        ros::spinOnce();
        loop_rate.sleep();

        if(stable_walk_controller.stoppppphhh() == 1) {
            break;
        }
    }



    alfarobi::Plotter plt = alfarobi::Plotter(2, 1);
    alfarobi::Plotter::PlotConfig cfg;
    std::vector<double> nval, xval, yval;
    stable_walk_controller.plot(nval, xval, yval);

    /* Plot 1 */
    plt.setTitle(0, 0, "x-axis Plot");
    cfg.label = "x[n]";
    cfg.line_color = "blue";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.setXLabel(0, 0, "n");
    plt.setYLabel(0, 0, "x[n]");
    plt.setTitle(0, 0, "x-axis");
    plt.plot(nval, xval, cfg, 0, 0);


    /* Plot 1 */
    plt.setTitle(1, 0, "y-axis Plot");
    cfg.label = "y[n]";
    cfg.line_color = "purple";
    cfg.plot_style = PLOT_STYLE_INTERPOLATE;
    plt.setXLabel(1, 0, "n");
    plt.setYLabel(1, 0, "y[n]");
    plt.setTitle(1, 0, "y-axis");
    plt.plot(nval, yval, cfg, 1, 0);

    plt.show();

    return 0;
}