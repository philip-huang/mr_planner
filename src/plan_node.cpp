#include <fstream>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <planner.h>
#include <adg.h>
#include <logger.h>
#include <lego/Lego.hpp>


int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_arm_joint_space_planner");
    
    // start ros node
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Read the task configuration in JSON
    std::string config_fname;
    nh.param<std::string>("config_fname", config_fname, "config.json");

    std::ifstream config_file(config_fname, std::ifstream::binary);
    Json::Value config;
    config_file >> config;
    
    // Initialize the robot's moveit interface

    // Initialize the precedence-constrained activity sequence

    // Initialize the planner

    // Plan the sequence

    // Postprocess the sequence

    // Execute the sequence
    
    ros::shutdown();
    return 0;
}
