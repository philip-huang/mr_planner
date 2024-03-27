#ifndef YK_INTERFACE_H
#define YK_INTERFACE_H
/**
 * @file yk_interface.h
 * @brief This file contains some implementation of a test interface on MFI yk robots
*/

#include <ros/ros.h>

#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MoveItErrorCodes.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/kinematic_constraints/utils.h>
#include <string.h>

class YK_Interface
{
private:
    ros::ServiceServer executeTrajectory_server_;
    ros::ServiceServer stopTrajectory_server_;
    ros::NodeHandle nh_;
    moveit::planning_interface::MoveGroupInterface move_group_;

    double max_velocity_scaling_factor_;
    double max_acceleration_scaling_factor_;
    std::string group_name_;


public:
    YK_Interface(std::string group_name, ros::NodeHandle &nh_);

    bool executeTrajectory(moveit_msgs::ExecuteKnownTrajectory::Request &req, moveit_msgs::ExecuteKnownTrajectory::Response &res);

    // bool stopTrajectory(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
};

#endif
