#include "yk_interface.h"

YK_Interface::YK_Interface(std::string group_name, ros::NodeHandle &nh_)
    : move_group_(group_name), group_name_(group_name)
{
    ROS_INFO_STREAM("planning frame.." << move_group_.getPlanningFrame());
    ROS_INFO_STREAM("pose reference frame.." << move_group_.getPoseReferenceFrame());
    ROS_INFO_STREAM("End Effector.." << move_group_.getEndEffector());
    ROS_INFO_STREAM("End Effector Link.." << move_group_.getEndEffectorLink());

    max_velocity_scaling_factor_ = 0.1;
    max_acceleration_scaling_factor_ = 0.3;
    move_group_.setMaxAccelerationScalingFactor(max_acceleration_scaling_factor_);
    move_group_.setMaxVelocityScalingFactor(max_velocity_scaling_factor_);

    // SERVICE SERVERS
    executeTrajectory_server_ = nh_.advertiseService("yk_execute_trajectory", &YK_Interface::executeTrajectory, this);
    //stopTrajectory_server_ = nh_.advertiseService("yk_stop_trajectory", &YK_Interface::stopTrajectory, this);
}

bool YK_Interface::executeTrajectory(moveit_msgs::ExecuteKnownTrajectory::Request &req,
                                     moveit_msgs::ExecuteKnownTrajectory::Response &res)
{
    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = req.trajectory;

    moveit_msgs::MoveItErrorCodes ret_val = move_group_.execute(plan);

    if (ret_val.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
        std::cerr << group_name_ << " Move failed due to ERROR CODE=" << ret_val;
        return false;
    }
    std::cout <<  group_name_ <<" Move successful" << std::endl;

    async_spinner.stop();
    return true;
}

// bool YK_Interface::stopTrajectory(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
// {
//  ros::AsyncSpinner async_spinner(1);
//  async_spinner.start();

//  move_group_.stop();
//  res.success = true;
//  res.message = "Trajectory stopped";

//  async_spinner.stop();
//  return true;
// }


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "yk_interface");

    ros::AsyncSpinner async_spinner(1);
    async_spinner.start();

    ROS_INFO("yk_interface node starting");

    std::string group_name;
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("group_name", group_name, "manipulator");

    YK_Interface gp4(group_name, nh_private);

    ROS_INFO("yk_interface node started");

    // ros::spin();
    ros::waitForShutdown();
}
