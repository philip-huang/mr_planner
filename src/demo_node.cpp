#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// define Statetype as a vector of doubles
typedef std::vector<double> StateType;

bool checkState(const moveit::planning_interface::MoveGroupInterface& move_group,
            const planning_scene::PlanningScenePtr &planning_scene,
            const StateType &state) {
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = move_group.getName();
    c_req.contacts = true;
    c_req.max_contacts = 100;

    // set the robot state to the one we are checking
    moveit::core::RobotState robot_state = planning_scene->getCurrentStateNonConst();
    robot_state.setJointGroupPositions(move_group.getName(), state);

    c_res.clear();
    planning_scene->checkCollision(c_req, c_res, robot_state);
    if (c_res.collision) {
        ROS_INFO("State is in collision");
        return false;
    }
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Declare the MoveGroupInterface
    std::string movegroup_name = "dual_arms";
    moveit::planning_interface::MoveGroupInterface move_group(movegroup_name);
    
    // Create the planning scene from robot model
    // Planning scene monitor.
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startStateMonitor();
    planning_scene_monitor->startWorldGeometryMonitor();
    planning_scene_monitor->requestPlanningSceneState();
    // wait 300 miliseconds for the planning scene to be ready
    ros::Duration(0.3).sleep();
    planning_scene::PlanningScenePtr planning_scene = planning_scene_monitor->getPlanningScene();

    // // Visualization
    // namespace rvt = rviz_visual_tools;
    // moveit_visual_tools::MoveItVisualTools visual_tools("base"); // Change base_frame to your base frame
    // visual_tools.deleteAllMarkers();

    // Set the planner target
    move_group.setNamedTarget("left_push");

    // Declare the Plan object
    moveit::planning_interface::MoveGroupInterface::Plan plan;


    // Plan the motion
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    // Print the result
    ROS_INFO_NAMED("plan_node", "Plan (success): %s", success ? "SUCCESS" : "FAILED");

    // Execute
    if (success) {
        // visual_tools.publishTrajectoryLine(plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(movegroup_name));
        // visual_tools.trigger();
        move_group.execute(plan);
    }

    StateType state1 = {-2.872096, -2.065656, 0.026423, 0.158731, 2.090787, -1.723881, 0.,
                                 0., 0., 0., 0., 0., 0., 0.};
    
    bool isValid = checkState(move_group, planning_scene, state1);
    ROS_INFO_NAMED("plan_node", "State (isValid): %s", isValid ? "VALID" : "INVALID");

    ros::shutdown();
    return 0;
}