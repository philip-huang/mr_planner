#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
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
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    
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

    // add a 2x4 lego block to the planning scene
    moveit_msgs::CollisionObject co;
    co.header.frame_id = "left_arm_link_tool";
    co.header.stamp = ros::Time::now();
    co.id = "lego_block";

    // define shape primitive
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.016; // 2 stud, 1 stud = 7.8mm
    primitive.dimensions[primitive.BOX_Y] = 0.032; // 4 stud
    primitive.dimensions[primitive.BOX_Z] = 0.012; // 1 stud height = 9.6mm + 1.7mm

    // define pose relative to frame id
    
    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0.006;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.07;

    co.primitives.push_back(primitive);
    co.primitive_poses.push_back(box_pose);
    co.operation = co.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(co);
    planning_scene_interface.addCollisionObjects(collision_objects);
    
    // wait 300 miliseconds for the planning scene to be ready
    ros::Duration(0.3).sleep();
    move_group.attachObject(co.id, "left_arm_link_tool");
    ROS_INFO("Add an object into the world");

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