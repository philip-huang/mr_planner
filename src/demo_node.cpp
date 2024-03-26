#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

#include "tpg.h"
#include "planner.h"

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

void test_add_lego(const moveit::planning_interface::PlanningSceneInterface& planning_scene_interface,
            moveit::planning_interface::MoveGroupInterface& move_group) {
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
}

void test_collision(moveit::planning_interface::MoveGroupInterface& move_group,
            const planning_scene::PlanningScenePtr &planning_scene) {
    StateType state1 = {-2.872096, -2.065656, 0.026423, 0.158731, 2.090787, -1.723881, 0.,
                                 0., 0., 0., 0., 0., 0., 0.};
    
    bool isValid = checkState(move_group, planning_scene, state1);
    ROS_INFO_NAMED("plan_node", "State (isValid): %s", isValid ? "VALID" : "INVALID");
}

void test_planning(moveit::planning_interface::MoveGroupInterface& move_group,
                    const std::string &pose_name) {
    // Set the planner target
    move_group.setNamedTarget(pose_name);

    // Declare the Plan object
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    // use AIT* for 1 second
    move_group.setPlannerId("AITstar");
    move_group.setPlanningTime(2.0);

    // Plan the motion
    bool success = (move_group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    double r1_time = -1, r2_time = -1;
    std::vector<double> goal_poses;
    for (int i = 0; i < 14; i++) {
        goal_poses.push_back(plan.trajectory_.joint_trajectory.points.back().positions[i]);
    }
    for (int t = plan.trajectory_.joint_trajectory.points.size() - 1; t > -1; t--) {
        bool robot1_reached = true;
        bool robot2_reached = true;
        for (int i = 0; i < 7; i++) {
            if (fabs(plan.trajectory_.joint_trajectory.points[t].positions[i] - goal_poses[i]) > 0.001) {
                robot1_reached = false;
            }
            if (fabs(plan.trajectory_.joint_trajectory.points[t].positions[i+7] - goal_poses[i+7]) > 0.001) {
                robot2_reached = false;
            }
        }
        if (!robot1_reached && r1_time < 0) {
            r1_time = plan.trajectory_.joint_trajectory.points[t].time_from_start.toSec();
        }
        if (!robot2_reached && r2_time < 0) {
            r2_time = plan.trajectory_.joint_trajectory.points[t].time_from_start.toSec();
        }
    }
    double flowtime = r1_time + r2_time;
    ROS_INFO_NAMED("plan_node", "Flowtime: %f", flowtime);

    // Print the result
    ROS_INFO_NAMED("plan_node", "Plan (success): %s", success ? "SUCCESS" : "FAILED");

    // Execute
    if (success) {
        // visual_tools.publishTrajectoryLine(plan.trajectory_, move_group.getCurrentState()->getJointModelGroup(movegroup_name));
        // visual_tools.trigger();
        move_group.execute(plan);
    }
}

class TestPPPlanning {
public:
    TestPPPlanning(ros::NodeHandle &nh,
                    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                    planning_scene::PlanningScenePtr &planning_scene,
                    robot_state::RobotStatePtr  &kinematic_state,
                    const std::vector<std::string> &group_names,
                    const std::string &planner_name,
                    double planning_time_limit,
                    bool async,
                    bool mfi) : nh(nh), move_group(move_group), planning_scene(planning_scene), 
                kinematic_state(kinematic_state), group_names(group_names), async(async),
                planning_time_limit(planning_time_limit), mfi(mfi), planner_name(planner_name) { }

    void test(const std::string &pose_name, const TPG::TPGConfig &tpg_config) {
        auto instance = std::make_shared<MoveitInstance>(kinematic_state, move_group, planning_scene);
        instance->setNumberOfRobots(2);
        instance->setRobotNames({"left_arm", "right_arm"});

        /*
        Set joint name and record start locations
        */
        std::vector<std::string> joint_names = move_group->getVariableNames();
        std::vector<std::vector<std::string>> joint_names_split;
        left_arm_joint_state_received = false;
        right_arm_joint_state_received = false;

        if (mfi) {
            std::vector<std::string> names;
            names.push_back("joint_1_s");
            names.push_back("joint_2_l");
            names.push_back("joint_3_u");
            names.push_back("joint_4_r");
            names.push_back("joint_5_b");
            names.push_back("joint_6_t");
            joint_names_split.push_back(names);
            joint_names_split.push_back(names);
            current_joints.resize(14, 0.0);
           
            left_arm_sub = nh.subscribe(group_names[0] + "/joint_states", 1, &TestPPPlanning::left_arm_joint_state_cb, this);
            right_arm_sub = nh.subscribe(group_names[1] + "/joint_states", 1, &TestPPPlanning::right_arm_joint_state_cb, this);

            while (!left_arm_joint_state_received || !right_arm_joint_state_received) {
                ros::Duration(0.1).sleep();
            }
        }
        else {
            current_joints.resize(14, 0.0);
            for (int i = 0; i < 2; i++) {
                std::vector<std::string> name_i;
                for (size_t j = 0; j < 7; j++) {
                    name_i.push_back(joint_names[i*7+j]);
                }
                joint_names_split.push_back(name_i);
            }
            dual_arm_sub = nh.subscribe("/joint_states", 1, &TestPPPlanning::dual_arm_joint_state_cb, this);
            while (!left_arm_joint_state_received || !right_arm_joint_state_received) {
                ros::Duration(0.1).sleep();
            }
        }

        /*
        Set the start and goal poses for planner instance
        */
        std::map<std::string, double> goal_joints = move_group->getNamedTargetValues(pose_name);
        for (int i = 0; i < 2; i++) {
            std::vector<double> start_pose(7, 0.0);
            std::vector<double> goal_pose(7, 0.0);
            for (size_t j = 0; j < 7; j++) {
                start_pose[j] = current_joints[i*7+j];
                goal_pose[j] = goal_joints[joint_names[i*7+j]];
            }
            instance->setStartPose(i, start_pose);
            instance->setGoalPose(i, goal_pose);
        }

        /*
        Call the planner
        */
        std::vector<RobotTrajectory> solution;
        if (planner_name == "PrioritizedPlanning") {
            PriorityPlanner planner(instance);
            PlannerOptions options(planning_time_limit, 1000000);
            bool success = planner.plan(options);
            if (!success) {
                ROS_INFO("Failed to plan");
                return;
            }
            
            success &= planner.getPlan(solution);
        }
        else {
            move_group->setNamedTarget(pose_name);
            move_group->setPlannerId(planner_name);
            move_group->setPlanningTime(planning_time_limit);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            bool success = (move_group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success) {
                ROS_INFO("Failed to plan");
                return;
            }
            solution.resize(2);
            solution[0].robot_id = 0;
            solution[1].robot_id = 1;
            std::cout << "Plan size: " << plan.trajectory_.joint_trajectory.points.size() << std::endl;
            for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {
                RobotPose pose1 = instance->initRobotPose(0);
                RobotPose pose2 = instance->initRobotPose(1);

                for (int j = 0; j < 7; j++) {
                    pose1.joint_values[j] = plan.trajectory_.joint_trajectory.points[i].positions[j];
                    pose2.joint_values[j] = plan.trajectory_.joint_trajectory.points[i].positions[j+7];
                }
                solution[0].trajectory.push_back(pose1);
                solution[1].trajectory.push_back(pose2);
                solution[0].times.push_back(plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec());
                solution[1].times.push_back(plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec());
            }

            solution[0].cost = solution[0].times.back();
            solution[1].cost = solution[1].times.back();
            
        }

        /*
        Build the TPG
        */
        tpg.reset();
        tpg.init(instance, solution, tpg_config);
        tpg.saveToDotFile("tpg.dot");

        /*
        * Execute the plan
        */
        if (async) {
            std::vector<ros::ServiceClient> clients;
            for (auto group_name: group_names) {
                clients.push_back(nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(group_name + "/yk_execute_trajectory"));
            }
            
            tpg.moveit_mt_execute(joint_names_split, clients);
        } else {
            tpg.moveit_execute(instance, move_group);
        }
    
    }

    void left_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 6; i++) {
            current_joints[i] = msg->position[i];
        }
        tpg.update_joint_states(msg->position, 0);
        left_arm_joint_state_received = true;
    }

    void right_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 6; i++) {
            current_joints[7+i] = msg->position[i];
        }
        tpg.update_joint_states(msg->position, 1);
        right_arm_joint_state_received = true;
    }

    void dual_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 7; i++) {
            current_joints[i] = msg->position[i];
            current_joints[7+i] = msg->position[7+i];
        }
        std::vector<double> left_joints(msg->position.begin(), msg->position.begin()+7);
        std::vector<double> right_joints(msg->position.begin()+7, msg->position.begin()+14);
        tpg.update_joint_states(left_joints, 0);
        tpg.update_joint_states(right_joints, 1);
        left_arm_joint_state_received = true;
        right_arm_joint_state_received = true;
    }

private:
    ros::NodeHandle nh;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    planning_scene::PlanningScenePtr planning_scene;
    robot_state::RobotStatePtr kinematic_state;
    std::string pose_name;
    std::string planner_name;
    std::vector<std::string> group_names;
    bool async;
    bool mfi;
    double planning_time_limit = 2.0;
    double tpg_time_limit = 2.0;

    TPG::TPG tpg;
    bool left_arm_joint_state_received = false;
    bool right_arm_joint_state_received = false;
    std::vector<double> current_joints;
    ros::Subscriber left_arm_sub, right_arm_sub, dual_arm_sub;
};
    

int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Declare the MoveGroupInterface
    std::string movegroup_name = "dual_arms";
    auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(movegroup_name);

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

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    auto robot_model = robot_model_loader.getModel();
    auto kinematic_state = std::make_shared<robot_state::RobotState>(robot_model);
    kinematic_state->setToDefaultValues();

    // test_add_lego(planning_scene_interface, move_group);

    // test_collision(move_group, planning_scene);

    // test_planning(*move_group);

    std::vector<std::string> group_names = {"left_arm", "right_arm"};
    for (int i = 0; i < 2; i++) {
        if (nh_private.hasParam("group_name_" + std::to_string(i))) {
            nh_private.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }

    bool async = true;
    bool mfi = true;
    bool shortcut = true;
    double planning_time_limit = 2.0;
    std::string pose_name = "left_push_up";
    std::string planner_name = "PrioritizedPlanning";
    if (nh_private.hasParam("mfi")) {
        nh_private.getParam("mfi", mfi);
    }
    if (nh_private.hasParam("async")) {
        nh_private.getParam("async", async);
    }
    if (nh_private.hasParam("pose_name")) {
        nh_private.getParam("pose_name", pose_name);
    }
    if (nh_private.hasParam("planner_name")) {
        nh_private.getParam("planner_name", planner_name);
    }
    if (nh_private.hasParam("planning_time_limit")) {
        nh_private.getParam("planning_time_limit", planning_time_limit);
        std::cout << "Planning time limit: " << planning_time_limit << std::endl;
    }
    if (nh_private.hasParam("shortcut")) {
        nh_private.getParam("shortcut", shortcut);
    }

    auto pp_tester = TestPPPlanning(nh, move_group, planning_scene, kinematic_state, group_names, 
        planner_name, planning_time_limit, async, mfi);
    //test_planning(*move_group, pose_name);
    TPG::TPGConfig tpg_config;
    tpg_config.random_shortcut_time = 1.0;
    tpg_config.shortcut = shortcut;
    pp_tester.test(pose_name, tpg_config);

    for (int i=1; ; i++ ) {
        if (nh_private.hasParam("pose_name" + std::to_string(i))) {
            nh_private.getParam("pose_name" + std::to_string(i), pose_name);
            //test_planning(*move_group, pose_name);
            pp_tester.test(pose_name, tpg_config);
        }
        else {
            break;
        }
    }
    
    
    ros::shutdown();
    return 0;
}
