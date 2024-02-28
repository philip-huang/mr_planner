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
#include <lego/Lego.hpp>

const std::string PLANNING_GROUP = "dual_arms";

struct GoalPose {
    bool use_robot;
    geometry_msgs::PoseStamped pose;
    std::vector<double> joint_values;
};

bool parsePoseLine(const std::string& line, std::vector<GoalPose>& poses) {
    std::istringstream stream(line);
    GoalPose pose1, pose2;
    pose1.joint_values.resize(7, 0);
    pose2.joint_values.resize(7, 0);

    // Read use_robot flags and pose data for each robot
    // line format: use_robot1,x,y,z,qx,qy,qz,qw,use_robot2,x,y,z,qx,qy,qz,qw
    char c;
    stream >> pose1.use_robot >> c >> pose1.pose.pose.position.x >> c >> pose1.pose.pose.position.y >> c >> pose1.pose.pose.position.z >> c
         >> pose1.pose.pose.orientation.x >> c >> pose1.pose.pose.orientation.y >> c >> pose1.pose.pose.orientation.z >> c >> pose1.pose.pose.orientation.w >> c 
         >> pose1.joint_values[0] >> c >> pose1.joint_values[1] >> c >> pose1.joint_values[2] >> c >> pose1.joint_values[3] >> c >> pose1.joint_values[4] >> c >> pose1.joint_values[5] >> c
         >> pose2.use_robot >> c >> pose2.pose.pose.position.x >> c >> pose2.pose.pose.position.y >> c >> pose2.pose.pose.position.z >> c 
         >> pose2.pose.pose.orientation.x >> c >> pose2.pose.pose.orientation.y >> c >> pose2.pose.pose.orientation.z >> c >> pose2.pose.pose.orientation.w >> c
         >> pose2.joint_values[0] >> c >> pose2.joint_values[1] >> c >> pose2.joint_values[2] >> c >> pose2.joint_values[3] >> c >> pose2.joint_values[4] >> c >> pose2.joint_values[5];

    // Check for stream errors
    if (stream.fail()) {
        ROS_ERROR("Failed to parse line: %s", line.c_str());
        return false;
    }
    pose1.pose.header.frame_id = "world";
    pose2.pose.header.frame_id = "world";
    for (size_t i = 0; i < 6; i++) {
        pose1.joint_values[i] = pose1.joint_values[i] * M_PI / 180.0;
        pose2.joint_values[i] = pose2.joint_values[i] * M_PI / 180.0;
    }

    poses.push_back(pose1);
    poses.push_back(pose2);

    return true;
}

void readPosesFromFile(const std::string& file_path, std::vector<std::vector<GoalPose>>& all_poses) {
    std::ifstream file(file_path);
    std::string line;
    if (!file.is_open()) {
        ROS_ERROR("Could not open file: %s", file_path.c_str());
        return;
    }

    while (getline(file, line)) {
        std::vector<GoalPose> poses;
        if (parsePoseLine(line, poses)) {
            all_poses.push_back(poses);
        }
    }

    file.close();
}

class DualArmPlanner {
public:
    DualArmPlanner(const std::string &planner_type) : planner_type_(planner_type) {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model_ = robot_model_loader.getModel();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        kinematic_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
        kinematic_state_->setToDefaultValues();
         // Create the planning scene from robot model
        // Planning scene monitor.
        planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        planning_scene_monitor_->startSceneMonitor();
        planning_scene_monitor_->startStateMonitor();
        planning_scene_monitor_->startWorldGeometryMonitor();
        planning_scene_monitor_->requestPlanningSceneState();
        ros::Duration(0.3).sleep();
        planning_scene_ = planning_scene_monitor_->getPlanningScene();

        instance_ = std::make_shared<MoveitInstance>(robot_model_, kinematic_state_, move_group_, planning_scene_);
        instance_->setNumberOfRobots(2);
        instance_->setRobotNames({"left_arm", "right_arm"});
        pp_planner_ = std::make_shared<PriorityPlanner>(instance_);

    }

    bool solveIKJointlyForPose(const EigenSTL::vector_Isometry3d &poses, 
                                            const std::string& group_name,
                                            const std::vector<std::string>& eef_links,
                                            std::vector<double>& joint_values) {
        
        // Function to check state validity, considering collisions
        moveit::core::GroupStateValidityCallbackFn stateValidityCheckFn = [this](robot_state::RobotState* robot_state, 
                                                    const robot_state::JointModelGroup* jm_group, 
                                                    const double* joint_group_variable_values)->bool {
            robot_state->setJointGroupPositions(jm_group, joint_group_variable_values);
            robot_state->update();
            collision_detection::CollisionRequest collision_request;
            collision_detection::CollisionResult collision_result;
            collision_request.group_name = jm_group->getName();
            planning_scene_->checkCollision(collision_request, collision_result, *robot_state);
            return !collision_result.collision;
        };

        const robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        std::vector<std::vector<double>> consistency_limits;
        
        kinematic_state_->setToDefaultValues();
        bool found_ik = kinematic_state_->setFromIKSubgroups(joint_model_group,
                            poses, eef_links, consistency_limits, 
                            0.0, // timeout
                            stateValidityCheckFn);
        if (found_ik) {
            kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
            return true;
        } else {
            ROS_ERROR("Failed to find IK solution for end effectors");
            return false;
        }
    }

    bool solveIKForPose(const geometry_msgs::PoseStamped& pose, const std::string& group_name, const std::string& eef_link, std::vector<double>& joint_values) {
        const robot_state::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_name);
        auto ik_link = robot_model_->getLinkModel(eef_link);
        if (!ik_link) {
            ROS_ERROR("Could not find end effector link: %s", eef_link.c_str());
            return false;
        }

        // Convert Pose to Eigen
        Eigen::Isometry3d target_pose_eigen;
        tf2:fromMsg(pose.pose, target_pose_eigen);

        kinematic_state_->setToDefaultValues();
        bool found_ik = kinematic_state_->setFromIK(joint_model_group, target_pose_eigen, eef_link);

        if (found_ik) {
            kinematic_state_->copyJointGroupPositions(joint_model_group, joint_values);
            return true;
        } else {
            ROS_ERROR("Failed to find IK solution for end effector: %s", eef_link.c_str());
            return false;
        }
    }


    bool planAndMoveJointSpace(const std::vector<GoalPose>& goal_poses, const std::vector<double>& joint_values) {
        bool success;
        if (planner_type_ == "BITstar") {
                move_group_->setPlannerId("BITstar");
                success = moveit_plan(joint_values);
        } else if (planner_type_ == "RRTConnect") {
                move_group_->setPlannerId("RRTConnect");
                success = moveit_plan(joint_values);
        } else if (planner_type_ == "PrioritizedPlanning") {
                success = priority_plan(goal_poses);
        }

        return success;
    }

    bool priority_plan(const std::vector<GoalPose>& goal_poses) {
        bool success = true;
        std::vector<double> current_joints = move_group_->getCurrentJointValues();
        for (size_t i = 0; i < goal_poses.size(); i++) {
            instance_->setStartPose(i, std::vector<double>(current_joints.begin() + i*7, current_joints.begin() + (i+1)*7));
            instance_->setGoalPose(i, goal_poses[i].joint_values);
        }
        success &= pp_planner_->plan();
        if (success) {
            ROS_INFO("Prioritized Planning succeeded");
            std::vector<RobotTrajectory> solution;
            success &= pp_planner_->getPlan(solution);
            // convert solution to moveit plan and execute
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            trajectory_msgs::JointTrajectory &joint_traj = my_plan.trajectory_.joint_trajectory;
            joint_traj.joint_names = move_group_->getJointNames();

            // get the max time
            double max_time = 0.0;
            for (size_t i = 0; i < solution.size(); i++) {
                max_time = std::max(max_time, solution[i].times.back());
            }
            double dt = 0.1;
            int num_points = max_time / dt + 1;
            joint_traj.points.resize(num_points);
            for (int i = 0; i < num_points; i++) {
                joint_traj.points[i].positions.resize(joint_traj.joint_names.size());
                joint_traj.points[i].velocities.resize(joint_traj.joint_names.size());
                joint_traj.points[i].accelerations.resize(joint_traj.joint_names.size());
                joint_traj.points[i].time_from_start = ros::Duration(i * dt);
            }

            // interpolate each trajectory
            for (int i = 0; i < solution.size(); i++) {
                int ind = 0;
                auto traj = solution[i];
                
                for (int j = 0; j < num_points; j++) {
                    double t = j * dt;
                    while (ind + 1 < traj.times.size() && traj.times[ind + 1] <= t) {
                        ind++;
                    }
                    RobotPose pose_j_t;
                    if (ind + 1 == traj.times.size()) {
                        // assuming obstacle stays at the end of the trajectory
                        pose_j_t = traj.trajectory[ind];
                    } else {
                        double alpha = (t - traj.times[ind]) / (traj.times[ind + 1] - traj.times[ind]);
                        pose_j_t = instance_->interpolate(traj.trajectory[ind], traj.trajectory[ind + 1], alpha);
                    }

                    for (int d = 0; d < pose_j_t.joint_values.size(); d++) {
                        joint_traj.points[j].positions[i*7 + d] = pose_j_t.joint_values[d];
                    }
                }
            }

            // compute velocities and accelerations with central difference
            for (int i = 1; i < num_points - 1; i++) {
                for (int j = 0; j < joint_traj.joint_names.size(); j++) {
                    joint_traj.points[i].velocities[j] = (joint_traj.points[i+1].positions[j] - joint_traj.points[i-1].positions[j]) / (2 * dt);
                    joint_traj.points[i].accelerations[j] = (joint_traj.points[i+1].positions[j] - 2 * joint_traj.points[i].positions[j] + joint_traj.points[i-1].positions[j]) / (dt * dt);
                }
            }

            // execute the plan
            move_group_->execute(my_plan);
        }
        return success;
    }

    bool moveit_plan(const std::vector<double>& joint_values) {
        move_group_->setJointValueTarget(joint_values);
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        // // print the plan
        // auto joint_traj = my_plan.trajectory_.joint_trajectory;
        // //names first
        // for (auto name : joint_traj.joint_names) {
        //     std::cout << name << " ";
        // }
        // std::cout << std::endl;
        // for (size_t i = 0; i < joint_traj.points.size(); i++) {
        //     std::cout << "Pos " << i << ": ";
        //     for (size_t j = 0; j < joint_traj.points[i].positions.size(); j++) {
        //         std::cout << joint_traj.points[i].positions[j] << " ";
        //     }
        //     // velocity
        //     std::cout << "Vel " << i << ": ";
        //     for (size_t j = 0; j < joint_traj.points[i].velocities.size(); j++) {
        //         std::cout << joint_traj.points[i].velocities[j] << " ";
        //     }
        //     // acceleration
        //     std::cout << "Acc " << i << ": ";
        //     for (size_t j = 0; j < joint_traj.points[i].accelerations.size(); j++) {
        //         std::cout << joint_traj.points[i].accelerations[j] << " ";
        //     }
        //     std::cout << std::endl;
        // }
        if (success) {
            ROS_INFO("Planning succeeded");
            // set the new start state to the target state
            move_group_->execute(my_plan);
        } else {
            ROS_ERROR("Planning failed");
        }

        return success;
    }

    bool planAndMovePose(const std::vector<GoalPose>& poses) {
        move_group_->setPoseTarget(poses[0].pose, "left_arm_link_camera");
        move_group_->setPoseTarget(poses[1].pose, "right_arm_link_camera");
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        move_group_->setPlannerId("RRTConnect");
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        if (success) {
            ROS_INFO("Planning succeeded");
            // set the new start state to the target state
            move_group_->execute(my_plan);
        } else {
            ROS_ERROR("Planning failed");
        }
        return success;
    }

    bool planAndMove(const std::vector<GoalPose>& poses) {
        std::vector<double> all_joints;
        std::vector<double> all_joints_given;
        for (auto pose : poses) {
            all_joints_given.insert(all_joints_given.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        for (size_t i = 0; i < all_joints_given.size(); i++) {
            ROS_INFO("Joint %lu: %f", i, all_joints_given[i]);
        }
        return planAndMoveJointSpace(poses, all_joints_given);

        // if (true /*poses[1].use_robot*/) {
        //     EigenSTL::vector_Isometry3d poses_eigen;
        //     for (size_t i = 0; i < poses.size(); i++) {
        //         Eigen::Isometry3d pose_eigen;
        //         tf2::fromMsg(poses[i].pose.pose, pose_eigen);
        //         poses_eigen.push_back(pose_eigen);
        //     }

        //     bool solve_ik = solveIKJointlyForPose(poses_eigen, 
        //         "dual_arms", {"left_arm_link_camera", "right_arm_link_camera"}, 
        //         all_joints);
        //     if (!solve_ik) {
        //         ROS_ERROR("Failed to solve IK for both arms");
        //         return false;
        //     }            
        // }
        // else {
        //     std::vector<double> joint_values;
        //     // Solve IK and stack the pose
        //     bool solve_ik1 = solveIKForPose(poses[0].pose, "left_arm", "left_arm_link_camera", joint_values);
        //     if (!solve_ik1) {
        //         ROS_ERROR("Failed to solve IK for right arm");
        //         return false;
        //     }
        //     all_joints.insert(all_joints.end(), joint_values.begin(), joint_values.end());

        //     // get a default pose for the right arm
        //     std::vector<double> default_joint_values = {0.0, 0.0, 0.0, 0.0, -1.5, 0.0};
        //     all_joints.insert(all_joints.end(), default_joint_values.begin(), default_joint_values.end());

        // }
        
        // ROS_INFO("Solved IK for both arms. Planning and moving to the target pose.");
        // for (size_t i = 0; i < all_joints.size(); i++) {
        //     ROS_INFO("Joint %lu: %f %f", i, all_joints[i], all_joints_given[i]);
        // }
        // return planAndMoveJointSpace(all_joints);
    }

    void checkFK(const std::vector<GoalPose> &poses) {
        kinematic_state_->setJointGroupPositions("left_arm", poses[0].joint_values);
        kinematic_state_->setJointGroupPositions("right_arm", poses[1].joint_values);
        kinematic_state_->update();
        
        const Eigen::Isometry3d& left_pose = kinematic_state_->getGlobalLinkTransform("left_arm_flange");
        const Eigen::Isometry3d& right_pose = kinematic_state_->getGlobalLinkTransform("right_arm_flange");
        Eigen::Quaterniond left_quat(left_pose.rotation());
        Eigen::Quaterniond right_quat(right_pose.rotation());
        ROS_INFO_STREAM("Left pose: " << left_pose.translation() << " " << left_quat.x() << " " << left_quat.y() << " " << left_quat.z() << " " << left_quat.w());
        ROS_INFO_STREAM("Right pose: " << right_pose.translation() << " " << right_quat.x() << " " << right_quat.y() << " " << right_quat.z() << " " << right_quat.w());
        ROS_INFO_STREAM("Left pose given: " << poses[0].pose.pose.position << " " << poses[0].pose.pose.orientation);
        ROS_INFO_STREAM("Right pose given: " << poses[1].pose.pose.position << " " << poses[1].pose.pose.orientation);
    
    }

    bool setLegoFactory(const std::string &config_fname, const std::string &root_pwd)
    {
        std::ifstream config_file(config_fname, std::ifstream::binary);
        Json::Value config;
        config_file >> config;
        std::string r1_DH_fname = root_pwd + config["r1_DH_fname"].asString();
        std::string r1_DH_tool_fname = root_pwd + config["r1_DH_tool_fname"].asString();
        std::string r1_DH_tool_assemble_fname = root_pwd + config["r1_DH_tool_assemble_fname"].asString();
        std::string r1_DH_tool_disassemble_fname = root_pwd + config["r1_DH_tool_disassemble_fname"].asString();
        std::string r1_robot_base_fname = root_pwd + config["Robot1_Base_fname"].asString();
        std::string r2_DH_fname = root_pwd + config["r2_DH_fname"].asString();
        std::string r2_DH_tool_fname = root_pwd + config["r2_DH_tool_fname"].asString();
        std::string r2_DH_tool_assemble_fname = root_pwd + config["r2_DH_tool_assemble_fname"].asString();
        std::string r2_DH_tool_disassemble_fname = root_pwd + config["r2_DH_tool_disassemble_fname"].asString();
        std::string r2_robot_base_fname = root_pwd + config["Robot2_Base_fname"].asString();

        std::string env_setup_fname = root_pwd + config["Env_Setup_fname"].asString();
        std::string lego_lib_fname = root_pwd + config["lego_lib_fname"].asString();

        std::string task_fname = root_pwd + config["Task_Graph_fname"].asString();
        std::ifstream task_file(task_fname, std::ifstream::binary);
        Json::Value task_json;
        task_file >> task_json;

        bool assemble = config["Start_with_Assemble"].asBool();

        set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr_->setup(env_setup_fname, lego_lib_fname, assemble, task_json, 
                        r1_DH_fname, r1_DH_tool_fname, r1_DH_tool_disassemble_fname, r1_DH_tool_assemble_fname, r1_robot_base_fname, 
                        r2_DH_fname, r2_DH_tool_fname, r2_DH_tool_disassemble_fname, r2_DH_tool_assemble_fname, r2_robot_base_fname, 
                        1, set_state_client_);
        return true;
    }

    bool initLegoPositions() {
        if (lego_ptr_ == nullptr) {
            ROS_ERROR("Lego pointer is not initialized");
            return false;
        }

        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        for (const auto & name : brick_names) {
            addMoveitCollisionObject(name);
        }

        return true;

    }

    void addMoveitCollisionObject(const std::string &name) {
        moveit_msgs::CollisionObject co;
        co.header.frame_id = "world";
        co.header.stamp = ros::Time::now();
        co.id = name;

        // define shape primitive
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);

        double x, y, z;
        lego_ptr_->get_brick_sizes(name, x, y, z);
        primitive.dimensions[primitive.BOX_X] = x; // 2 stud, 1 stud = 7.8mm
        primitive.dimensions[primitive.BOX_Y] = y; // 4 stud
        primitive.dimensions[primitive.BOX_Z] = z; // 1 stud height = 9.6mm + 1.7mm

        // define pose relative to frame id
        
        geometry_msgs::Pose box_pose = lego_ptr_->get_init_brick_pose(name);

        co.primitives.push_back(primitive);
        co.primitive_poses.push_back(box_pose);
        co.operation = co.ADD;

        std::vector<moveit_msgs::CollisionObject> collision_objects;
        collision_objects.push_back(co);
        planning_scene_interface_->addCollisionObjects(collision_objects);
        
        // wait 10 miliseconds for the planning scene to be ready
        ros::Duration(0.01).sleep();
        move_group_->attachObject(co.id, "world");
        ROS_INFO("Add an object into the world");

    }

private:
    ros::NodeHandle nh_;
    
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
    planning_scene::PlanningScenePtr planning_scene_;

    // lego pointer
    lego_manipulation::lego::Lego::Ptr lego_ptr_;

    // update gazebo state
    ros::ServiceClient set_state_client_;

    std::string planner_type_;
    std::shared_ptr<MoveitInstance> instance_;
    std::shared_ptr<PriorityPlanner> pp_planner_;

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans_;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_arm_joint_space_planner");
    
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();


    std::string file_path, planner_type, config_fname, root_pwd;
    //retrieve the file path as ros param
    nh.getParam("fullorder_targets_filename", file_path);
    nh.getParam("planner_type", planner_type);
    nh.getParam("config_fname", config_fname);
    nh.getParam("root_pwd", root_pwd);

    DualArmPlanner planner(planner_type);

    // wait 2 seconds
    ros::Duration(2).sleep();

    // Read the robot poses
    std::vector<std::vector<GoalPose>> all_poses;
    readPosesFromFile(file_path, all_poses);
    ROS_INFO("Read %lu poses", all_poses.size());

    // Read the lego poses
    planner.setLegoFactory(config_fname, root_pwd);
    planner.initLegoPositions();

    for (int i = 0; i < all_poses.size(); i++) {
        std::vector<GoalPose> poses = all_poses[i];
        ROS_INFO("use robot 1: %d, use robot 2: %d", poses[0].use_robot, poses[1].use_robot);
        //planner.checkFK(poses);
        planner.planAndMove(poses);
    }
    

    ros::shutdown();
    return 0;
}
