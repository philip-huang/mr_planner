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
    DualArmPlanner(const std::string &planner_type, const std::string &output_dir,
                const std::vector<std::string> &group_names, bool async, bool mfi) : 
        planner_type_(planner_type), output_dir_(output_dir), group_names_(group_names), async_(async), mfi_(mfi) {
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

        planning_scene_diff_client = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();
        get_planning_scene_client = nh_.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");
        get_planning_scene_client.waitForExistence();

        instance_ = std::make_shared<MoveitInstance>(kinematic_state_, move_group_->getName(), planning_scene_);
        instance_->setNumberOfRobots(2);
        instance_->setRobotNames({"left_arm", "right_arm"});
        instance_->setRobotDOF(0, 7);
        instance_->setRobotDOF(1, 7);
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


    bool planJointSpace(const std::vector<GoalPose>& goal_poses, const std::vector<double>& joint_values,
                std::vector<RobotTrajectory> &solution) {
        bool success;
        if (planner_type_ == "BITstar") {
                move_group_->setPlannerId("BITstar");
                success = moveit_plan(joint_values, solution);
        } else if (planner_type_ == "RRTConnect") {
                move_group_->setPlanningTime(1.0);
                move_group_->setPlannerId("RRTConnect");
                success = moveit_plan(joint_values, solution);
        } else if (planner_type_ == "PrioritizedPlanning") {
                success = priority_plan(goal_poses, solution);
        }

        return success;
    }

    void setup_once() {
        /*
        Set joint name and record start locations. Necessary for execution
        */
        joint_names_ = move_group_->getVariableNames();
        joint_names_split_.clear();
        left_arm_joint_state_received = false;
        right_arm_joint_state_received = false;

        if (mfi_) {
            std::vector<std::string> names;
            names.push_back("joint_1_s");
            names.push_back("joint_2_l");
            names.push_back("joint_3_u");
            names.push_back("joint_4_r");
            names.push_back("joint_5_b");
            names.push_back("joint_6_t");
            joint_names_split_.push_back(names);
            joint_names_split_.push_back(names);
            current_joints_.resize(14, 0.0);
           
            left_arm_sub = nh_.subscribe(group_names_[0] + "/joint_states", 1, &DualArmPlanner::left_arm_joint_state_cb, this);
            right_arm_sub = nh_.subscribe(group_names_[1] + "/joint_states", 1, &DualArmPlanner::right_arm_joint_state_cb, this);

        }
        else {
            current_joints_.resize(14, 0.0);
            for (int i = 0; i < 2; i++) {
                std::vector<std::string> name_i;
                for (size_t j = 0; j < 7; j++) {
                    name_i.push_back(joint_names_[i*7+j]);
                }
                joint_names_split_.push_back(name_i);
            }
            dual_arm_sub = nh_.subscribe("/joint_states", 1, &DualArmPlanner::dual_arm_joint_state_cb, this);
        }
    }

    bool priority_plan(const std::vector<GoalPose>& goal_poses, std::vector<RobotTrajectory>& solution) {
        bool success = true;
        for (size_t i = 0; i < goal_poses.size(); i++) {
            std::vector<double> start_pose(7, 0.0);
            for (size_t j = 0; j < 7; j++) {
                start_pose[j] = current_joints_[i*7+j];
            }
            instance_->setStartPose(i, start_pose);
            instance_->setGoalPose(i, goal_poses[i].joint_values);
        }
        // Use the PlanningSceneMonitor to update and get the current planning scene

        PlannerOptions options;
        options.max_planning_time = 2.0;
        success &= pp_planner_->plan(options);
        if (success) {
            ROS_INFO("Prioritized Planning succeeded");
            success &= pp_planner_->getPlan(solution);
        }
        return success;
    }

    bool moveit_plan(const std::vector<double>& joint_values, std::vector<RobotTrajectory>& solution) {
        move_group_->setJointValueTarget(joint_values);
        ROS_INFO("Planning with planner: %s", move_group_->getPlannerId().c_str());
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if (success) {
            ROS_INFO("Planning succeeded");
            convertSolution(instance_, my_plan, solution);
            // set the new start state to the target state
        } else {
            ROS_ERROR("Planning failed");
        }

        return success;
    }

    void set_tpg(std::shared_ptr<TPG::TPG> tpg) {
        tpg_ = tpg;
    }

    void reset_joint_states_flag() {
        left_arm_joint_state_received = false;
        right_arm_joint_state_received = false;
        while (!left_arm_joint_state_received || !right_arm_joint_state_received) {
            ros::Duration(0.01).sleep();
        }
    }

    bool execute(std::shared_ptr<TPG::TPG> tpg) {
        
        bool success = true;
        if (async_) {
            std::vector<ros::ServiceClient> clients;
            for (auto group_name: group_names_) {
                clients.push_back(nh_.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(group_name + "/yk_execute_trajectory"));
            }
            success &= tpg->moveit_mt_execute(joint_names_split_, clients);
        } else {
            success &= tpg->moveit_execute(instance_, move_group_);
        }
        return success;
    }

    bool saveTPG(std::shared_ptr<TPG::TPG> tpg, const std::string &filename) {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return false;
        }
        boost::archive::text_oarchive oa(ofs);
        oa << tpg;
        
        return true;
    }

    std::shared_ptr<TPG::TPG> loadTPG(const std::string &filename) {
        // open the file safely
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return nullptr;
        }
        std::shared_ptr<TPG::TPG> tpg = std::make_shared<TPG::TPG>();
        boost::archive::text_iarchive ia(ifs);
        ia >> tpg;
        return tpg;
    }

    bool planAndMove(const std::vector<GoalPose>& poses, const TPG::TPGConfig &tpg_config) {
        std::vector<double> all_joints;
        std::vector<double> all_joints_given;
        for (auto pose : poses) {
            all_joints_given.insert(all_joints_given.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        std::vector<RobotTrajectory> solution;
        bool success = planJointSpace(poses, all_joints_given, solution);
        if (success) {
            if (tpg_ == nullptr) {
                tpg_ = std::make_shared<TPG::TPG>();
            }
            tpg_->reset();
            
            success &= tpg_->init(instance_, solution, tpg_config);
            success &= tpg_->optimize(instance_, tpg_config);
            saveTPG(tpg_, output_dir_ + "/tpg_" + std::to_string(counter_) + ".txt");
            tpg_->saveToDotFile(output_dir_ + "/tpg_" + std::to_string(counter_) + ".dot");

            success &= execute(tpg_);
        }
        counter_++;
        return success;

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
        task_file >> task_json_;
        num_tasks_ = task_json_.size();
        
        std::string world_base_fname = root_pwd + config["world_base_fname"].asString();

        bool assemble = config["Start_with_Assemble"].asBool();

        set_state_client_ = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

        lego_ptr_ = std::make_shared<lego_manipulation::lego::Lego>();
        lego_ptr_->setup(env_setup_fname, lego_lib_fname, assemble, task_json_, world_base_fname,
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
        addMoveitCollisionObject("table");
        setCollision("table", "left_arm_link_tool", false);
        for (const auto & name : brick_names) {
            addMoveitCollisionObject(name);
            setCollision(name, "left_arm_link_tool", false);
        }

        return true;

    }

    void addMoveitCollisionObject(const std::string &name) {
        Object obj;
        
        // define the object
        obj.name = name;
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        obj.shape = Object::Shape::Box;

        // get the starting pose and size
        geometry_msgs::Pose box_pose;
        if (name == "table") {
            lego_ptr_->get_table_size(obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_table_pose();

        } else {
            lego_ptr_->get_brick_sizes(name, obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_init_brick_pose(name);
        }
        obj.x = box_pose.position.x;
        obj.y = box_pose.position.y;
        obj.z = box_pose.position.z - obj.height/2;
        obj.qx = box_pose.orientation.x;
        obj.qy = box_pose.orientation.y;
        obj.qz = box_pose.orientation.z;
        obj.qw = box_pose.orientation.w; 

        // add the object to the planning scene
        instance_->addMoveableObject(obj);

        // update the moveit simulation
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = instance_->getPlanningSceneDiff();
        planning_scene_diff_client.call(srv);
        
        moveit_msgs::GetPlanningScene srv_get;
        srv_get.request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
        if (!get_planning_scene_client.call(srv_get)) {
            ROS_WARN("Failed to get planning scene collision matrix");
            return;
        }

        lego_collision_objects_.insert(name);    
        current_acm_ = srv_get.response.scene.allowed_collision_matrix;
        
        ROS_INFO("Adding collision object %s at %f %f %f", name.c_str(), box_pose.position.x, box_pose.position.y, box_pose.position.z);
        ROS_INFO("Added collision object %s to world frame", name.c_str());
    }

    void attachMoveitCollisionObject(const std::string &name, int robot_id, const std::string &link_name) {
        instance_->attachObjectToRobot(name, robot_id, link_name);
        
        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = instance_->getPlanningSceneDiff();
        planning_scene_diff_client.call(srv);

        ROS_INFO("Attached collision object %s to %s", name.c_str(), link_name.c_str());
    }

    void detachMoveitCollisionObject(const std::string &name) {
        instance_->detachObjectFromRobot(name);

        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = instance_->getPlanningSceneDiff();
        planning_scene_diff_client.call(srv);

        ROS_INFO("Removed collision object %s", name.c_str());
    }

    void getLegoBrickName(int task_idx, std::string &brick_name) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
    }

    Object getLegoStart(int task_idx) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());

        geometry_msgs::Pose box_pose = lego_ptr_->get_init_brick_pose(brick_name);
        Object obj = instance_->getObject(brick_name);
        // define the object
        obj.state = Object::State::Static;
        obj.parent_link = "world";

        obj.x = box_pose.position.x;
        obj.y = box_pose.position.y;
        obj.z = box_pose.position.z;
        obj.qx = box_pose.orientation.x;
        obj.qy = box_pose.orientation.y;
        obj.qz = box_pose.orientation.z;
        obj.qw = box_pose.orientation.w;

        return obj;
    }

    Object getLegoTarget(int task_idx) {
        auto cur_graph_node =  task_json_[std::to_string(task_idx)];
        std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());

        Eigen::Matrix4d brick_pose_mtx;
        lego_ptr_->calc_bric_asssemble_pose(brick_name, cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(),
                 cur_graph_node["z"].asInt(), cur_graph_node["ori"].asInt(), brick_pose_mtx);
        
        Object obj = instance_->getObject(brick_name);
        // define the object
        obj.state = Object::State::Static;
        obj.parent_link = "world";

        obj.x = brick_pose_mtx(0, 3);
        obj.y = brick_pose_mtx(1, 3);
        obj.z = brick_pose_mtx(2, 3);
        Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
        obj.qx = quat.x();
        obj.qy = quat.y();
        obj.qz = quat.z();
        obj.qw = quat.w();

        return obj;
    }

    bool setCollision(const std::string& object_id, const std::string& link_name, bool allow) {
        instance_->setCollision(object_id, link_name, allow);

        moveit_msgs::ApplyPlanningScene srv_apply;
        srv_apply.request.scene = instance_->getPlanningSceneDiff();
        planning_scene_diff_client.call(srv_apply);
 
        if (allow)
            ROS_INFO("Allow collision between %s and %s", object_id.c_str(), link_name.c_str());
        else
            ROS_INFO("Disallow collision between %s and %s", object_id.c_str(), link_name.c_str());

        return true;
    }

    void left_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 6; i++) {
            current_joints_[i] = msg->position[i];
        }
        left_arm_joint_state_received = true;

        if (tpg_ != nullptr) {
            tpg_->update_joint_states(msg->position, 0);
        }

    }

    void right_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 6; i++) {
            current_joints_[7+i] = msg->position[i];
        }
        right_arm_joint_state_received = true;

        if (tpg_ != nullptr) {
            tpg_->update_joint_states(msg->position, 1);
        }
        
    }

    void dual_arm_joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 7; i++) {
            current_joints_[i] = msg->position[i];
            current_joints_[7+i] = msg->position[7+i];
        }
        left_arm_joint_state_received = true;
        right_arm_joint_state_received = true;

        std::vector<double> left_joints(msg->position.begin(), msg->position.begin()+7);
        std::vector<double> right_joints(msg->position.begin()+7, msg->position.begin()+14);
        if (tpg_ != nullptr) {
            tpg_->update_joint_states(left_joints, 0);
            tpg_->update_joint_states(right_joints, 1);

        }

    }

    std::shared_ptr<PlanInstance> getInstance() {
        return instance_;
    }

    std::shared_ptr<TPG::TPG> copyCurrentTPG() {
        return std::make_shared<TPG::TPG>(*tpg_);
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
    Json::Value task_json_;
    int num_tasks_ = 0;
    std::set<std::string> lego_collision_objects_;
    moveit_msgs::AllowedCollisionMatrix current_acm_;

    // update gazebo state
    ros::ServiceClient set_state_client_;
    ros::ServiceClient planning_scene_diff_client;
    ros::ServiceClient get_planning_scene_client;

    // joint names for execution
    std::vector<std::string> joint_names_;
    std::vector<std::vector<std::string>> joint_names_split_;
    std::vector<double> current_joints_;
    bool left_arm_joint_state_received = false;
    bool right_arm_joint_state_received = false;
    ros::Subscriber left_arm_sub, right_arm_sub, dual_arm_sub;

    // tpg execution
    std::shared_ptr<TPG::TPG> tpg_;

    bool async_ = false;
    bool mfi_ = false;
    std::vector<std::string> group_names_; // group name for moveit controller services

    std::string planner_type_;
    std::shared_ptr<MoveitInstance> instance_;
    std::shared_ptr<PriorityPlanner> pp_planner_;

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans_;

    std::string output_dir_;
    int counter_ = 0;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "dual_arm_joint_space_planner");
    
    // start ros node
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Read ROS Params
    std::string file_path, planner_type, config_fname, root_pwd, output_dir;
    bool async = false;
    bool mfi = false;
    bool load_tpg = false;
    bool load_adg = false;
    std::vector<std::string> group_names = {"left_arm", "right_arm"};
    for (int i = 0; i < 2; i++) {
        if (nh.hasParam("group_name_" + std::to_string(i))) {
            nh.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }
    nh.getParam("fullorder_targets_filename", file_path);
    nh.getParam("planner_type", planner_type);
    nh.param<std::string>("config_fname", config_fname, "");
    nh.param<std::string>("root_pwd", root_pwd, "");
    nh.param<std::string>("output_dir", output_dir, "");
    nh.param<bool>("async", async, false);
    nh.param<bool>("mfi", mfi, false);
    nh.param<bool>("load_tpg", load_tpg, false);
    nh.param<bool>("load_adg", load_adg, false);

    // Initialize the DUal Arm Planner
    setLogLevel(LogLevel::INFO);

    TPG::TPGConfig tpg_config;
    tpg_config.shortcut_time = 0.1;

    DualArmPlanner planner(planner_type, output_dir, group_names, async, mfi);

    // wait 2 seconds
    ros::Duration(2).sleep();

    // Read the robot poses
    std::vector<std::vector<GoalPose>> all_poses;
    readPosesFromFile(file_path, all_poses);
    ROS_INFO("Read %lu poses", all_poses.size());

    // Read the lego poses
    if (!config_fname.empty() && !root_pwd.empty()) {
        planner.setLegoFactory(config_fname, root_pwd);
        planner.initLegoPositions();
    }

    planner.setup_once();
    ROS_INFO("Execution setup done");

    // Start Execution Loop
    auto adg = std::make_shared<TPG::ADG>(2);
    if (load_adg) {
        // open the file safely
        std::string filename = output_dir + "/adg.txt";
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return -1;
        }
        boost::archive::text_iarchive ia(ifs);
        ia >> adg;
        planner.reset_joint_states_flag();
        planner.execute(adg);

        ros::shutdown();
        return 0;
    }

    std::vector<std::shared_ptr<TPG::TPG>> tpgs;
    int mode = 1;
    int task_idx = 1;
    std::string brick_name;
    std::string last_brick_name = "b2_5"; // TODO: fix this hardcoding
    bool use_robot2 = false;
    for (int i = 0; i < all_poses.size(); i++) {
        std::vector<GoalPose> poses = all_poses[i];

        if (mode == 0 || mode == 6) {
            adg->add_activity(0, TPG::Activity::Type::home);
            adg->add_activity(1, TPG::Activity::Type::home);
            planner.setCollision("table", "left_arm_link_tool", false);
        }
        if (mode == 1) {
            Object obj = planner.getLegoStart(task_idx);
            adg->add_activity(0, TPG::Activity::Type::pick_tilt_up, obj);
            adg->add_activity(1, TPG::Activity::Type::home);
            planner.getLegoBrickName(task_idx, brick_name);
            planner.setCollision(brick_name, "left_arm_link_tool", true);
        }
        if (mode == 2) {
            adg->add_activity(0, TPG::Activity::Type::pick_up);
            adg->add_activity(1, TPG::Activity::Type::home);
        }
        if (mode == 3) {
            adg->add_activity(0, TPG::Activity::Type::pick_down);
            adg->add_activity(1, TPG::Activity::Type::home);
        }
        if (mode == 4) {
            ROS_INFO("attach lego to robot 0");
            adg->add_activity(0, TPG::Activity::Type::pick_twist);
            adg->add_activity(1, TPG::Activity::Type::home);
            planner.attachMoveitCollisionObject(brick_name, 0, "left_arm_link_tool");
            planner.setCollision("table", "left_arm_link_tool", true);
        }
        if (mode == 5) {
            adg->add_activity(0, TPG::Activity::Type::pick_twist_up);
            adg->add_activity(1, TPG::Activity::Type::home);
        }
        if (mode == 7) {
            adg->add_activity(0, TPG::Activity::Type::drop_tilt_up);
            adg->add_activity(1, TPG::Activity::Type::home);
        }
        if (mode == 8) {
            adg->add_activity(0, TPG::Activity::Type::drop_up);
            if (poses[1].use_robot) {
                use_robot2 = true;
                adg->add_activity(1, TPG::Activity::Type::support);
            } else {
                adg->add_activity(1, TPG::Activity::Type::home);
            }
        }
        if (mode == 9) {
            if (use_robot2) {
                adg->add_activity(0, TPG::Activity::Type::drop_down, adg->get_last_activity(1, TPG::Activity::Type::support));
                adg->add_activity(1, TPG::Activity::Type::support);
            } else {
                adg->add_activity(0, TPG::Activity::Type::drop_down);
                adg->add_activity(1, TPG::Activity::Type::home);
            }
            planner.setCollision(last_brick_name, brick_name, true);
            last_brick_name = brick_name;
        }
        if (mode == 10) {
            ROS_INFO("detach lego from robot 0");
            adg->add_activity(0, TPG::Activity::Type::drop_twist);
            if (use_robot2) {
                adg->add_activity(1, TPG::Activity::Type::support);
            } else {
                adg->add_activity(1, TPG::Activity::Type::home);
            }
            planner.detachMoveitCollisionObject(brick_name);
        }
        if (mode == 11) {
            Object obj = planner.getLegoTarget(task_idx);
            adg->add_activity(0, TPG::Activity::Type::drop_twist_up, obj);
            if (use_robot2) {
                adg->add_activity(1, TPG::Activity::Type::support);
            } else {
                adg->add_activity(1, TPG::Activity::Type::home);
            }
        }
        if (mode == 12) {
            adg->add_activity(0, TPG::Activity::Type::home);
            if (poses[1].use_robot) {
                adg->add_activity(1, TPG::Activity::Type::home, adg->get_last_activity(0, TPG::Activity::Type::drop_twist_up));
            } else {
                adg->add_activity(1, TPG::Activity::Type::home);
            }
            planner.setCollision(brick_name, "left_arm_link_tool", false);
        }

        ROS_INFO("mode %d, task_id: %d, use robot 1: %d, use robot 2: %d", mode, task_idx, poses[0].use_robot, poses[1].use_robot);
        if (load_tpg) {
            std::shared_ptr<TPG::TPG> tpg = planner.loadTPG(output_dir + "/tpg_" + std::to_string(i) + ".txt");
            if (tpg != nullptr) {
                planner.set_tpg(tpg);
                planner.reset_joint_states_flag(); // do this after tpg is set
                //planner.execute(tpg);
            }
        }
        else {
            planner.reset_joint_states_flag();
            planner.planAndMove(poses, tpg_config);
        }

        std::shared_ptr<TPG::TPG> tpg = planner.copyCurrentTPG(); 
        tpgs.push_back(tpg);

        mode ++;
        if (mode == 13) {
            mode = 0;
            task_idx ++;
            use_robot2 = false;
        }
    }
    
    // create adg
    adg->init(planner.getInstance(), tpg_config, tpgs);

    // save adg
    ROS_INFO("Saving ADG to file");
    adg->saveADGToDotFile(output_dir + "/adg.dot");
    std::ofstream ofs(output_dir + "/adg.txt");
    if (!ofs.is_open()) {
        ROS_ERROR("Failed to open file: %s", (output_dir + "/adg.txt").c_str());
        return -1;
    }
    boost::archive::text_oarchive oa(ofs);
    oa << adg;

    planner.reset_joint_states_flag();
    planner.execute(adg);

    ros::shutdown();
    return 0;
}
