#include <fstream>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <planner.h>
#include <adg.h>
#include <logger.h>


class Planner {
public:
    Planner(const std::string& planning_group, const std::string &planner_type, const std::string &output_dir,
            const std::vector<std::string> &group_names, bool fake_move, double vmax) : 
        planner_type_(planner_type), output_dir_(output_dir), group_names_(group_names), fake_move_(fake_move) {
        
        num_robots_ = group_names.size();

        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model_ = robot_model_loader.getModel();
        
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(planning_group);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        kinematic_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
        kinematic_state_->setToDefaultValues();
    
        ros::Duration(0.3).sleep();
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

        planning_scene_diff_client = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();

        instance_ = std::make_shared<MoveitInstance>(kinematic_state_, move_group_->getName(), planning_scene_);
        instance_->setNumberOfRobots(num_robots_);
        instance_->setRobotNames(group_names);
        robot_dofs_.resize(num_robots_);
        for (size_t i = 0; i < num_robots_; i++) {
            int dof = robot_model_->getJointModelGroup(group_names[i])->getActiveVariableCount();
            instance_->setRobotDOF(i, dof);
            robot_dofs_[i] = dof;
        }
        instance_->setVmax(vmax);
        instance_->setPlanningSceneDiffClient(planning_scene_diff_client);
        pp_planner_ = std::make_shared<PriorityPlanner>(instance_);

    }


    bool planJointSpace(const std::vector<RobotPose>& goal_poses, const std::vector<double>& joint_values,
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
        joint_state_received = false;

        // create a directory for saving TPGs if it does not exist
        if (!boost::filesystem::exists(output_dir_)) {
            boost::filesystem::create_directories(output_dir_);
        }

        current_joints_.clear();
        for (int i = 0; i < num_robots_; i++) {
            std::vector<std::string> name_i;
            for (size_t j = 0; j < robot_dofs_[i]; j++) {
                name_i.push_back(joint_names_[i*7+j]);
                current_joints_.push_back(0);
            }
            joint_names_split_.push_back(name_i);
        }
        dual_arm_sub = nh_.subscribe("/joint_states", 1, &Planner::joint_state_cb, this);
        
    }

    bool priority_plan(const std::vector<RobotPose>& goal_poses, std::vector<RobotTrajectory>& solution) {
        bool success = true;
        size_t dof_s = 0;
        for (size_t i = 0; i < num_robots_; i++) {
            std::vector<double> start_pose(robot_dofs_[i], 0.0);
            for (size_t j = 0; j < robot_dofs_[i]; j++) {
                start_pose[j] = current_joints_[dof_s + j];
            }
            dof_s += robot_dofs_[i];

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
        auto start_state = planning_scene_->getCurrentStateNonConst();
        start_state.setJointGroupPositions(move_group_->getName(), current_joints_);
        move_group_->setStartState(start_state);
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
        joint_state_received = false;
        while (!joint_state_received) {
            ros::Duration(0.01).sleep();
        }
    }

    bool fake_execute(std::shared_ptr<TPG::TPG> tpg) {

        int dof_s = 0;
        for (int i = 0; i < num_robots_; i++) {
            const RobotPose &pose = tpg->getEndNode(i)->pose;
            instance_->moveRobot(i, pose);
            instance_->updateScene();

            for (size_t j = 0; j < robot_dofs_[i]; j++) {
                current_joints_[dof_s+j] = pose.joint_values[j];
            }
            dof_s += robot_dofs_[i];
        }

        return true;
    }

    bool execute(std::shared_ptr<TPG::TPG> tpg) {
        
        bool success = true;
        success &= tpg->moveit_execute(instance_, move_group_);
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

    bool planAndMove(const std::vector<RobotPose>& poses, const TPG::TPGConfig &tpg_config) {
        if (fake_move_) {
            dual_arm_sub.shutdown();
        }
        else {
            reset_joint_states_flag();
        }

        auto t_start = std::chrono::high_resolution_clock::now();
        std::vector<double> all_joints;
        std::vector<double> all_joints_given;
        for (auto pose : poses) {
            all_joints_given.insert(all_joints_given.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        std::vector<RobotTrajectory> solution;
        bool success = planJointSpace(poses, all_joints_given, solution);
        int t_plan_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();
        planning_time_ += (t_plan_ms * 0.001);

        if (success) {
            if (tpg_ == nullptr) {
                tpg_ = std::make_shared<TPG::TPG>();
            }
            tpg_->reset();
            
            success &= tpg_->init(instance_, solution, tpg_config);
            //success &= tpg_->optimize(instance_, tpg_config);
            saveTPG(tpg_, output_dir_ + "/tpg_" + std::to_string(counter_) + ".txt");
            tpg_->saveToDotFile(output_dir_ + "/tpg_" + std::to_string(counter_) + ".dot");

            if (fake_move_) {
                success &= fake_execute(tpg_);
            } else {
                success &= execute(tpg_);
            }
        }
        counter_++;
        return success;
    }

    void addMoveitCollisionObject(const std::string &name) {
    }

    void attachMoveitCollisionObject(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) {
        instance_->attachObjectToRobot(name, robot_id, link_name, pose);
        instance_->updateScene();

        ROS_INFO("Attached collision object %s to %s", name.c_str(), link_name.c_str());
    }

    void detachMoveitCollisionObject(const std::string &name, const RobotPose &pose) {
        instance_->detachObjectFromRobot(name, pose);
        instance_->updateScene();

        ROS_INFO("Removed collision object %s", name.c_str());
    }

    bool setCollision(const std::string& object_id, const std::string& link_name, bool allow) {
        instance_->setCollision(object_id, link_name, allow);
        instance_->updateScene();

        if (allow)
            ROS_INFO("Allow collision between %s and %s", object_id.c_str(), link_name.c_str());
        else
            ROS_INFO("Disallow collision between %s and %s", object_id.c_str(), link_name.c_str());

        return true;
    }

    void joint_state_cb(const sensor_msgs::JointState::ConstPtr& msg) {
        for (size_t i = 0; i < 7; i++) {
            current_joints_[i] = msg->position[i];
            current_joints_[7+i] = msg->position[7+i];
        }
        joint_state_received = true;

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

    std::vector<double> getCurrentJoints() {
        return current_joints_;
    }

    double getPlanningTime() {
        return planning_time_;
    }

    void read_act_graph(const std::string &filename) {
    }


private:
    ros::NodeHandle nh_;
    
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene::PlanningScenePtr planning_scene_;

    int num_robots_;
    std::vector<int> robot_dofs_;
    std::vector<std::string> group_names_; // group name for moveit controller services

    ros::ServiceClient planning_scene_diff_client;

    // joint names for execution
    std::vector<std::string> joint_names_;
    std::vector<std::vector<std::string>> joint_names_split_;
    std::vector<double> current_joints_;
    bool joint_state_received = false;
    ros::Subscriber left_arm_sub, right_arm_sub, dual_arm_sub;

    // tpg execution
    std::shared_ptr<TPG::TPG> tpg_;

    bool fake_move_ = true;

    std::string planner_type_;
    std::shared_ptr<MoveitInstance> instance_;
    std::shared_ptr<PriorityPlanner> pp_planner_;

    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> plans_;

    std::string output_dir_;
    int counter_ = 0;
    double planning_time_ = 0.0;
};



int main(int argc, char** argv) {
    ros::init(argc, argv, "plan_node");
    
    // start ros node
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Read ROS Params
    std::string planner_type, config_fname, output_dir, planning_group;
    bool load_tpg = false;
    bool load_adg = false;
    bool benchmark = false;
    bool fake_move = false;
    double vmax = 1.0;
    int num_robots = 3;
    std::vector<std::string> group_names = {"panda0_arm", "panda1_arm", "panda2_arm"};
    nh.param<int>("num_robots", num_robots, 2);
    for (int i = 0; i < num_robots; i++) {
        if (nh.hasParam("group_name_" + std::to_string(i))) {
            nh.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }
    nh.param<std::string>("planning_group", planning_group, "panda_multi_arm");
    std::vector<std::string> eof_groups = {"panda0_hand", "panda1_hand", "panda2_hand"};
    for (int i = 0; i < num_robots; i++) {
        if (nh.hasParam("eof_group_" + std::to_string(i))) {
            nh.getParam("eof_group_" + std::to_string(i), eof_groups[i]);
        }
    }

    nh.getParam("config_fname", config_fname);
    nh.param<std::string>("planner_type", planner_type, "RRTConnect");
    nh.param<std::string>("output_dir", output_dir, "");
   
    nh.param<bool>("load_tpg", load_tpg, false);
    nh.param<bool>("load_adg", load_adg, false);
    nh.param<bool>("benchmark", benchmark, false);
    nh.param<bool>("fake_move", fake_move, false);
    if (load_tpg || load_adg) {
        fake_move = false;
    }

    // Initialize the Dual Arm Planner
    setLogLevel(LogLevel::INFO);

    TPG::TPGConfig tpg_config;
    nh.param<double>("shortcut_time", tpg_config.shortcut_time, 0.1);
    nh.param<bool>("tight_shortcut", tpg_config.tight_shortcut, false);
    nh.param<int>("seed", tpg_config.seed, 1);
    nh.param<std::string>("progress_file", tpg_config.progress_file, "");
    nh.param<double>("vmax", vmax, 1.0);
    tpg_config.dt = 0.1 / vmax;
    ROS_INFO("TPG Config: vmax: %f, dt: %f, shortcut_time: %f, tight_shortcut: %d", vmax, tpg_config.dt, tpg_config.shortcut_time, tpg_config.tight_shortcut);

    Planner planner(planning_group, planner_type, output_dir, group_names, fake_move, vmax);

    // wait 2 seconds
    ros::Duration(2).sleep();
    planner.setup_once();
    ROS_INFO("Execution setup done");

    
    planner.read_act_graph(config_fname);
    

    ros::shutdown();
    return 0;
}
