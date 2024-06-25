#include <fstream>
#include <iostream>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <planner.h>
#include <adg.h>
#include <logger.h>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>

class Planner {
public:
    Planner(const std::string& planning_group, const std::string &planner_type, const std::string &output_dir,
            const std::vector<std::string> &group_names, const std::vector<std::string> &eof_groups, bool fake_move, double vmax) : 
        planner_type_(planner_type), output_dir_(output_dir), group_names_(group_names), eof_groups_(eof_groups), fake_move_(fake_move) {
        
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

        home_poses_.clear();
        std::map<std::string, double> home_pose_map = move_group_->getNamedTargetValues("ready_pose");

        for (int i = 0; i < num_robots_; i++) {
            std::vector<std::string> name_i;
            for (size_t j = 0; j < robot_dofs_[i]; j++) {
                name_i.push_back(joint_names_[i*7+j]);
            }
            joint_names_split_.push_back(name_i);

            RobotPose home_pose = instance_->initRobotPose(i);
            for (size_t j = 0; j < robot_dofs_[i]; j++) {
                home_pose.joint_values[j] = home_pose_map[name_i[j]];
            }
            home_poses_.push_back(home_pose);
        }

        dual_arm_sub = nh_.subscribe("/joint_states", 1, &Planner::joint_state_cb, this);
        
    }

    void reset_joint_states_flag() {
        joint_state_received = false;
        while (!joint_state_received) {
            ros::Duration(0.01).sleep();
        }
    }

    bool saveADG(std::shared_ptr<TPG::ADG> adg, const std::string &filename) {
        std::ofstream ofs(filename);
        if (!ofs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return false;
        }
        boost::archive::text_oarchive oa(ofs);
        oa << adg;
        
        return true;
    }

    std::shared_ptr<TPG::ADG> loadADG(const std::string &filename) {
        // open the file safely
        std::ifstream ifs(filename);
        if (!ifs.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return nullptr;
        }
        std::shared_ptr<TPG::ADG> adg = std::make_shared<TPG::ADG>();
        boost::archive::text_iarchive ia(ifs);
        ia >> adg;
        return adg;
    }

    void addMoveitCollisionObject(const Object &obj) {
        instance_->addMoveableObject(obj);
        instance_->updateScene();

        ROS_INFO("Added collision object %s", obj.name.c_str());
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
        int idx = 0;
        for (int i = 0; i < num_robots_; i++) {
            std::vector<double> robot_i_joints;
            for (size_t j = 0; j < robot_dofs_[i]; j++) {
                robot_i_joints.push_back(msg->position[i*9+j]);
            }
            if (adg_ != nullptr) {
                adg_->update_joint_states(robot_i_joints, i);
            }
        }
        joint_state_received = true;

    }

    std::shared_ptr<PlanInstance> getInstance() {
        return instance_;
    }
    std::shared_ptr<TPG::ADG> getADG() {
        return adg_;
    }

    double getPlanningTime() {
        return planning_time_;
    }

    void init_act_graph(const std::string &config_fname) {
        // read the json file
        std::ifstream config_file(config_fname);
        log("Reading config file: " + config_fname, LogLevel::INFO);
        Json::Value config;
        config_file >> config;

        act_graph_ = std::make_shared<ActivityGraph>(num_robots_);
        
        std::map<std::string, ObjPtr> obj_map;

        const Json::Value objects = config["objects"];
        for (const auto &objectName : objects.getMemberNames()) {
            const Json::Value obj = objects[objectName];
            std::string shape = obj["shape"].asString();
            std::string grip_pose = obj["grip_pose"].asString();
            double x = obj["x"].asDouble();
            double y = obj["y"].asDouble();
            double z = obj["z"].asDouble();

            const Json::Value quat = obj["quat"];
            double qx = quat[0].asDouble();
            double qy = quat[1].asDouble();
            double qz = quat[2].asDouble();
            double qw = quat[3].asDouble();

            Object object(objectName, "base", Object::State::Static, x, y, z, qx, qy, qz, qw);
            if (shape == "box") {
                object.shape = Object::Shape::Box;
                object.length = obj["length"].asDouble();;
                object.width = obj["width"].asDouble();;
                object.height = obj["height"].asDouble();;
            } else if (shape == "cylinder") {
                object.shape = Object::Shape::Cylinder;
                object.length = obj["length"].asDouble();;
                object.radius = obj["radius"].asDouble();;
            

                // compute grip pose for the robot in world coordinate
                if (grip_pose == "top") {
                    object.x_attach = 0;
                    object.y_attach = 0;
                    object.z_attach = object.length/2;
                    object.qx_attach = 1;
                    object.qy_attach = 0;
                    object.qz_attach = 0;
                    object.qw_attach = 0;
                }
                else if (grip_pose == "left") {
                    object.x_attach = 0;
                    object.y_attach = object.radius;
                    object.z_attach = 0;
                    object.qx_attach = 0.5;
                    object.qy_attach = -0.5;
                    object.qz_attach = 0.5;
                    object.qw_attach = 0.5;
                }
                else if (grip_pose == "right") {
                    object.x_attach = 0;
                    object.y_attach = -object.radius;
                    object.z_attach = 0;
                    object.qx_attach = 0.5;
                    object.qy_attach = 0.5;
                    object.qz_attach = 0.5;
                    object.qw_attach = -0.5;
                }
            }


            ObjPtr obj_p = act_graph_->add_obj(object);
            obj_map[objectName] = obj_p;

            std::cout << "Object: " << objectName << ", Shape: " << shape 
                    << ", Position: (" << x << ", " << y << ", " << z << ")"
                    << ", Quaternion: (" << qx << ", " << qy << ", " << qz << ", " << qw << ")"
                    << ", Grip Pose: " << grip_pose << std::endl;
        }

        std::map<std::string, ObjPtr> goalobj_map;

        // Access goal
        const Json::Value goal = config["goal"];
        for (const auto &goalName : goal.getMemberNames()) {
            const Json::Value obj = goal[goalName];
            double x = obj["x"].asDouble();
            double y = obj["y"].asDouble();
            double z = obj["z"].asDouble();

            const Json::Value quat = obj["quat"];
            double qx = quat[0].asDouble();
            double qy = quat[1].asDouble();
            double qz = quat[2].asDouble();
            double qw = quat[3].asDouble();

            Object goal_obj(goalName, "base", Object::State::Static, x, y, z, qx, qy, qz, qw);
            goal_obj.x_attach = obj_map[goalName]->obj.x_attach;
            goal_obj.y_attach = obj_map[goalName]->obj.y_attach;
            goal_obj.z_attach = obj_map[goalName]->obj.z_attach;
            goal_obj.qx_attach = obj_map[goalName]->obj.qx_attach;
            goal_obj.qy_attach = obj_map[goalName]->obj.qy_attach;
            goal_obj.qz_attach = obj_map[goalName]->obj.qz_attach;
            goal_obj.qw_attach = obj_map[goalName]->obj.qw_attach;
            ObjPtr obj_p = act_graph_->add_obj(goal_obj);
            goalobj_map[goalName] = obj_p;

            std::cout << "Goal: " << goalName 
                    << ", Position: (" << x << ", " << y << ", " << z << ")"
                    << ", Quaternion: (" << qx << ", " << qy << ", " << qz << ", " << qw << ")" << std::endl;
        }

        // Access sequence
        const Json::Value sequence = config["sequence"];

        // For each robot arm sequence
        for (const auto &robotArm : sequence.getMemberNames()) {
            int robot_id = robotArm[5] - '0';
            std::string eof_group = eof_groups_[robot_id];
            const std::vector<std::string>& ee_links = robot_model_->getJointModelGroup(eof_groups_[robot_id])->getLinkModelNames();
           
            const Json::Value tasks = sequence[robotArm];
            if (tasks.isArray()) {
                for (const auto &task : tasks) {
                    std::string taskType = task["task"].asString();
                    std::string obj = task["obj"].asString();
                    int id = task["id"].asInt();
                    std::shared_ptr<Activity> act;

                    if (taskType == "pick_pre") {
                        act = act_graph_->add_act(robot_id, Activity::Type::pick_up);

                        // generate approach pose
                        Eigen::Matrix4d approach_pose;
                        act->end_pose = instance_->initRobotPose(robot_id);

                        generate_approach_pose(obj_map[obj]->obj, robot_id, approach_pose);
                        solveIK(approach_pose, robot_id, act->end_pose);
                        log(act->end_pose, LogLevel::INFO);
                    }
                    else if (taskType == "pick") {
                        act = act_graph_->add_act(robot_id, Activity::Type::pick_down);
                        
                        for (const auto &link_name : ee_links) {
                            std::cout << "Setting collision for " << obj << " and " << link_name << std::endl;
                            act_graph_->set_collision(obj, link_name, act, true);
                        }

                        // generate grasp pose
                        Eigen::Matrix4d grasp_pose;
                        act->end_pose = instance_->initRobotPose(robot_id);

                        generate_grasp_pose(obj_map[obj]->obj, robot_id, grasp_pose);
                        solveIK(grasp_pose, robot_id, act->end_pose);
                        log(act->end_pose, LogLevel::INFO);

                    } else if (taskType == "place") {
                        act = act_graph_->add_act(robot_id, Activity::Type::drop_down);
                        act_graph_->attach_obj(obj_map[obj], ee_links.front(), act);
                        // generate place pose
                        Eigen::Matrix4d place_pose;
                        act->end_pose = instance_->initRobotPose(robot_id);
                        generate_grasp_pose(goalobj_map[obj]->obj, robot_id, place_pose);
                        solveIK(place_pose, robot_id, act->end_pose);
                        log(act->end_pose, LogLevel::INFO);
                    } 
                    else if (taskType == "place_post") {
                        act = act_graph_->add_act(robot_id, Activity::Type::drop_up);
                        act_graph_->detach_obj(goalobj_map[obj], act);

                        // generate release pose
                        Eigen::Matrix4d release_pose;
                        act->end_pose = instance_->initRobotPose(robot_id);
                        generate_approach_pose(goalobj_map[obj]->obj, robot_id, release_pose);
                        solveIK(release_pose, robot_id, act->end_pose);
                        log(act->end_pose, LogLevel::INFO);
                        
                    }
                    else if (taskType == "home") {
                        act = act_graph_->add_act(robot_id, Activity::Type::home);
                        for (const auto &link_name : ee_links) {
                            std::cout << "Setting collision for " << obj << " and " << link_name << std::endl;
                            act_graph_->set_collision(obj, link_name, act, false);
                        }

                        // set to home pose
                        act->end_pose = home_poses_[robot_id];
                    }
                    act_map[id] = act;
                    // set the start pose
                    if (act->type1_prev != nullptr) {
                        act->start_pose = act->type1_prev->end_pose;
                    } else {
                        act->start_pose = home_poses_[robot_id];
                    }

                    // set other robot activity graph
                    for (int i = 0; i < num_robots_; i++) {
                        if (i != robot_id) {
                            act = act_graph_->add_act(i, Activity::Type::home);
                            act->start_pose = home_poses_[i];
                            act->end_pose = home_poses_[i];
                        }
                    }

                    std::cout << "Robot Arm: " << robotArm 
                            << ", Task: " << taskType 
                            << ", Object: " << obj 
                            << ", ID: " << id << std::endl;
                }
            }
        }

        // Access task dependencies
        const Json::Value taskDeps = sequence["task_dep"];
        for (const auto &dep : taskDeps) {
            int task = dep["task"].asInt();
            const Json::Value deps = dep["deps"];
            std::cout << "Task Dependency: Task " << task << " depends on ";
            for (const auto &d : deps) {
                int dependency = d.asInt();
                act_map[task]->add_type2_dep(act_map[dependency]);
                act_map[dependency]->add_type2_next(act_map[task]);
                std::cout << "Task " << dependency << ", ";
            }
            
            std::cout << std::endl;
        }
        
        act_graph_->saveGraphToFile(output_dir_ + "/act_graph.dot");
        
    }

    void add_initial_objs() {
        std::vector<ObjPtr> obj_nodes;
        
        obj_nodes = act_graph_->get_start_obj_nodes();
        
        for (auto &obj : obj_nodes) {
            instance_->addMoveableObject(obj->obj);
            instance_->updateScene();
            
            log("Added object: " + obj->obj.name + " at (" + std::to_string(obj->obj.x) + ", " + std::to_string(obj->obj.y) + ", "
                 + std::to_string(obj->obj.z) + ")", LogLevel::INFO);
        }

    }

    void generate_grasp_pose(const Object &obj, int robot_id, Eigen::Matrix4d &grasp_pose) {
        // grasp_pose = global_pose * offset * grasp_orientation
        Eigen::Matrix4d obj_pose = Eigen::Matrix4d::Identity();
        obj_pose.block<3, 1>(0, 3) = Eigen::Vector3d(obj.x, obj.y, obj.z);
        obj_pose.block<3, 3>(0, 0) = Eigen::Quaterniond(obj.qw, obj.qx, obj.qy, obj.qz).toRotationMatrix();

        Eigen::Matrix4d obj_offset = Eigen::Matrix4d::Identity();
        obj_offset.block<3, 1>(0, 3) = Eigen::Vector3d(obj.x_attach, obj.y_attach, obj.z_attach);
        obj_offset.block<3, 3>(0, 0) = Eigen::Quaterniond(obj.qw_attach, obj.qx_attach, obj.qy_attach, obj.qz_attach).toRotationMatrix();

        Eigen::Matrix4d grasp_offset = Eigen::Matrix4d::Identity();
        grasp_offset.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, -0.1);

        grasp_pose = obj_pose * obj_offset * grasp_offset;
    }

    void generate_approach_pose(const Object &obj, int robot_id, Eigen::Matrix4d &approach_pose) {
        // approach_pose = global_pose * offset * approach_orientation
        // grasp_pose = global_pose * offset * grasp_orientation
        Eigen::Matrix4d obj_pose = Eigen::Matrix4d::Identity();
        obj_pose.block<3, 1>(0, 3) = Eigen::Vector3d(obj.x, obj.y, obj.z);
        obj_pose.block<3, 3>(0, 0) = Eigen::Quaterniond(obj.qw, obj.qx, obj.qy, obj.qz).toRotationMatrix();

        Eigen::Matrix4d obj_offset = Eigen::Matrix4d::Identity();
        obj_offset.block<3, 1>(0, 3) = Eigen::Vector3d(obj.x_attach, obj.y_attach, obj.z_attach);
        obj_offset.block<3, 3>(0, 0) = Eigen::Quaterniond(obj.qw_attach, obj.qx_attach, obj.qy_attach, obj.qz_attach).toRotationMatrix();

        Eigen::Matrix4d app_offset = Eigen::Matrix4d::Identity();
        app_offset.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, -0.15);

        approach_pose = obj_pose * obj_offset * app_offset;
    }

    bool solveIK(const Eigen::Matrix4d &task_pose, int robot_id, RobotPose &joint_pose) {
        
        // solve ik for the robot
        std::string eef_link = eof_groups_[robot_id];
        std::string group_name = group_names_[robot_id];
        const robot_state::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(group_name);

        // Convert Pose to Eigen
        Eigen::Isometry3d target_pose_eigen;
        target_pose_eigen.matrix() = task_pose;

        moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
        
        robot_state.setToDefaultValues();
        bool found_ik = robot_state.setFromIK(joint_model_group, target_pose_eigen, eef_link);

        std::cout << "Task pose: " << robot_id << " "<< task_pose << std::endl;
        if (found_ik) {
            robot_state.copyJointGroupPositions(joint_model_group, joint_pose.joint_values);
            return true;
        } else {
            ROS_ERROR("Failed to find IK solution for end effector: %s", eef_link.c_str());
            return false;
        }
    }

    bool sequential_plan(const TPG::TPGConfig &tpg_config, double planning_time_limit) {
        // do planning sequentially
        int num_tasks = act_map.size();
        bool success = true;

        // reset the scene
        instance_->resetScene(true);
        for (int i = 0; i < num_robots_; i++) {
            instance_->moveRobot(i, home_poses_[i]);
            instance_->updateScene();
        }
        
        // add the objects to the scene
        add_initial_objs();

        std::vector<std::shared_ptr<TPG::TPG>> tpgs;

        for (int t = 0; t < num_tasks; t++) {
            auto act = act_map[t];
            log("Executing task " + std::to_string(t) + " ( " + std::to_string(act->robot_id) + " , " 
                + act->type_string() + " )", LogLevel::INFO);

            // set the collision geometry and attached objects;
            for (auto &obj : act->obj_attached) {
                instance_->attachObjectToRobot(obj->obj.name, act->robot_id, obj->next_attach_link, act->start_pose);
                std::cout << "Attaching object " << obj->obj.name << " to " << obj->next_attach_link << std::endl;
                instance_->updateScene();
            }
            for (auto &obj : act->obj_detached) {
                instance_->detachObjectFromRobot(obj->obj.name, act->start_pose);
                std::cout << "Detaching object " << obj->obj.name << std::endl;
                instance_->updateScene();
            }
            for (auto &col : act->collision_nodes) {
                instance_->setCollision(col.obj_name, col.link_name, col.allow);
                std::cout << "Setting collision for " << col.obj_name << " and " << col.link_name << std::endl;
                instance_->updateScene();
            }

            // set start and goal poses
            instance_->setStartPose(act->robot_id, act->start_pose.joint_values);
            instance_->setGoalPose(act->robot_id, act->end_pose.joint_values);
            
            std::vector<RobotTrajectory> resting_robots;
            for (int i = 0; i < num_robots_; i++) {
                if (i != act->robot_id) {
                    RobotTrajectory rest;
                    rest.robot_id = i;
                    rest.cost = 0;
                    rest.times = {0.0};
                    rest.trajectory = {home_poses_[i]};
                    resting_robots.push_back(rest);
                }
            }

            RobotTrajectory solution;
            PlannerOptions options;
            options.max_planning_time = planning_time_limit;
            options.obstacles = resting_robots;
            options.max_planning_iterations = 1000000;
            segment_plan_rrt(options, act->robot_id, instance_, solution);

            std::vector<RobotTrajectory> solution_vec = resting_robots;
            solution_vec.insert(solution_vec.begin() + act->robot_id, solution);

            // build tpg
            std::shared_ptr<TPG::TPG> tpg = std::make_shared<TPG::TPG>();
            tpg->init(instance_, solution_vec, tpg_config);
            //tpg->moveit_execute(instance_, move_group_);
            tpgs.push_back(tpg);

            instance_->moveRobot(act->robot_id, act->end_pose);
            instance_->updateScene();
            
        }

        adg_ = std::make_shared<TPG::ADG>(*act_graph_);
        adg_->init_from_tpgs(instance_, tpg_config, tpgs);
        adg_->optimize(instance_, tpg_config);
        adg_->saveToDotFile(output_dir_ + "/adg.dot");

        return success;
    }
    
    bool execute_plan() {
        int num_tasks = act_map.size();
        bool success = true;

        // reset the scene
        instance_->resetScene(true);
        
        // add the objects to the scene
        add_initial_objs();

        reset_joint_states_flag();
        success &= adg_->moveit_execute(instance_, move_group_);

        return success;
    }

    bool segment_plan_strrt(const PlannerOptions &options, int robot_id, std::shared_ptr<PlanInstance> inst, RobotTrajectory &solution)
    {
        SingleAgentPlannerPtr planner;
        planner = std::make_shared<STRRT>(inst, robot_id);
        bool success = planner->plan(options);
        if (!success) {
            ROS_ERROR("Failed to plan for robot %d", robot_id);
        }
        success &= planner->getPlan(solution);

        return success;
    }

    bool segment_plan_rrt(const PlannerOptions &options, int robot_id, std::shared_ptr<PlanInstance> inst, RobotTrajectory &solution)
    {
        planning_interface::MotionPlanRequest req;
        req.group_name = group_names_[robot_id];
        req.allowed_planning_time = options.max_planning_time;
        req.planner_id = "RRTConnect";

        // Set the start state
        req.start_state.joint_state.name = joint_names_split_[robot_id];
        req.start_state.joint_state.position = inst->getStartPose(robot_id).joint_values;

        // add attached object
        std::vector<Object> attached_objs = inst->getAttachedObjects(robot_id);
        for (auto &obj : attached_objs) {
            moveit_msgs::AttachedCollisionObject co;
            co.link_name = obj.parent_link;
            co.object.id = obj.name;
            co.object.header.frame_id = obj.parent_link;
            co.object.operation = co.object.ADD;
            req.start_state.attached_collision_objects.push_back(co);
        }

        // set the goal state
        robot_state::RobotState goal_state(robot_model_);
        const robot_model::JointModelGroup* joint_model_group = robot_model_->getJointModelGroup(group_names_[robot_id]);
        goal_state.setJointGroupPositions(joint_model_group, inst->getGoalPose(robot_id).joint_values);
        moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

        req.goal_constraints.clear();
        req.goal_constraints.push_back(joint_goal);

        planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model_, nh_, "planning_plugin", "request_adapters"));

        planning_interface::MotionPlanResponse res;
        planning_pipeline->generatePlan(planning_scene_, req, res);

        if (res.error_code_.val != res.error_code_.SUCCESS)
        {
            ROS_ERROR("Could not compute plan successfully");
            return false;
        }

        moveit_msgs::RobotTrajectory plan_traj;
        res.trajectory_->getRobotTrajectoryMsg(plan_traj);
        convertSolution(inst, plan_traj, robot_id, solution);
        return true;
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
    std::vector<std::string> eof_groups_; // group name for end effector

    ros::ServiceClient planning_scene_diff_client;

    // joint names for execution
    std::vector<std::string> joint_names_;
    std::vector<std::vector<std::string>> joint_names_split_;
    std::vector<RobotPose> home_poses_;
    bool joint_state_received = false;
    ros::Subscriber left_arm_sub, right_arm_sub, dual_arm_sub;

    // tpg execution
    std::shared_ptr<TPG::ADG> adg_;

    bool fake_move_ = true;

    std::string planner_type_;
    std::shared_ptr<MoveitInstance> instance_;
    std::shared_ptr<PriorityPlanner> pp_planner_;
    std::shared_ptr<ActivityGraph> act_graph_;
    std::map<int, std::shared_ptr<Activity>> act_map;

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
    group_names.resize(num_robots);
    for (int i = 0; i < num_robots; i++) {
        if (nh.hasParam("group_name_" + std::to_string(i))) {
            nh.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }
    nh.param<std::string>("planning_group", planning_group, "panda_multi_arm");
    std::vector<std::string> eof_groups = {"panda0_hand", "panda1_hand", "panda2_hand"};
    eof_groups.resize(num_robots);
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

    Planner planner(planning_group, planner_type, output_dir, group_names, eof_groups, fake_move, vmax);

    // wait 2 seconds
    ros::Duration(2).sleep();
    planner.setup_once();
    ROS_INFO("Execution setup done");

    
    planner.init_act_graph(config_fname);
    planner.sequential_plan(tpg_config, 2.0);

    planner.saveADG(planner.getADG(), output_dir + "/adg.txt");
    ROS_INFO("ADG saved to file");
    planner.execute_plan();

    ROS_INFO("Planning completed successfully");
    ros::shutdown();
    return 0;
}
