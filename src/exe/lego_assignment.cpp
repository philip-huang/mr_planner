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
#include <lego/Lego.hpp>

const std::string PLANNING_GROUP = "dual_arms";
 

class TaskAssignment {
public:
    TaskAssignment(const std::string &output_dir,
                const std::vector<std::string> &group_names) : 
        output_dir_(output_dir), group_names_(group_names) {
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model_ = robot_model_loader.getModel();
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(PLANNING_GROUP);
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
        kinematic_state_ = std::make_shared<robot_state::RobotState>(robot_model_);
        kinematic_state_->setToDefaultValues();

        ros::Duration(0.3).sleep();
        planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);

        planning_scene_diff_client = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
        planning_scene_diff_client.waitForExistence();

        instance_ = std::make_shared<MoveitInstance>(kinematic_state_, move_group_->getName(), planning_scene_);
        instance_->setNumberOfRobots(2);
        instance_->setRobotNames({"left_arm", "right_arm"});
        instance_->setRobotDOF(0, 7);
        instance_->setRobotDOF(1, 7);
        instance_->setPlanningSceneDiffClient(planning_scene_diff_client);
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
        instance_->setObjectColor("table", 1.0, 1.0, 1.0, 1.0);
        for (const auto & name : brick_names) {
            addMoveitCollisionObject(name);
        }

        return true;

    }

    void addMoveitCollisionObject(const std::string &name) {
        Object obj = getLegoStart(name);

        // add the object to the planning scene
        instance_->addMoveableObject(obj);
        instance_->updateScene();
        
        ROS_INFO("Adding collision object %s at %f %f %f", name.c_str(), obj.x, obj.y, obj.z);
        ROS_INFO("Added collision object %s to world frame", name.c_str());
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

    void getLegoBrickName(int task_idx, std::string &brick_name) {
        auto cur_graph_node = task_json_[std::to_string(task_idx)];
        brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());
    }

    Object getLegoStart(const std::string &brick_name) {
        Object obj;
        
        // define the object
        obj.name = brick_name;
        obj.state = Object::State::Static;
        obj.parent_link = "world";
        obj.shape = Object::Shape::Box;

        // get the starting pose and size
        geometry_msgs::Pose box_pose;
        if (brick_name == "table") {
            lego_ptr_->get_table_size(obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_table_pose();

        } else {
            lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);
            box_pose = lego_ptr_->get_init_brick_pose(brick_name);
        }
        obj.x = box_pose.position.x;
        obj.y = box_pose.position.y;
        obj.z = box_pose.position.z - obj.height/2;
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
        lego_ptr_->get_brick_sizes(brick_name, obj.length, obj.width, obj.height);

        obj.x = brick_pose_mtx(0, 3);
        obj.y = brick_pose_mtx(1, 3);
        obj.z = brick_pose_mtx(2, 3) - obj.height/2;
        Eigen::Quaterniond quat(brick_pose_mtx.block<3, 3>(0, 0));
        obj.qx = quat.x();
        obj.qy = quat.y();
        obj.qz = quat.z();
        obj.qw = quat.w();

        return obj;
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

    void get_brick_corners(const Object &obj, double &lx, double &ly, double &rx, double &ry) {
        double yaw = atan2(2.0*(obj.qx*obj.qy + obj.qw*obj.qz), obj.qw*obj.qw + obj.qx*obj.qx - obj.qy*obj.qy - obj.qz*obj.qz);
        yaw = yaw * 180.0 / M_PI;
        double eps = 5;
        if ((std::abs(yaw - 90) < eps) || (std::abs(yaw + 90) < eps)) {
            lx = obj.x - obj.width/2;
            rx = obj.x + obj.width/2;
            ly = obj.y - obj.length/2;
            ry = obj.y + obj.length/2;   
        } else {
            lx = obj.x - obj.length/2;
            rx = obj.x + obj.length/2;
            ly = obj.y - obj.width/2;
            ry = obj.y + obj.width/2;
        }
    }

    bool brick_overlap(const Object &obj1, const Object &obj2) {
        double lx1, ly1, rx1, ry1, lx2, ly2, rx2, ry2;
        get_brick_corners(obj1, lx1, ly1, rx1, ry1);
        get_brick_corners(obj2, lx2, ly2, rx2, ry2);

        if (lx1 > rx2 || lx2 > rx1) {
            return false;
        }
        if (ly1 > ry2 || ly2 > ry1) {
            return false;
        }

        return true;
    }

    void getLegoBottom(const std::string &brick_name, int task_idx, bool target_pose, std::vector<std::string> &bot_objects) {
        // loop over all blocks, check if it is in previous task.
        // if yes, use target position, otherwise use initial position
        bot_objects.clear();

        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        Object cur_obj;
        if (target_pose) {
            cur_obj = getLegoTarget(task_idx);
        } else {
            cur_obj = getLegoStart(brick_name);
        }

        std::map<std::string, int> moved_bricks;
        for (int tid = 1; tid < task_idx; tid++) {
            std::string brick_name;
            getLegoBrickName(tid, brick_name);
            moved_bricks[brick_name] = tid;
        }

        for (const auto & name : brick_names) {
            if (name == brick_name) {
                continue;
            }
            Object obj;
            auto it = moved_bricks.find(name);
            if (it != moved_bricks.end()) {
                obj = getLegoTarget(it->second);
            } else {
                obj = getLegoStart(name);
            }
            
            if (std::abs(cur_obj.z - cur_obj.height/2 - obj.z - obj.height/2) > 0.002) {
                continue;
            }
            if (brick_overlap(cur_obj, obj)) {
                bot_objects.push_back(name);
            }
        }
        
        if (bot_objects.size() == 0) {
            bot_objects.push_back("table");
        }

        for (const auto & obj : bot_objects) {
            ROS_INFO("Bottom object: %s", obj.c_str());
        }
        return;
    }

    void calculateIKforLego(const Eigen::MatrixXd& T, const Eigen::MatrixXd & home_q,
            int robot_id, bool check_collision, lego_manipulation::math::VectorJd& joint_q, bool &reachable) {
        
        if (robot_id == 0) {
            joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5);
        }
        else {
            joint_q = lego_manipulation::math::IK(home_q, T.block(0, 3, 3, 1), T.block(0, 0, 3, 3),
                                                        lego_ptr_->robot_DH_tool_r2(), lego_ptr_->robot_base_r2(), 0, 10e4, 10e-4*5);
        }

        reachable = (joint_q - home_q).norm() > 1e-6;

        if (check_collision && reachable) {
            RobotPose pose = instance_->initRobotPose(robot_id);
            for (int i = 0; i < 6; i++) {
                pose.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
            }
            bool hasCollision = instance_->checkCollision({pose}, false, true);
            reachable &= !hasCollision;
        }
    }

    void visualize_robot_pose(const lego_manipulation::math::VectorJd &joint_q, int robot_id) {
        RobotPose pose = instance_->initRobotPose(robot_id);
        for (int i = 0; i < 6; i++) {
            pose.joint_values[i] = joint_q(i, 0) / 180.0 * M_PI;
        }
        instance_->moveRobot(robot_id, pose);
        instance_->updateScene();
    }

    void calculateCostMatrix() {
        lego_manipulation::math::VectorJd r1_home = Eigen::MatrixXd::Zero(6, 1);
        r1_home(1, 0) = -15.456043;
        r1_home(2, 0) = -40.3566;
        r1_home(4, 0) = -65.09948;
        
        // copy the value of r1_home to r2_home
        lego_manipulation::math::VectorJd r2_home = r1_home;

        // initialize home pose
        Eigen::MatrixXd home_q(lego_ptr_->robot_dof_1(), 1);
        home_q.col(0) << 0, 0, 0, 0, -90, 0;
        Eigen::Matrix4d home_T = lego_manipulation::math::FK(home_q, lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), false);
        home_T.col(3) << 0.2, 0, 0.4, 1; // Home X, Y, Z in base frame of the Flange
        home_T = lego_ptr_->world_base_frame() * home_T;
        home_q = lego_manipulation::math::IK(home_q, home_T.block(0, 3, 3, 1), home_T.block(0, 0, 3, 3),
                                             lego_ptr_->robot_DH_r1(), lego_ptr_->robot_base_r1(), 0, 10e4, 10e-4*5); // Home
        
        // initialize support_t1, support_t2
        Eigen::Matrix4d y_n90, y_s90, z_180;
        y_n90 << 0, 0, -1, 0, 
                0, 1, 0, 0,
                1, 0, 0, 0,
                0, 0, 0, 1;
        y_s90 << 0, 0, 1, 0,
                0, 1, 0, 0,
                -1, 0, 0, 0,
                0, 0, 0, 1;
        z_180 << -1, 0, 0, 0,
                 0, -1, 0, 0,
                 0, 0, 1, 0,
                 0, 0, 0, 1;
        Eigen::MatrixXd support_T2 = home_T * y_n90 * z_180;
        Eigen::MatrixXd support_pre_T2 = home_T * y_n90 * z_180;
        Eigen::MatrixXd support_T1 = home_T * y_s90;
        Eigen::MatrixXd support_pre_T1 = home_T * y_s90;
        
        std::vector<std::string> brick_names = lego_ptr_->get_brick_names();
        int num_bricks = brick_names.size();

        // initialize the cost matrix we want to calculate for milp solver
        std::vector<std::vector<double>> cost_matrix_a(num_tasks_, std::vector<double>(num_bricks, 1000000));
        std::vector<std::vector<double>> cost_matrix_b(num_tasks_, std::vector<double>(num_bricks, 1000000));
        std::vector<std::vector<double>> support_matrix(2, std::vector<double>(num_tasks_, 1000000));
        std::vector<std::vector<int>> delta_matrix(num_tasks_, std::vector<int>(num_bricks, 0));
        std::vector<std::vector<int>> precedence;
        std::vector<int> sup_required(num_tasks_, 0);

        // calculate the ik for each lego block's initial pose
        std::vector<bool> lego_r1_reachable(num_bricks, false);
        std::vector<bool> lego_r2_reachable(num_bricks, false);
        std::vector<lego_manipulation::math::VectorJd> r1_lego_goals(num_bricks, Eigen::MatrixXd::Zero(6, 1));
        std::vector<lego_manipulation::math::VectorJd> r2_lego_goals(num_bricks, Eigen::MatrixXd::Zero(6, 1));
        for (int i = 0; i < num_bricks; i++) {
            std::string brick_name = brick_names[i];
            log("Calculating IK for brick: " + std::to_string(i) + " " + brick_name, LogLevel::INFO);
            Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);
            int press_side = 2;
            lego_ptr_->calc_brick_grab_pose(brick_name, true, 1,
                                -1, -1, -1, -1, press_side, cart_T);

            bool r1_reachable, r2_reachable;
            calculateIKforLego(cart_T, home_q, 0, false, r1_lego_goals[i], r1_reachable);
            calculateIKforLego(cart_T, home_q, 1, false, r2_lego_goals[i], r2_reachable);
            lego_r1_reachable[i] = r1_reachable;
            lego_r2_reachable[i] = r2_reachable;

            // find any blocks on top, add precedence
            std::vector<std::string> above_bricks = lego_ptr_->get_brick_above(brick_name);
            for (auto & above_brick : above_bricks) {
                int above_idx = std::find(brick_names.begin(), brick_names.end(), above_brick) - brick_names.begin();
                precedence.push_back({above_idx, i});
            }
        }

        for (int i = 0; i < num_tasks_; i++) {
            int task_idx = i + 1;
            // calculate the ik for the target pose
            auto cur_graph_node = task_json_[std::to_string(task_idx)];
            int brick_id = cur_graph_node["brick_id"].asInt();
            std::string brick_name = lego_ptr_->get_brick_name_by_id(cur_graph_node["brick_id"].asInt(), cur_graph_node["brick_seq"].asInt());

            lego_manipulation::math::VectorJd r1_cur_goal, r2_cur_goal;
            bool r1_reachable, r2_reachable;
            Eigen::MatrixXd cart_T = Eigen::MatrixXd::Identity(4, 4);
            lego_ptr_->calc_brick_grab_pose(brick_name, 1, 0, cur_graph_node["x"].asInt(), cur_graph_node["y"].asInt(), 
                                cur_graph_node["z"].asInt(), cur_graph_node["ori"].asInt(), cur_graph_node["press_side"].asInt(),
                                cart_T);
            calculateIKforLego(cart_T, home_q, 0, true, r1_cur_goal, r1_reachable);
            calculateIKforLego(cart_T, home_q, 1, true, r2_cur_goal, r2_reachable);

            if (r1_reachable) {
                visualize_robot_pose(r1_cur_goal, 0);
            }
            if (r2_reachable) {
                visualize_robot_pose(r2_cur_goal, 1);
            }
            ros::Duration(1).sleep();

            std::string req_type = "b" + std::to_string(brick_id) + "_";
            for (int t = 0; t < num_bricks; t++) {
                std::string brick_name = brick_names[t];
                // check if brick name start with the req_type
                if (brick_name.find(req_type) == 0) {
                    delta_matrix[i][t] = 1;
                }
                else {
                    continue;
                }

                if (r1_reachable && lego_r1_reachable[t]) {
                    double cost = 0;
                    cost += (r1_home - r1_lego_goals[t]).norm();
                    cost += (r1_lego_goals[t] - r1_cur_goal).norm();
                    cost_matrix_a[i][t] = cost;
                }
                if (r2_reachable && lego_r2_reachable[t]) {
                    double cost = 0;
                    cost += (r2_home - r2_lego_goals[t]).norm();
                    cost += (r2_lego_goals[t] - r2_cur_goal).norm();
                    cost_matrix_b[i][t] = cost;
                }
            }

            if (cur_graph_node["sup_robot_id"].asInt() > 0) {
                sup_required[i] = 1;
                lego_ptr_->calc_brick_sup_pose(brick_name, cart_T, cur_graph_node["sup_side"].asInt(), true, support_pre_T1);
                lego_ptr_->calc_brick_sup_pose(brick_name, cart_T, cur_graph_node["sup_side"].asInt(), true, support_pre_T2);
                lego_ptr_->calc_brick_sup_pose(brick_name, cart_T, cur_graph_node["sup_side"].asInt(), false, support_T1);
                lego_ptr_->calc_brick_sup_pose(brick_name, cart_T, cur_graph_node["sup_side"].asInt(), false, support_T2);
                
                Eigen::MatrixXd init_q = home_q;
                init_q(4) = 30;
                calculateIKforLego(support_pre_T1, init_q, 0, true, r1_cur_goal, r1_reachable);
                calculateIKforLego(support_pre_T2, init_q, 1, true, r2_cur_goal, r2_reachable);

                if (r1_reachable) {
                    support_matrix[0][i] = (r1_home - r1_cur_goal).norm();
                    visualize_robot_pose(r1_cur_goal, 0);
                }
                if (r2_reachable) {
                    support_matrix[1][i] = (r2_home - r2_cur_goal).norm();
                    visualize_robot_pose(r2_cur_goal, 1);
                }
                ros::Duration(1).sleep();
            }

            // move the lego brick to the target pose
            instance_->moveObject(getLegoTarget(task_idx));
            instance_->updateScene();

        }

        // print the cost matrix
        std::ofstream cost_file(output_dir_ + "/cost_matrix_a.csv");
        for (int j = 0; j < num_bricks; j++) {
            cost_file << brick_names[j] << ",";
        }
        cost_file << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            for (int j = 0; j < num_bricks; j++) {
                cost_file << cost_matrix_a[i][j] << ",";
            }
            cost_file << std::endl;
        }
        cost_file.close();

        cost_file.open(output_dir_ + "/cost_matrix_b.csv");
        for (int j = 0; j < num_bricks; j++) {
            cost_file << brick_names[j] << ",";
        }
        cost_file << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            for (int j = 0; j < num_bricks; j++) {
                cost_file << cost_matrix_b[i][j] << ",";
            }
            cost_file << std::endl;
        }

        // print the support matrix
        std::ofstream support_file(output_dir_ + "/support_matrix.csv");
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < num_tasks_; j++) {
                support_file << support_matrix[i][j] << ",";
            }
            support_file << std::endl;
        }
        support_file.close();

        // print the delta matrix
        std::ofstream delta_file(output_dir_ + "/delta_matrix.csv");
        for (int j = 0; j < num_bricks; j++) {
            delta_file << brick_names[j] << ",";
        }
        delta_file << std::endl;
        for (int i = 0; i < num_tasks_; i++) {
            for (int j = 0; j < num_bricks; j++) {
                delta_file << delta_matrix[i][j] << ",";
            }
            delta_file << std::endl;
        }
        delta_file.close();

        // print the support_req matrix
        std::ofstream support_req_file(output_dir_ + "/support_req.csv");
        for (int i = 0; i < num_tasks_; i++) {
            support_req_file << sup_required[i] << ",";
        }
        support_req_file.close();

        // print the precedence matrix
        std::ofstream precedence_file(output_dir_ + "/precedence.csv");
        for (int i = 0; i < precedence.size(); i++) {
            precedence_file << precedence[i][0] << "," << precedence[i][1] << "," << std::endl;
        }
        precedence_file.close();
    }


private:
    ros::NodeHandle nh_;
    
    robot_model::RobotModelPtr robot_model_;
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    planning_scene::PlanningScenePtr planning_scene_;

    // lego pointer
    lego_manipulation::lego::Lego::Ptr lego_ptr_;
    Json::Value task_json_;
    int num_tasks_ = 0;

    // update gazebo state
    ros::ServiceClient set_state_client_;
    ros::ServiceClient planning_scene_diff_client;

     
    std::vector<std::string> group_names_; // group name for moveit controller services
    std::shared_ptr<MoveitInstance> instance_;
    std::string output_dir_; 
};


int main(int argc, char** argv) {
    ros::init(argc, argv, "lego_node");
    
    // start ros node
    ros::NodeHandle nh("~");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Read ROS Params
    std::string  config_fname, root_pwd, output_dir;
    
    std::vector<std::string> group_names = {"left_arm", "right_arm"};
    for (int i = 0; i < 2; i++) {
        if (nh.hasParam("group_name_" + std::to_string(i))) {
            nh.getParam("group_name_" + std::to_string(i), group_names[i]);
       }
    }
    std::vector<std::string> eof_links = {"left_arm_link_tool", "right_arm_link_tool"};

    nh.param<std::string>("config_fname", config_fname, "");
    nh.param<std::string>("root_pwd", root_pwd, "");
    nh.param<std::string>("output_dir", output_dir, "");
    
    // Initialize the Dual Arm Planner
    setLogLevel(LogLevel::INFO);

    TaskAssignment planner(output_dir, group_names);

    ROS_INFO("Setting up the Lego Factory");

    // Read the lego poses
    if (!config_fname.empty() && !root_pwd.empty()) {
        planner.setLegoFactory(config_fname, root_pwd);
        planner.initLegoPositions();
    }

    planner.calculateCostMatrix();

    ROS_INFO("Finished calculating cost matrix");
    
    ros::shutdown();
    return 0;
}
