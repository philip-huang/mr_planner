#include <instance.h>
#include <logger.h>

MoveitInstance::MoveitInstance(robot_model::RobotModelPtr robot_model, robot_state::RobotStatePtr kinematic_state,
                               std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                               planning_scene::PlanningScenePtr planning_scene)
    : robot_model_(robot_model), kinematic_state_(kinematic_state), move_group_(move_group), planning_scene_(planning_scene)
{}

void PlanInstance::setNumberOfRobots(int num_robots) {
    num_robots_ = num_robots;
    start_poses_.resize(num_robots);
    goal_poses_.resize(num_robots);
}

void PlanInstance::setStartPose(int robot_id, const std::vector<double> &pose) {
    start_poses_[robot_id].robot_id = robot_id;
    start_poses_[robot_id].robot_name = robot_names_[robot_id];
    start_poses_[robot_id].joint_values = pose;
}

void PlanInstance::setGoalPose(int robot_id, const std::vector<double> &pose) {
    goal_poses_[robot_id].robot_id = robot_id;
    goal_poses_[robot_id].robot_name = robot_names_[robot_id];
    goal_poses_[robot_id].joint_values = pose;
}

RobotPose PlanInstance::initRobotPose(int robot_id) const {
    RobotPose pose;
    pose.robot_id = robot_id;
    pose.robot_name = robot_names_[robot_id];
    pose.joint_values.resize(start_poses_[robot_id].joint_values.size());
    return pose;
}

bool MoveitInstance::checkCollision(const std::vector<RobotPose> &poses, bool self) const {
    /* check if there is robot-robot or scene collision for a set of poses for all robots*/
    /* true if no collision, false if has collision*/
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = move_group_->getName();

    // set the robot state to the one we are checking
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    
    std::vector<double> all_joints;
    collision_detection::AllowedCollisionMatrix acm = planning_scene_->getAllowedCollisionMatrixNonConst();

    int index = 0;
    for (int i = 0; i < num_robots_; i++) {
        std::string group = robot_names_[i];

        // find if this robot is in collision with the environment
        bool checking_i = false;
        RobotPose pose;
        for (int j = 0; j < poses.size(); j++) {
            if (poses[j].robot_id == i) {
                checking_i = true;
                pose = poses[j];
                break;
            }
        }

        // set the acm for this robot to true if it is not checked for collision
        if (!checking_i) {
            auto links = kinematic_state_->getJointModelGroup(group)->getLinkModelNamesWithCollisionGeometry();
            for (const auto &link : links) {
                acm.setEntry(link, true);
            }
            // insert the joint values for this robot
            all_joints.insert(all_joints.end(), start_poses_[i].joint_values.begin(), start_poses_[i].joint_values.end());
        }
        else {
            // copy the joint values for this robot
            all_joints.insert(all_joints.end(), pose.joint_values.begin(), pose.joint_values.end());
        }
        
        index += start_poses_[i].joint_values.size();
    }

    robot_state.setJointGroupPositions(move_group_->getName(), all_joints);

    c_res.clear();
    if (self) {
        planning_scene_->checkSelfCollision(c_req, c_res, robot_state, acm);
    } else {
        planning_scene_->checkCollision(c_req, c_res, robot_state, acm);
    }
    return !c_res.collision;
}

double MoveitInstance::computeDistance(const RobotPose& a, const RobotPose &b) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    double distance = robot_state_a.distance(robot_state_b);
    return distance;
}

RobotPose MoveitInstance::interpolate(const RobotPose &a, const RobotPose&b, double t) const {
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);

    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);

    moveit::core::RobotState res_state = planning_scene_->getCurrentStateNonConst();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    robot_state_a.interpolate(robot_state_b, t, res_state, joint_model_group);

    RobotPose res = initRobotPose(a.robot_id);
    res_state.copyJointGroupPositions(a.robot_name, res.joint_values);
    return res;
}

bool MoveitInstance::connect(const RobotPose& a, const RobotPose& b) {
    /* check if a collision-free kinematic path exists from pose a to b for the robot (ignoring other robots)*/
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    
    // discretize and check for collision along the path
    double joint_distance = computeDistance(a, b);
    int num_steps = std::ceil(joint_distance / 0.1);

    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = a.robot_name;
    c_req.contacts = true;
    c_req.max_contacts = 100;

    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);

    for (int i = 0; i <= num_steps; i++) {
        c_res.clear();
        
        robot_state.setJointGroupPositions(a.robot_name, a.joint_values);
        robot_state_a.interpolate(robot_state_b, (double)i / num_steps, robot_state, joint_model_group);
        planning_scene_->checkCollision(c_req, c_res, robot_state);
        if (c_res.collision) {
            return false;
        }
    }

    return true;
}

bool MoveitInstance::steer(const RobotPose& a, const RobotPose& b, double max_dist, RobotPose& result) {
    /* find a collision-free that steers the robot from a towards b for max_distance */
    assert(a.robot_id == b.robot_id && a.robot_name == b.robot_name);
    
    double joint_distance = computeDistance(a, b);
    if (joint_distance <= max_dist) {
        result = b;
        return true;
    }

    auto joint_model_group = kinematic_state_->getJointModelGroup(a.robot_name);
    moveit::core::RobotState robot_state_a = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state_b = planning_scene_->getCurrentStateNonConst();
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();

    robot_state_a.setJointGroupPositions(a.robot_name, a.joint_values);
    robot_state_b.setJointGroupPositions(b.robot_name, b.joint_values);
    robot_state_a.interpolate(robot_state_b, max_dist / joint_distance, robot_state, joint_model_group);
    result.robot_id = a.robot_id;
    result.robot_name = a.robot_name;
    result.joint_values.resize(a.joint_values.size());
    robot_state.copyJointGroupPositions(a.robot_name, result.joint_values);

    if (connect(a, result)) {
        return true;
    }
    else {
        return false;
    }
}

bool MoveitInstance::sample(RobotPose &pose) {
    /* sample a collision free pose for the robot (ignoring other robots)
    */

    // initialize the joint values vector
    std::string robot_name = robot_names_[pose.robot_id];
    pose.robot_name = robot_name;

    // get the bounds of the joint space for the robot
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state_->getJointModelGroup(robot_name);
    const std::vector<const moveit::core::JointModel*> & joint_models = joint_model_group->getActiveJointModels();
    
    // boilerplate for checking collision
    moveit::core::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
    collision_detection::CollisionRequest c_req;
    collision_detection::CollisionResult c_res;
    c_req.group_name = robot_name;

    bool in_collision = true;
    int attempt = 0;
    int max_attempts = 10;
    do {
        std::vector<double> joint_values;
        joint_values.reserve(joint_models.size());

        // sample each joint
        for (int i = 0; i < joint_models.size(); i++) {
            const auto &bounds = joint_models[i]->getVariableBounds();
            if (!bounds.empty()) {
                // assume the joint has only one variable
                std::uniform_real_distribution<double> distribution(bounds[0].min_position_, bounds[0].max_position_);
                joint_values.push_back(distribution(rng_));
            }
            else {
                // raise an error if the joint has no bounds
                throw std::runtime_error("Joint " + joint_models[i]->getName() + " has no bounds");
            }
        }

        // check collision
        c_res.clear();
        robot_state.setJointGroupPositions(robot_name, joint_values);
        planning_scene_->checkCollision(c_req, c_res, robot_state);

        in_collision = c_res.collision;
        if (!in_collision) {
            pose.joint_values = joint_values;
        }

        attempt ++;
    } while (in_collision && attempt < max_attempts);
    

    
    return !in_collision;
}

