#include "SingleAgentPlanner.h"
#include "logger.h"

bool SingleAgentPlanner::init(const PlannerOptions &options) {
    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);
    return true;
}

STRRT::STRRT(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(instance, robot_id) {
    }

bool STRRT::init(const PlannerOptions &options) { 
    // initialize two trees, one for the start and one for the goal
    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);

    auto startVertex = std::make_shared<Vertex>(start_pose_);
    startVertex->setTime(0.0);
    start_tree_.addRoot(startVertex);
    totalTreeSize++;

    min_time_ = instance_->computeDistance(start_pose_, goal_pose_) / vMax_;
    max_time_ = min_time_ * 2.0;

    obstacles_ = options.obstacles;

    return true;
}

bool STRRT::plan(const PlannerOptions &options) {

    start_time_ = std::chrono::high_resolution_clock::now();

    init(options);

    // 1. Start the binary tree with a start and goal poses
    // 2. Sample a random pose
    
    while (!terminate(options)) {
        log("iteration: " + std::to_string(numIterations_) 
            + ", start tree size " + std::to_string(start_tree_.vertices.size())
            + ", goal tree size " + std::to_string(goal_tree_.vertices.size()), LogLevel::DEBUG);
        numIterations_++;
        cur_batch_size++;
        
        // 1. expand batch time bound
        if (shouldExpandTime()) {
            expandTime();
            log("iteration: " + std::to_string(numIterations_) 
                + ", start tree size " + std::to_string(start_tree_.vertices.size())
                + ", goal tree size " + std::to_string(goal_tree_.vertices.size()), LogLevel::INFO);
            log ("Expanding time bound to " + std::to_string(max_time_), LogLevel::INFO);
            cur_batch_size = 0;
        }

        // 2. sample goal
        std::shared_ptr<Vertex> new_goal;
        sampleGoal(new_goal);
        goal_tree_.addRoot(new_goal);

        // 3. sample conditionally
        std::shared_ptr<Vertex> new_sample;
        bool found_sample = sampleConditionally(new_sample);
        if (!found_sample) {
            log("Failed to sample new pose", LogLevel::DEBUG);
            continue;
        }
        numValidSamples_++;
        log("Sampled new pose", LogLevel::DEBUG);
        log(new_sample->pose, LogLevel::DEBUG);
        log("at time " + std::to_string(new_sample->time), LogLevel::DEBUG);
        
        // 4. connect
        std::shared_ptr<Vertex> delta_sample;
        if (extend(new_sample, *current_tree_, delta_sample, goal_tree) != TRAPPED) {
            totalTreeSize++;
            if (connect(delta_sample, *other_tree_, !goal_tree) == REACHED) {
                update_solution(delta_sample, goal_tree);
                prune_trees();
            }
        }

        // 5. swap_trees
        swap_trees();

    }

    log("Final iteration: " + std::to_string(numIterations_) 
            + ", " + std::to_string(numValidSamples_) + " valid samples"
            + ", start tree size " + std::to_string(start_tree_.vertices.size())
            + ", goal tree size " + std::to_string(goal_tree_.vertices.size()), LogLevel::INFO);

    if (best_cost_ < std::numeric_limits<double>::max()) {
        return true;
    }
    return false;
}

bool STRRT::terminate(const PlannerOptions &options) {
    log("num iterations: " + std::to_string(numIterations_) + " max iterations: " + std::to_string(options.max_planning_iterations), LogLevel::DEBUG);
    if (numIterations_ > options.max_planning_iterations) {
        return true;
    }
    auto elapsed = std::chrono::high_resolution_clock::now() - start_time_;
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
    log("elapsed time: " + std::to_string(elapsed_seconds) + " max time: " + std::to_string(options.max_planning_time), LogLevel::DEBUG);
    if (elapsed_seconds > options.max_planning_time) {
        return true;
    }

    return false;
}

bool STRRT::shouldExpandTime() {
    if (time_bounded_)
        return false;
    return cur_batch_size >= batch_size;
}

void STRRT::expandTime() {
    max_time_ *= 2.0;
}

void STRRT::sampleGoal(std::shared_ptr<Vertex> &new_goal) {
    new_goal = std::make_shared<Vertex>(goal_pose_);
    
    // sample a time for the goal
    std::uniform_real_distribution<double> distribution(min_time_, max_time_);
    new_goal->setTime(distribution(rng_));
}

bool STRRT::sampleConditionally(std::shared_ptr<Vertex> &new_sample) {
    RobotPose newpose = instance_->initRobotPose(robot_id_);
    bool found_sample = false;
    int tries = 0;
    int max_tries = 10;

    while (!found_sample && tries < max_tries) {
        tries++;
        //log("sample tries: " + std::to_string(tries), LogLevel::DEBUG);
        found_sample = instance_->sample(newpose);
        if (!found_sample) {
            continue;
        }
        // calculate time bound
        double min_time = instance_->computeDistance(start_pose_, newpose) / vMax_;
        double max_time = max_time_ - instance_->computeDistance(newpose, goal_pose_) / vMax_;
        if (min_time > max_time) {
            found_sample = false;
            continue;
        }
        
        new_sample = std::make_shared<Vertex>(newpose);
        std::uniform_real_distribution<double> distribution(min_time, max_time);
        new_sample->setTime(distribution(rng_));
    }

    return found_sample;
}

double STRRT::distanceSpaceTime(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
    double distanceSpace = instance_->computeDistance(a->pose, b->pose);
    double distanceTime = b->time - a->time;
    if (distanceTime * vMax_ < distanceSpace)
        return std::numeric_limits<double>::max();
    return distanceSpace + distanceTime;
}

void STRRT::nearest(const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &nearest_vertex, const Tree &tree, bool goalTree) {
    assert(tree.vertices.size() > 0);

    double min_distance = std::numeric_limits<double>::max();
    for (auto &vertex : tree.vertices) {
        double distance;
        if (goalTree) {
            distance = distanceSpaceTime(sample, vertex);
        }
        else {
            distance = distanceSpaceTime(vertex, sample);
        }
        if (distance <= min_distance) {
            min_distance = distance;
            nearest_vertex = vertex;
        }
    }
    assert(nearest_vertex != nullptr);
}

GrowState STRRT::extend(const std::shared_ptr<Vertex> &new_sample, Tree &tree, std::shared_ptr<Vertex> &delta_vertex, bool goalTree) {
    // find the nearest vertex in the tree
    std::shared_ptr<Vertex> nearest_vertex;
    nearest(new_sample, nearest_vertex, tree, goalTree);
    log("nearest vertex:", LogLevel::DEBUG);
    log(nearest_vertex->pose, LogLevel::DEBUG);
    log("nearest vertex time: " + std::to_string(nearest_vertex->time), LogLevel::DEBUG);
    log("new sample:", LogLevel::DEBUG);
    log(new_sample->pose, LogLevel::DEBUG);
    log("new sample time: " + std::to_string(new_sample->time), LogLevel::DEBUG);

    double distanceSpace = instance_->computeDistance(nearest_vertex->pose, new_sample->pose);
    double distanceTime = (goalTree) ? nearest_vertex->time - new_sample->time : new_sample->time - nearest_vertex->time;
    if (distanceTime * vMax_ < distanceSpace) {
        log("nearest vertex not reachable", LogLevel::DEBUG);
        return TRAPPED;
    }
    bool reached = true;
    if (distanceTime <= max_deltat_ && distanceSpace <= vMax_ * max_deltat_)
    {
        // check if can directly reach the new sample
        if (std::abs(distanceSpace) < 1e-6) {
            log("Reached the new sample", LogLevel::DEBUG);
            delta_vertex = new_sample;
        }
        else if (instance_->connect(nearest_vertex->pose, new_sample->pose)) {
            log("Reach the new sample", LogLevel::DEBUG);
            delta_vertex = new_sample;
        }
        else {
            log("Trapped on the way to new sample", LogLevel::DEBUG);
            return TRAPPED;
        }
    }
    else {
        reached = false;
        // steer from the nearest vertex to the new sample
        RobotPose delta_pose = instance_->initRobotPose(robot_id_);
        if (instance_->steer(nearest_vertex->pose, new_sample->pose, vMax_ * max_deltat_, delta_pose)) {
            log("Steer to the new sample", LogLevel::DEBUG);
            delta_vertex = std::make_shared<Vertex>(delta_pose);
            delta_vertex->setPose(delta_pose);

            // move the time by the moved distance
            double coveredSpace = instance_->computeDistance(nearest_vertex->pose, delta_pose);
            double delta_time = (goalTree) ? nearest_vertex->time - coveredSpace/vMax_ : nearest_vertex->time + coveredSpace / vMax_;
            // if no movement (already reached but waiting for the time to catch up)
            if (std::abs(coveredSpace) < 1e-6) {
                delta_time = (goalTree) ? nearest_vertex->time - max_deltat_ : nearest_vertex->time + max_deltat_;
            }
            delta_vertex->setTime(delta_time);
        }
        else {
            log("Trapped on the way to new sample", LogLevel::DEBUG);
            return TRAPPED;
        }
    }

    log("delta pose:", LogLevel::DEBUG);
    log(delta_vertex->pose, LogLevel::DEBUG);
    log("delta time: " + std::to_string(delta_vertex->time), LogLevel::DEBUG);

    if (delta_vertex != nullptr) {
        // check collision
        bool valid_motion = false;
        if (goalTree) {
            valid_motion = validateMotion(delta_vertex, nearest_vertex);
        }
        else {
            valid_motion = validateMotion(nearest_vertex, delta_vertex);
        }
        
        if (valid_motion) {
            tree.addVertex(delta_vertex);
            if (delta_vertex->parent == nullptr) {
                delta_vertex->addParent(nearest_vertex);
                log("add parent to delta vertex", LogLevel::DEBUG);
            }
            else {
                delta_vertex->addOtherParent(nearest_vertex);
                log("add other parent to delta vertex", LogLevel::DEBUG);
            }
            assert(delta_vertex->parent != nullptr);
            return reached ? REACHED : ADVANCED;
        }
    }

    return TRAPPED;
}

// RobotPose STRRT::interpolate(const RobotPose &a, const RobotPose &b, double t1, double t2, double t) {
//     assert (t1 <= t2 && t1 <= t && t <= t2 && a.robot_id == b.robot_id && a.robot_name == b.robot_name);
//     RobotPose result = instance_->initRobotPose(a.robot_id);
//     assert(result.joint_values.size() == a.joint_values.size());

//     for (int i = 0; i < a.joint_values.size(); i++) {
//         result.joint_values[i] = (a.joint_values[i] + (b.joint_values[i] - a.joint_values[i]) * (t - t1) / (t2 - t1));
//     }

//     return result;
// }

bool STRRT::validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
    // check inter agent collision
    double t1 = a->time;
    double t2 = b->time;
    assert (t1 <= t2);
    int num_steps = (t2 - t1) / 0.1 + 1;

    std::vector<std::vector<RobotPose>> obs_poses;
    // precompute all the obstacle poses
    obs_poses.reserve(obstacles_.size());
    for (const RobotTrajectory &obs : obstacles_) {
        std::vector<RobotPose> obs_i_pose;
        obs_i_pose.reserve(num_steps);

        int ind = 0;
        // compile this dynamic obstacle's poses at each time step
        for (int s = 0; s < num_steps; s++) {
            double t = t1 + s * 0.1;
            while (ind + 1 < obs.times.size() && obs.times[ind + 1] <= t) {
                ind++;
            }
            if (ind + 1 == obs.times.size()) {
                // assuming obstacle stays at the end of the trajectory
                obs_i_pose.push_back(obs.trajectory[ind]);
            } else {
                double alpha = (t - obs.times[ind]) / (obs.times[ind + 1] - obs.times[ind]);
                RobotPose obs_i_pose_s = instance_->interpolate(obs.trajectory[ind], obs.trajectory[ind + 1], alpha);
                obs_i_pose.push_back(obs_i_pose_s);
            }
        }
        obs_poses.push_back(obs_i_pose);
    }

    // check for inter-agent collision
    for (int s = 0; s < num_steps; s++) {
        double t = t1 + s * 0.1;
        double alpha = (t - t1) / (t2 - t1);
        RobotPose a_pose = instance_->interpolate(a->pose, b->pose, alpha);
        std::vector<RobotPose> all_poses;
        all_poses.push_back(a_pose);

        for (int i = 0; i < obs_poses.size(); i++) {
            all_poses.push_back(obs_poses[i][s]);
        }
        
        if (instance_->checkCollision(all_poses, true) == false) {
            return false;
        }
    }
    return true;
}

GrowState STRRT::connect(const std::shared_ptr<Vertex> &new_sample, Tree &tree, bool goalTree) {
    GrowState gsc = ADVANCED;
    while (gsc == ADVANCED) {
        std::shared_ptr<Vertex> delta_sample;
        gsc = extend(new_sample, tree, delta_sample, goalTree);
    }
    return gsc;
}

void STRRT::update_solution(std::shared_ptr<Vertex> &new_sample, bool goalTree) {
    // connect the new sample to the 
    assert (new_sample->parent != nullptr && new_sample->otherParent != nullptr);

    // trace back the path from the new sample to the root
    std::vector<std::shared_ptr<Vertex>> path, back_path;
    std::shared_ptr<Vertex> current = new_sample;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    // reverse the stack
    std::reverse(path.begin(), path.end());

    // trace to the other parent root
    current = new_sample->otherParent;
    while (current != nullptr) {
        back_path.push_back(current);
        current = current->parent;
    }

    // concatenate the two paths
    path.insert(path.end(), back_path.begin(), back_path.end());

    if (goalTree) {
        std::reverse(path.begin(), path.end());
    }

    double solutionCost = path.back()->time;
    if (solutionCost < best_cost_) {
        best_cost_ = solutionCost;
        // set the path to the solution
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        for (auto &vertex : path) {
            solution_.times.push_back(vertex->time);
            solution_.trajectory.push_back(vertex->pose);
        }
        log("Found a better solution with cost: " + std::to_string(best_cost_), LogLevel::HLINFO);
    }

    time_bounded_ = true;
    max_time_ = best_cost_;

    return;
}

void STRRT::prune_trees() {

}

void STRRT::swap_trees() {
    current_tree_ = (current_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    other_tree_ = (other_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    goal_tree = !goal_tree;
}



bool STRRT::getPlan(RobotTrajectory &solution) const {
    if (best_cost_ < std::numeric_limits<double>::max()) {
        solution = solution_;
        return true;
    }
    return false;
}