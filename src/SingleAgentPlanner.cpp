#include "SingleAgentPlanner.h"


STRRT::STRRT(std::shared_ptr<PlanInstance> instance, int robot_id)
    : SingleAgentPlanner(instance, robot_id) 
{
    // initialize two trees, one for the start and one for the goal
    auto start_pose = instance_->getStartPose(robot_id);
    auto goal_pose = instance_->getGoalPose(robot_id);

    start_tree_.addRoot(std::make_shared<Vertex>(start_pose));
    totalTreeSize++;

    min_time_ = instance_->computeDistance(start_pose, goal_pose) / vMax_;
    max_time_ = min_time_ * 2.0;
}

bool STRRT::plan(const PlannerOptions &options) {

    start_time_ = std::chrono::high_resolution_clock::now();
    const std::vector<RobotTrajectory> obstacles = options.obstacles;

    // 1. Start the binary tree with a start and goal poses
    // 2. Sample a random pose
    
    while (terminate(options)) {
        numIterations_++;

        
        // 1. expand batch time bound
        if (shouldExpandTime()) {
            expandTime();
            cur_batch_size = 0;
        }

        // 2. sample goal
        std::shared_ptr<Vertex> new_goal;
        sampleGoal(new_goal);
        goal_tree_.addRoot(new_goal);

        // 3. sample conditionally
        std::shared_ptr<Vertex> new_sample;
        sampleConditionally(new_sample);
        
        // 4. connect
        if (extend(new_sample, *current_tree_, goal_tree) != TRAPPED) {
            cur_batch_size++;
            totalTreeSize++;
            if (connect(new_sample, *other_tree_, !goal_tree) == REACHED) {
                update_solution(new_sample);
                prune_trees();
            }
        }

        // 5. swap_trees
        swap_trees();

    }


    return true;
}

bool STRRT::terminate(const PlannerOptions &options) {
    if (numIterations_ > options.max_planning_iterations) {
        return true;
    }
    auto elapsed = std::chrono::high_resolution_clock::now() - start_time_;
    double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(elapsed).count();
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
    new_goal = std::make_shared<Vertex>();
    new_goal->setPose(goal_pose_);
    
    // sample a time for the goal
    std::uniform_real_distribution<double> distribution(min_time_, max_time_);
    new_goal->setTime(distribution(rng_));
}

void STRRT::sampleConditionally(std::shared_ptr<Vertex> &new_sample) {
    new_sample = std::make_shared<Vertex>();
    RobotPose &newpose = new_sample->pose;
    instance_->sample(newpose);

    // calculate time bound
    double min_time = instance_->computeDistance(start_pose_, newpose) / vMax_;
    double max_time = max_time_ - instance_->computeDistance(newpose, goal_pose_) / vMax_;
    std::uniform_real_distribution<double> distribution(min_time, max_time);
    new_sample->setTime(distribution(rng_));
}

double STRRT::distanceSpaceTime(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b) {
    double distanceSpace = instance_->computeDistance(a->pose, b->pose);
    double distanceTime = b->time - a->time;
    if (distanceTime * vMax_ < distanceSpace)
        return std::numeric_limits<double>::max();
    return distanceSpace + distanceTime;
}

std::shared_ptr<Vertex> STRRT::nearest(std::shared_ptr<Vertex> &sample, Tree &tree, bool goalTree) {
    std::shared_ptr<Vertex> nearest_vertex;
    double min_distance = std::numeric_limits<double>::max();
    for (auto &vertex : tree.vertices) {
        double distance;
        if (goalTree) {
            distance = distanceSpaceTime(sample, vertex);
        }
        else {
            distance = distanceSpaceTime(vertex, sample);
        }
        if (distance < min_distance) {
            min_distance = distance;
            nearest_vertex = vertex;
        }
    }
    return nearest_vertex;
}

GrowState STRRT::extend(std::shared_ptr<Vertex> &new_sample, Tree &tree, bool goalTree) {
    // find the nearest vertex in the tree
    std::shared_ptr<Vertex> nearest_vertex = nearest(new_sample, tree, goalTree);
    double distanceSpace = instance_->computeDistance(nearest_vertex->pose, new_sample->pose);

    // steer from the nearest vertex to the new sample
    RobotPose delta_pose;
    
    if (instance_->steer(nearest_vertex->pose, new_sample->pose, vMax_ * max_deltat_, delta_pose)) {
        std::shared_ptr<Vertex> delta_vertex = std::make_shared<Vertex>();
        delta_vertex->setPose(delta_pose);

        double coveredSpace = instance_->computeDistance(nearest_vertex->pose, delta_pose);
        double coveredTime = coveredSpace / distanceSpace * (new_sample->time - nearest_vertex->time);
        double delta_time = nearest_vertex->time + coveredTime;
        delta_vertex->setTime(delta_time);

        // TODO: check for collision with constraints / obstacles
        
        tree.addVertex(delta_vertex);
        delta_vertex->addParent(nearest_vertex);
        if (std::abs(delta_time - new_sample->time) < 1e-6)
            return REACHED;
        return ADVANCED;
    }
    return TRAPPED;
}

GrowState STRRT::connect(std::shared_ptr<Vertex> &new_sample, Tree &tree, bool goalTree) {
    GrowState gsc = ADVANCED;
    while (gsc == ADVANCED) {
        gsc = extend(new_sample, tree, goalTree);
    }
    return gsc;
}

void STRRT::update_solution(std::shared_ptr<Vertex> &new_sample) {

}

void STRRT::prune_trees() {

}

void STRRT::swap_trees() {
    current_tree_ = (current_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    other_tree_ = (other_tree_ == &start_tree_) ? &goal_tree_ : &start_tree_;
    goal_tree = !goal_tree;
}



bool STRRT::getPlan(RobotTrajectory &solution) const {
    // Retrieve the plan. For now, we just indicate success or failure.
    return true; // Placeholder
}