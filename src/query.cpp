#include "query.h"
#include "roadmap.h"
#include "roadmap.cpp"
#include <queue>

/**
 * @brief Initialize the planner with the given options
 * @param options: Planner options
 */
bool AStarPlanner::init(const PlannerOptions &options) {
    max_planning_time_ = options.max_planning_time;
    max_planning_iterations_ = options.max_planning_iterations;
    obstacles_ = options.obstacles;
    vMax_ = instance_->getVMax(robot_id_);

    start_pose_ = instance_->getStartPose(robot_id_);
    goal_pose_ = instance_->getGoalPose(robot_id_);

    // find the last time when obstacles pass thru the goal poses
    min_time_ = instance_->computeDistance(start_pose_, goal_pose_) / vMax_;
    for (const RobotTrajectory &obs : obstacles_) {
        double last_time = obs.times.back();
        int num_points = last_time / col_dt_ + 1;
        int ind = 0;
        RobotPose obs_i_pose;
        for (int i = 0; i < num_points; i++) {
            double t = i * col_dt_;
            while (ind + 1 < obs.times.size() && obs.times[ind + 1] <= t) {
                ind++;
            }
            if (ind + 1 == obs.times.size()) {
                // assuming obstacle stays at the end of the trajectory
                obs_i_pose = obs.trajectory[ind];
            } else {
                double alpha = (t - obs.times[ind]) / (obs.times[ind + 1] - obs.times[ind]);
                obs_i_pose = instance_->interpolate(obs.trajectory[ind], obs.trajectory[ind + 1], alpha);
            }
            if (instance_->checkCollision({goal_pose_, obs_i_pose}, true) == true) {
                // has collision
                min_time_ = std::max(min_time_, t + col_dt_);
            }
        }
    }

    if (min_time_ < 1e-6) {
        return false;
    }

    max_time_ = min_time_ * 2.0;

    return true;
}

bool AStarPlanner::setGraph(const PlannerOptions &options) {
    //Generate roadmap
    RoadmapOptions roadmap_options = RoadmapOptions(options.num_samples, options.max_dist);
    auto roadmap = std::make_shared<Roadmap>(instance_, robot_id_);
    roadmap->init(roadmap_options);
    if (!roadmap->buildRoadmap()) {
        return false;
    }
    graph_ = roadmap->queryRoadmap();

    //add/connect start and goal vertices
    start_ = graph_->addVertex(start_pose_);
    goal_ = graph_->addVertex(goal_pose_);
    if (roadmap->validateMotion(start_, goal_)) {
        graph_->addEdge(start_, goal_);
    }
    if (!graph_->sameComponent(start_, goal_)) {
        return false;
    }
    return true;
}

/**
 * @brief Perform the planning process
 * @param options: Planner options
 */
bool AStarPlanner::plan(const PlannerOptions &options) {
    //if start and goal are the same
    if (!init(options)) {
        best_cost_ = 0.0;
        solution_.robot_id = robot_id_;
        solution_.times.clear();
        solution_.trajectory.clear();
        solution_.times.push_back(0.0);
        solution_.trajectory.push_back(goal_pose_);
        solution_.cost = 0.0;
        return true;
    }
    
    //try to generate valid roadmap
    int count = 10;
    while (count--) {
        if (setGraph(options)) {
            break;
        }
    }

    //A* search
    std::priority_queue<std::pair<double, std::shared_ptr<Vertex>>, std::vector<std::pair<double, std::shared_ptr<Vertex>>>, std::greater<std::pair<double, std::shared_ptr<Vertex>>>> open;
    std::unordered_map<std::shared_ptr<Vertex>, double> gScore;
    std::unordered_map<std::shared_ptr<Vertex>, double> fScore;
    std::unordered_map<std::shared_ptr<Vertex>, std::shared_ptr<Vertex>> cameFrom;

    gScore[start_] = 0.0;
    fScore[start_] = instance_->computeDistance(start_->pose, goal_->pose);
    open.push({fScore[start_], start_});

    while (!open.empty()) {
        auto current = open.top().second;
        open.pop();

        if (isSameVertex(current, goal_)) {
            best_cost_ = gScore[current];
            
            //reconstruct the path using cameFrom
            std::vector<RobotPose> trajectory;
            trajectory.push_back(goal_->pose);
            while (cameFrom.find(current) != cameFrom.end()) {
                current = cameFrom[current];
                trajectory.push_back(current->pose);
            }
            std::reverse(trajectory.begin(), trajectory.end());

            solution_.robot_id = robot_id_;
            solution_.trajectory = trajectory;
            solution_.times.clear();
            solution_.cost = best_cost_;
            return true;
        }

        for (auto neighbor : graph_->getNeighbors(current)) {
            double tentative_gScore = gScore[current] + instance_->computeDistance(current->pose, neighbor->pose);
            if (gScore.find(neighbor) == gScore.end() || tentative_gScore < gScore[neighbor]) {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentative_gScore;
                fScore[neighbor] = gScore[neighbor] + instance_->computeDistance(neighbor->pose, goal_->pose);
                open.push({fScore[neighbor], neighbor});
            }
        }
    }

    if (open.empty()) {
        return false;
    }
}

/**
 * @brief Get the plan
 * @param solution: Robot trajectory
 */
bool AStarPlanner::getPlan(RobotTrajectory &solution) const {
    solution = solution_;
    return true;
}