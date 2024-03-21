#include "planner.h"
#include "instance.h"
#include "logger.h"

PriorityPlanner::PriorityPlanner(std::shared_ptr<PlanInstance> instance) : AbstractPlanner(instance) {
    setLogLevel(LogLevel::INFO);
}

bool PriorityPlanner::plan(const PlannerOptions &options) {
    // Implement the planning algorithm using problem_instance_

    // set a randomized order
    std::vector<int> order;
    for (int i = 0; i < num_robots_; i++) {
        order.push_back(i);
    }
    if (random_order_) {
        std::random_device rd;
        std::mt19937 g(rd());
        std::shuffle(order.begin(), order.end(), g);
    }

    // iterate through the robots in the randomized order
    std::vector<RobotTrajectory> solution;
    for (int i = 0; i < num_robots_; i++) {
        int robot_id = order[i];
        
        auto planner = std::make_shared<STRRT>(instance_, robot_id);
        PlannerOptions option_i = options;
        option_i.obstacles = solution;
        
        bool solved = planner->plan(option_i);
        if (solved) {
            RobotTrajectory traj;
            planner->getPlan(traj);
            solution.push_back(traj);
            log("Found plan for robot " + std::to_string(robot_id), LogLevel::HLINFO);
        } else {
            log("Failed to plan for robot " + std::to_string(robot_id) + "!", LogLevel::ERROR);
            return false; // Return false if planning fails
        }
    }

    solution_ = solution;
    solved = true;
    return true; // Return true if planning succeeds
}

bool PriorityPlanner::getPlan(std::vector<RobotTrajectory> &solution) const {
    // Retrieve the plan.
    if (!solved) {
        return false;
    }
    solution = solution_;
    return true;
}