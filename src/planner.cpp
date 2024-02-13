#include "planner.h"
#include "instance.h"

PriorityPlanner::PriorityPlanner(std::shared_ptr<PlanInstance> instance) : AbstractPlanner(instance) {
    
}

bool PriorityPlanner::plan() {
    // Implement the planning algorithm using problem_instance_

    // set a randomized order
    std::vector<int> order;
    for (int i = 0; i < num_robots_; i++) {
        order.push_back(i);
    }
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(order.begin(), order.end(), g);

    // iterate through the robots in the randomized order
    std::vector<RobotTrajectory> solution;
    for (int i = 0; i < num_robots_; i++) {
        int robot_id = order[i];
        
        auto planner = std::make_shared<STRRT>(instance_, robot_id);
        PlannerOptions options;
        options.obstacles = solution;
        
        bool solved = planner->plan(options);
        if (solved) {
            RobotTrajectory traj;
            planner->getPlan(traj);
            solution.push_back(traj);
        } else {
            return false; // Return false if planning fails
        }
    }

    return true; // Return true if planning succeeds
}

bool PriorityPlanner::getPlan(std::vector<RobotTrajectory> &solution) const {
    // Retrieve the plan. For now, we just indicate success or failure.
    return true; // Placeholder
}