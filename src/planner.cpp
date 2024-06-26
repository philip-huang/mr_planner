#include "planner.h"
#include "instance.h"
#include "query.h"
#include "logger.h"

/*
PriorityPlanner::PriorityPlanner(std::shared_ptr<PlanInstance> instance) : AbstractPlanner(instance) {
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
        option_i.max_planning_time = options.max_planning_time / num_robots_;
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

bool convertSolution(std::shared_ptr<PlanInstance> instance,
                    const moveit::planning_interface::MoveGroupInterface::Plan &plan,
                    std::vector<RobotTrajectory> &solution) {
    // Convert a MoveIt plan to a RobotTrajectory
    int numRobots = instance->getNumberOfRobots();
    solution.resize(numRobots);
    for (int i = 0; i < numRobots; i++) {
        solution[i].robot_id = i;
    }

    for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++) {
        int st = 0;
        double timeDilation = 1;
        for (int j = 0; j < numRobots; j++) {
            RobotPose pose = instance->initRobotPose(j);
            assert (st + pose.joint_values.size() <= plan.trajectory_.joint_trajectory.points[i].positions.size());
            for (int k = 0; k < pose.joint_values.size(); k++) {
                pose.joint_values[k] = plan.trajectory_.joint_trajectory.points[i].positions[k + pose.joint_values.size()*j];
            }

            if (i > 0) {
                double dt = plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec() - plan.trajectory_.joint_trajectory.points[i-1].time_from_start.toSec();
                double speed = std::abs(instance->computeDistance(solution[j].trajectory.back(), pose)) / dt;
                if (speed > instance->getVMax(j)) {
                    timeDilation = std::max(timeDilation, speed / instance->getVMax(j));
                }
            }
            solution[j].trajectory.push_back(pose);
            st += pose.joint_values.size();
        }

        for (int j = 0; j < numRobots; j++) {
            if (i > 0) {
                double dt = plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec() - plan.trajectory_.joint_trajectory.points[i-1].time_from_start.toSec();
                dt = dt * timeDilation;
                solution[j].times.push_back(solution[j].times.back() + dt);
            }
            else {
                solution[j].times.push_back(plan.trajectory_.joint_trajectory.points[i].time_from_start.toSec());
            }
        }
    }


    for (int i = 0; i < numRobots; i++) {
        solution[i].cost = solution[i].times.back();
    }

    return true;
}
*/

/**
 * @brief Plan for a single robot using AStar
 * @param options: Planner options
 * @param robot_id: Id of the robot
 * @param solution: Solution for the robot
 */
bool CBSPlanner::planSingleAgent(const PlannerOptions &options, int robot_id, RobotTrajectory &solution) {
    auto planner = std::make_shared<AStarPlanner>(instance_, robot_id);
    if (planner->plan(options)) {
        planner->getPlan(solution);
        return true;
    } else {
        log("Failed to plan for robot " + std::to_string(robot_id) + "!", LogLevel::ERROR);
        return false;
    }
}

/**
 * @brief Plan the trajectory for the robots using conflict-based search
 */
bool CBSPlanner::plan(const PlannerOptions &options) {
    //Plan each robots solution using AStar
    std::vector<RobotTrajectory> solution;
    for (int i = 0; i < num_robots_; i++) {
        planSingleAgent(options, i, solution[i]);
    }

    //Create OOPEN set of options
    std::vector<CBSNode> open;

    //Create the initial node
    CBSNode node(solution);
    node.options = options;
    node.conflicts = getConflicts(solution);
    open.push_back(node);

    //Iterate through the conflicts
    while (open.size() != 0) {
        //Get the first node
        CBSNode node = open[0];
        open.erase(open.begin());

        //If no conflicts, return the solution
        if (node.conflicts.size() == 0) {
            solution_ = node.solution;
            return true;
        }

        //Resolve the first conflict
        Conflict conflict = node.conflicts[0];
        std::vector<RobotTrajectory> newSolution;;
        

    }
}