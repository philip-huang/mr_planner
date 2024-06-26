#ifndef MR_PLANNER_PLANNER_H
#define MR_PLANNER_PLANNER_H

#include "instance.h"
#include "query.h"
#include <memory>
#include <vector>

// Abstract planner class
class AbstractPlanner {
    public:
        // Initialize the planner with a specific planning problem instance
        AbstractPlanner(std::shared_ptr<PlanInstance> instance) : instance_(instance) {
            num_robots_ = instance->getNumberOfRobots();
        }
        
        // Perform the planning process
        virtual bool plan(const PlannerOptions &options) = 0;

        // Retrieve the plan (if needed, depending on your design, this could return a path, a series of actions, etc.)
        // For simplicity, this could return a boolean indicating success for now,
        // but you might want to define a more complex structure for the plan itself.
        virtual bool getPlan(std::vector<RobotTrajectory> &solution) const = 0;

        virtual ~AbstractPlanner() = default;

    protected:
        int num_robots_;
        std::shared_ptr<PlanInstance> instance_;
};



// Example of a concrete planner class that implements the AbstractPlanner interface
// This is where you would implement specific planning algorithms
// class PriorityPlanner : public AbstractPlanner {
//     public:
//         PriorityPlanner(std::shared_ptr<PlanInstance> instance);

//         virtual bool plan(const PlannerOptions &options) override;

//         virtual bool getPlan(std::vector<RobotTrajectory> &solution) const override;

//     protected:
//         std::vector<SingleAgentPlannerPtr> agent_planners_;
//         std::vector<RobotTrajectory> solution_;
//         bool solved = false;
//         bool random_order_ = false;
// };

class Conflict {
    public:
        Conflict(int a1, int a2, int t) : agent1(a1), agent2(a2), timestep(t) {}
        int agent1;
        int agent2;
        int timestep;
};

class CBSNode {
    public:
        CBSNode(const std::vector<RobotTrajectory> &solution) : solution(solution) {}
        PlannerOptions options;
        std::vector<RobotTrajectory> solution;
        std::vector<Conflict> conflicts;
};

class CBSPlanner : public AbstractPlanner {
    public:
        CBSPlanner::CBSPlanner(std::shared_ptr<PlanInstance> instance) : 
                        AbstractPlanner(instance) {}

        virtual bool planSingleAgent(const PlannerOptions &options, int robot_id, RobotTrajectory &solution);

        std::vector<Conflict> getConflicts(const std::vector<RobotTrajectory> &solution);

        virtual bool plan(const PlannerOptions &options) override;

        virtual bool getPlan(std::vector<RobotTrajectory> &solution) const override;

    protected:
        int num_robots_;
        std::shared_ptr<PlanInstance> instance_;
        std::vector<RobotTrajectory> solution_;
        bool solved = false;
};

// // utils
// bool convertSolution(std::shared_ptr<PlanInstance> instance,
//                     const moveit::planning_interface::MoveGroupInterface::Plan &plan,
//                     std::vector<RobotTrajectory> &solution);

#endif // MR_PLANNER_PLANNER_H
