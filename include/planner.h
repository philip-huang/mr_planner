#ifndef MR_PLANNER_PLANNER_H
#define MR_PLANNER_PLANNER_H

#include "instance.h" // Include the abstract problem instance definition
#include "SingleAgentPlanner.h"
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
class PriorityPlanner : public AbstractPlanner {
public:
    PriorityPlanner(std::shared_ptr<PlanInstance> instance);

    virtual bool plan(const PlannerOptions &options) override;

    virtual bool getPlan(std::vector<RobotTrajectory> &solution) const override;

protected:
    std::vector<SingleAgentPlannerPtr> agent_planners_;
    std::vector<RobotTrajectory> solution_;
    bool solved = false;
    bool random_order_ = false;
};

#endif // MR_PLANNER_PLANNER_H
