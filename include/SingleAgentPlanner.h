#ifndef SINGLE_AGENT_PLANNER_H
#define SINGLE_AGENT_PLANNER_H

#include "instance.h" // Include the abstract problem instance definition
#include <memory>
#include <vector>
#include <chrono>

struct RobotTrajectory {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & robot_id;
        ar & trajectory;
        ar & times;
        ar & cost;
    }
    int robot_id;
    std::vector<RobotPose> trajectory;
    std::vector<double> times;
    double cost;
};

typedef std::vector<RobotTrajectory> DynamicObstacles;

struct PlannerOptions {
    double max_planning_time = 1.0;
    int max_planning_iterations = 10000;
    DynamicObstacles obstacles;

    // constructor
    PlannerOptions() = default;
    PlannerOptions(double max_planning_time, int max_planning_iterations)
    {
        this->max_planning_time = max_planning_time;
        this->max_planning_iterations = max_planning_iterations;
    }
};

// Abstract planner class
class SingleAgentPlanner {
public:
    // Initialize the planner with a specific planning problem instance
    SingleAgentPlanner(std::shared_ptr<PlanInstance> instance,
                      int robot_id) : instance_(instance), robot_id_(robot_id)
                     {}

    virtual bool init(const PlannerOptions &options);
    
    // Perform the planning process
    virtual bool plan(const PlannerOptions &options) = 0;

    // Retrieve the plan (if needed, depending on your design, this could return a path, a series of actions, etc.)
    // For simplicity, this could return a boolean indicating success for now,
    // but you might want to define a more complex structure for the plan itself.
    virtual bool getPlan(RobotTrajectory &solution) const = 0;

    virtual ~SingleAgentPlanner() = default;

    virtual bool terminate(const PlannerOptions &options) {
        return false;
    }

protected:
    int robot_id_;
    RobotPose start_pose_;
    RobotPose goal_pose_;
    std::shared_ptr<PlanInstance> instance_;
};

class Vertex {
public:
    Vertex(const RobotPose &pose) : pose(pose) {};

    void setPose(const RobotPose &pose) {
        this->pose = pose;
    }

    void setTime(double time) {
        this->time = time;
        time_set = true;
    }

    void addParent(std::shared_ptr<Vertex> parent) {
        this->parent = parent;
        // find the root of the tree
        this->root = parent;
        while (this->root->parent != nullptr) {
            this->root = this->root->parent;
        }
    }

    void addOtherParent(std::shared_ptr<Vertex> other_parent) {
        this->otherParent = other_parent;

        this->otherRoot = other_parent;
        while (this->otherRoot->parent != nullptr) {
            this->otherRoot = this->otherRoot->parent;
        }
    }

    RobotPose pose;
    double time;
    bool time_set = false;
    std::shared_ptr<Vertex> parent;
    std::shared_ptr<Vertex> otherParent; // if this node is a connection point, it has another parent
    std::shared_ptr<Vertex> root;
    std::shared_ptr<Vertex> otherRoot;
};
 

class Tree {
public:
    Tree() {};

    void addVertex(std::shared_ptr<Vertex> vertex) {
        vertices.push_back(vertex);
    }

    void addRoot(std::shared_ptr<Vertex> root) {
        roots.push_back(root);
        vertices.push_back(root);
    }

    void removeVertex(std::shared_ptr<Vertex> vertex) {
        for (auto it = vertices.begin(); it != vertices.end(); ++it) {
            if (*it == vertex) {
                vertices.erase(it);
                break;
            }
        }
    }

    std::vector<std::shared_ptr<Vertex>> roots;
    std::vector<std::shared_ptr<Vertex>> vertices;
};

enum GrowState {
    ADVANCED,
    TRAPPED,
    REACHED
};

// Example of a concrete planner class that implements the AbstractPlanner interface
// This is where you would implement specific planning algorithms
class STRRT : public SingleAgentPlanner {
public:
    STRRT(std::shared_ptr<PlanInstance> instance,
                      int robot_id);

    virtual bool init(const PlannerOptions &options) override;

    virtual bool plan(const PlannerOptions &options) override;

    virtual bool getPlan(RobotTrajectory &solution) const override;

    virtual bool terminate(const PlannerOptions &options) override;

    bool shouldExpandTime();

    bool sampleConditionally(std::shared_ptr<Vertex> &new_sample);

    void sampleGoal(std::shared_ptr<Vertex> &new_goal);

    void expandTime();

    GrowState extend(const std::shared_ptr<Vertex> &new_sample, Tree &tree, std::shared_ptr<Vertex> &added_sample, bool goalTree);

    GrowState connect(const std::shared_ptr<Vertex> &new_sample, Tree &tree, bool goalTree);

    void update_solution(std::shared_ptr<Vertex> &new_sample, bool goalTree);

    void prune_trees();

    void swap_trees();

    double distanceSpaceTime(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b);

    void nearest(const std::shared_ptr<Vertex> &sample, std::shared_ptr<Vertex> &nearest_vertex, const Tree &tree, bool goalTree);

    bool validateMotion(const std::shared_ptr<Vertex> &a, const std::shared_ptr<Vertex> &b);

private:
    Tree start_tree_;
    Tree goal_tree_;
    Tree* current_tree_ = &start_tree_;
    Tree* other_tree_ = &goal_tree_;
    bool goal_tree = false;

    double min_time_ = 0.0;
    double max_time_ = 0.0;
    double max_deltat_ = 0.5;
    bool time_bounded_ = false;
    double vMax_ = 1.0;
    double col_dt_ = 0.05;
    int numIterations_ = 0;
    int totalTreeSize = 0;
    int numValidSamples_ = 0;
    int batch_size = 200;
    int cur_batch_size = 0;
    std::chrono::time_point<std::chrono::system_clock> start_time_;

    RobotTrajectory solution_;
    double best_cost_ = std::numeric_limits<double>::infinity();
    DynamicObstacles obstacles_;

    // random number generator
    std::mt19937 rng_;
};

typedef std::shared_ptr<SingleAgentPlanner> SingleAgentPlannerPtr;

#endif // SINGLE_AGENT_PLANNER_H
