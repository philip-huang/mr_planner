#ifndef QUERY_H
#define QUERY_H

#include "instance.h"
#include "roadmap.h"
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

    int num_samples = 1000;
    double max_dist = 1.0;

    PlannerOptions() = default;
    PlannerOptions(double max_planning_time, int max_planning_iterations, 
                   int num_samples, double max_dist)
    {
        this->max_planning_time = max_planning_time;
        this->max_planning_iterations = max_planning_iterations;
        this->num_samples = num_samples;
        this->max_dist = max_dist;
    }
};

class Query {
    public:
        // Initialize the planner with a specific planning problem instance
        Query(std::shared_ptr<PlanInstance> instance, int robot_id) : 
                        instance_(instance), robot_id_(robot_id) {}

        virtual bool init(const PlannerOptions &options) = 0;
        
        virtual bool plan(const PlannerOptions &options) = 0;

        virtual bool getPlan(RobotTrajectory &solution) const = 0;

        virtual bool terminate(const PlannerOptions &options) {
            return true;
        }

    protected:
        std::shared_ptr<PlanInstance> instance_;
        int robot_id_;
        DynamicObstacles obstacles_;
        
};

class AStarPlanner : public Query {
    public:
        AStarPlanner(std::shared_ptr<PlanInstance> instance, int robot_id) : 
                        Query(instance, robot_id) {}

        bool init(const PlannerOptions &options) override;

        bool setGraph(const PlannerOptions &options);

        bool isSameVertex(std::shared_ptr<Vertex> u, std::shared_ptr<Vertex> v) {
            return instance_->computeDistance(u->pose, v->pose) < 1e-6;
        }

        bool plan(const PlannerOptions &options) override;

        bool getPlan(RobotTrajectory &solution) const override;

    protected:
        std::shared_ptr<PlanInstance> instance_;
        int robot_id_;


        double max_planning_time_;
        int max_planning_iterations_;
        DynamicObstacles obstacles_;

        double vMax_ = 1.0;
        double col_dt_ = 0.05;
        double min_time_;
        double max_time_;
        std::chrono::time_point<std::chrono::system_clock> start_time_;
        
        RobotTrajectory solution_;
        double best_cost_ = std::numeric_limits<double>::infinity();
        
        RobotPose start_pose_;
        RobotPose goal_pose_;
        std::shared_ptr<Graph> graph_;
        std::shared_ptr<Vertex> start_;
        std::shared_ptr<Vertex> goal_;


};

#endif // QUERY_H