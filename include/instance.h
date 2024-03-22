#ifndef MR_PLANNER_INSTANCE_H
#define MR_PLANNER_INSTANCE_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <memory>
#include <vector>
#include <random>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <thread>

// Abstract base class for the planning scene interface

struct RobotPose {
    int robot_id;
    std::string robot_name; // same as group name in moveit
    std::vector<double> joint_values;
};

class PlanInstance {
public:
    virtual void setNumberOfRobots(int num_robots);
    virtual void setRobotNames(const std::vector<std::string>& robot_names) {
        robot_names_ = robot_names;
    }
    virtual void setStartPose(int robot_id, const std::vector<double>& pose);
    virtual void setGoalPose(int robot_id, const std::vector<double>& pose);
    virtual bool checkCollision(const std::vector<RobotPose> &poses, bool self) const = 0;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b) const = 0;
    virtual bool connect(const RobotPose& a, const RobotPose& b) = 0;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist,  RobotPose& result) = 0;
    virtual bool sample(RobotPose &pose) = 0;
    virtual double getVMax(int robot_id);
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const = 0;
    // Additional methods for future functionalities can be added here
    virtual ~PlanInstance() = default;

    virtual int getNumberOfRobots() const {
        return num_robots_;
    }

    virtual std::vector<RobotPose> getStartPoses() const {
        return start_poses_;
    }

    virtual std::vector<RobotPose> getGoalPoses() const {
        return goal_poses_;
    }

    virtual RobotPose getStartPose(int robot_id) const {
        assert (robot_id < start_poses_.size());
        return start_poses_[robot_id];
    }

    virtual RobotPose getGoalPose(int robot_id) const {
        assert (robot_id < goal_poses_.size());
        return goal_poses_[robot_id];
    }

    virtual RobotPose initRobotPose(int robot_id) const;

protected:
    int num_robots_;
    std::vector<RobotPose> start_poses_;
    std::vector<RobotPose> goal_poses_;
    std::vector<std::string> robot_names_;
};

// Concrete implementation using MoveIt
class MoveitInstance : public PlanInstance {
public:
    MoveitInstance(robot_state::RobotStatePtr kinematic_state,
                   std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group,
                   planning_scene::PlanningScenePtr planning_scene);
    void updatePlanningScene(planning_scene::PlanningScenePtr planning_scene) {
        planning_scene_ = planning_scene;
    }
    virtual bool checkCollision(const std::vector<RobotPose> &poses, bool self) const override;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b) const override;
    virtual bool connect(const RobotPose& a, const RobotPose& b) override;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist, RobotPose& result) override;
    virtual bool sample(RobotPose &pose) override;
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const override;
    // Implementation of abstract methods using MoveIt functionalities

private:
    // moveit move_group and planning_scene_interface pointers
    robot_state::RobotStatePtr kinematic_state_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    planning_scene::PlanningScenePtr planning_scene_;

    // random number generator
    std::mt19937 rng_;

};

#endif // MR_PLANNER_INSTANCE_H
