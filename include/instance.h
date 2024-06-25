#ifndef MR_PLANNER_INSTANCE_H
#define MR_PLANNER_INSTANCE_H

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <actionlib/client/simple_action_client.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_fcl/collision_detector_allocator_fcl.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>

#include <std_msgs/ColorRGBA.h>

#include <boost/serialization/vector.hpp>
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <memory>
#include <vector>
#include <random>
#include <Eigen/Geometry>
#include <chrono>
#include <atomic>
#include <thread>
#include <unordered_map>
#include <set>

// Abstract base class for the planning scene interface

struct Object  {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & name;
        ar & state;
        ar & parent_link;
        ar & robot_id;
        ar & x;
        ar & y;
        ar & z;
        ar & qx;
        ar & qy;
        ar & qz;
        ar & qw;
        ar & shape;
        ar & radius;
        ar & length;
        ar & width;
        ar & height;
        ar & mesh_path;
    }    
    
    enum State {
        Static = 0,
        Attached = 1,
        Supported = 2,
    };
    enum Shape {
        Box = 0,
        Sphere = 1,
        Cylinder = 2,
        Mesh = 3,
    };

    Object() = default;
    Object(const std::string &name, const std::string& parent_link, State state, double x, double y, double z, double qx, double qy, double qz, double qw):
        name(name), parent_link(parent_link), state(state), x(x), y(y), z(z), qx(qx), qy(qy), qz(qz), qw(qw) 
        {}
    
    std::string name;
    // mode of the object
    State state;
    std::string parent_link; 
    int robot_id;

    // geometry of the object
    double x, y, z;
    double qx = 0, qy = 0, qz = 0, qw = 1.0;
    double x_attach, y_attach, z_attach;
    double qx_attach = 0, qy_attach = 0, qz_attach = 0, qw_attach = 0;

    // collision shape of the object
    Shape shape;
    double radius;
    double length; // x
    double width; // y
    double height; // z
    std::string mesh_path;
};

struct RobotMode {

    enum Type {
        Free = 0,
        Carry = 1,
        Hold = 2,
    };
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & type;
        ar & carried_obj;
        ar & held_obj;
        ar & ee_link;
        ar & obj;
    }
    
    Type type = Free;
    std::string carried_obj;
    std::string held_obj;
    std::string ee_link;
    std::shared_ptr<Object> obj;
};

struct RobotPose {
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & robot_id;
        ar & robot_name;
        ar & joint_values;
    }
    int robot_id;
    // RobotMode mode;
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
    virtual bool connect(const RobotPose& a, const RobotPose& b, double col_step_size = 0.1) = 0;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist,  RobotPose& result, double col_step_size = 0.1) = 0;
    virtual bool sample(RobotPose &pose) = 0;
    virtual double getVMax(int robot_id);
    virtual void setVmax(double vmax);
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const = 0;
    virtual void addMoveableObject(const Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void moveObject(const Object& obj) { throw std::runtime_error("Not implemented");};
    virtual void moveRobot(int robot_id, const RobotPose& pose) { throw std::runtime_error("Not implemented");};
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) { throw std::runtime_error("Not implemented");};
    virtual void detachObjectFromRobot(const std::string& name, const RobotPose &pose) { throw std::runtime_error("Not implemented");};
    virtual void updateScene() = 0;
    virtual void resetScene(bool reset_sim) = 0;
    virtual void setPadding(double padding) {throw std::runtime_error("Not implemented");};
    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) { throw std::runtime_error("Not implemented");};
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

    virtual void setRobotDOF(int robot_id, size_t dof);

    virtual size_t getRobotDOF(int robot_id) const {
        return robot_dof_[robot_id];
    
    }

    virtual Object getObject(const std::string& name) const {
        return objects_.at(name);
    }

    virtual std::vector<Object> getAttachedObjects(int robot_id) const {
        std::vector<Object> attached_objects;
        for (const auto& obj : objects_) {
            if (obj.second.robot_id == robot_id && obj.second.state == Object::State::Attached) {
                attached_objects.push_back(obj.second);
            }
        }
        return attached_objects;
    }

protected:
    int num_robots_;
    double v_max_ = 1.0;
    std::vector<RobotPose> start_poses_;
    std::vector<size_t> robot_dof_;
    std::vector<RobotPose> goal_poses_;
    std::vector<std::string> robot_names_;
    std::unordered_map<std::string, Object> objects_;
};

// Concrete implementation using MoveIt
class MoveitInstance : public PlanInstance {
public:
    MoveitInstance(robot_state::RobotStatePtr kinematic_state,
                   const std::string &joint_group_name,
                   planning_scene::PlanningScenePtr planning_scene);
    virtual bool checkCollision(const std::vector<RobotPose> &poses, bool self) const override;
    virtual double computeDistance(const RobotPose& a, const RobotPose &b) const override;
    virtual bool connect(const RobotPose& a, const RobotPose& b, double col_step_size = 0.1) override;
    virtual bool steer(const RobotPose& a, const RobotPose& b, double max_dist, RobotPose& result, double col_step_size = 0.1) override;
    virtual bool sample(RobotPose &pose) override;
    virtual RobotPose interpolate(const RobotPose &a, const RobotPose&b, double t) const override;
    // Implementation of abstract methods using MoveIt functionalities
    virtual void addMoveableObject(const Object& obj) override;
    virtual void moveObject(const Object& obj) override;
    virtual void moveRobot(int robot_id, const RobotPose& pose) override;
    virtual void attachObjectToRobot(const std::string &name, int robot_id, const std::string &link_name, const RobotPose &pose) override;
    virtual void detachObjectFromRobot(const std::string& name, const RobotPose &pose) override;
    virtual void setObjectColor(const std::string &name, double r, double g, double b, double a);
    virtual moveit_msgs::PlanningScene getPlanningSceneDiff() const {
        return planning_scene_diff_;
    }
    virtual void setPlanningSceneDiffClient(ros::ServiceClient &client) {
        planning_scene_diff_client_ = client;
    }
    virtual void updateScene() override;
    virtual void resetScene(bool reset_sim) override;
    virtual void setPadding(double padding) override;

    virtual bool setCollision(const std::string& obj_name, const std::string& link_name, bool allow) override;

private:
    // moveit move_group and planning_scene_interface pointers
    std::string joint_group_name_;
    robot_state::RobotStatePtr kinematic_state_;
    planning_scene::PlanningScenePtr planning_scene_;
    moveit_msgs::PlanningScene original_scene_;

    /* store the planning scene diff temporarily*/
    moveit_msgs::PlanningScene planning_scene_diff_;
    ros::ServiceClient planning_scene_diff_client_;

    // random number generator
    std::mt19937 rng_;

};

#endif // MR_PLANNER_INSTANCE_H
