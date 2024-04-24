#ifndef MR_PLANNER_EXECUTION_H
#define MR_PLANNER_EXECUTION_H

#include <planner.h>
#include <queue>
#include <stack>

typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> TrajectoryClient;

namespace boost {
namespace serialization {

template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void serialize(
    Archive & ar, 
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> & t, 
    const unsigned int file_version
){
    int rows = t.rows(), cols = t.cols();
    ar & rows;
    ar & cols;
    if(rows * cols != t.size())
        t.resize(rows, cols);

    for(int i = 0; i < t.size(); i++)
        ar & t.data()[i];
}

} // namespace serialization
} // namespace boost

namespace TPG {

    struct TPGConfig {
        bool shortcut = true;
        bool random_shortcut = true;
        bool forward_doubleloop = false;
        bool backward_doubleloop = false;
        bool forward_singleloop = true;
        bool ignore_far_collisions = false;
        bool helpful_shortcut = false;
        bool tight_shortcut = true;
        bool tight_shortcut_makespan = true;
        double shortcut_time = 1;
        double dt = 0.1;
        double switch_shortcut = false;
        int ignore_steps = 5;
    };
    
    struct type2Edge;
    struct Node
    {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & pose;
            ar & Type1Next;
            ar & Type2Next;
            ar & Type1Prev;
            ar & Type2Prev;
            ar & timeStep;
            ar & robotId;
            ar & nodeId;
        }

        RobotPose pose; // < The pose of the robot at this Node
        std::shared_ptr<Node> Type1Next;                    ///< Pointer to the next Node of type 1
        std::vector<type2Edge> Type2Next; ///< Vector of pointers to the next Nodes of type 2
        std::shared_ptr<Node> Type1Prev;                    ///< Pointer to the previous Node of type 1
        std::vector<type2Edge> Type2Prev; ///< Vector of pointers to the previous Nodes of type 2
        int timeStep = -1;                       ///< The time step at which this Node exists
        int robotId = -1;                        ///< The ID of the robot at this Node
        int nodeId = -1;                         ///< The ID of the Node
        
        Node() = default;
        Node(int robot_id, int t)
        {
            this->timeStep = t;
            this->robotId = robot_id;
        };

    };

    struct type2Edge
    {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & edgeId;
            ar & switchable;
            ar & nodeFrom;
            ar & nodeTo;
        }

        int edgeId = -1; ///< The ID of the edge
        bool switchable = true;                 ///< Whether this Node is switchable
        bool tight = false;

        std::shared_ptr<Node> nodeFrom; ///< Pointer to the Node from which this edge originates
        std::shared_ptr<Node> nodeTo;   ///< Pointer to the Node to which this edge leads
    };


    class TPG {
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & dt_;
        ar & num_robots_;
        ar & type2Edges_;
        ar & start_nodes_;
        ar & end_nodes_;
        ar & numNodes_;
        ar & solution_;
        ar & pre_shortcut_flowtime_;
        ar & pre_shortcut_makespan_;
        ar & post_shortcut_flowtime_;
        ar & post_shortcut_makespan_;
        ar & t_shortcut_;
        ar & t_init_;
        ar & t_simplify_;
        ar & t_shortcut_check_;
        ar & num_shortcut_checks_;
        ar & collisionCheckMatrix_;
    }
    public:
        TPG() = default;
        void reset();
        virtual bool init(std::shared_ptr<PlanInstance> instance, const std::vector<RobotTrajectory> &solution, const TPGConfig &config);
        virtual bool optimize(std::shared_ptr<PlanInstance> instance, const TPGConfig &config);
        bool saveToDotFile(const std::string &filename) const;
        bool moveit_execute(std::shared_ptr<PlanInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) const;
        bool actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client) const;
        bool moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients);
        void update_joint_states(const std::vector<double> &joint_states, int robot_id);
        void saveStats(const std::string &filename, const std::string &start_pose, const std::string &goal_pose) const;
        void getSolution(std::vector<RobotTrajectory> &solution) const {
            solution = solution_;
        }

        std::shared_ptr<Node> getStartNode(int robot_id) const {
            return (robot_id < start_nodes_.size()) ? start_nodes_[robot_id] : nullptr;
        }

        std::shared_ptr<Node> getEndNode(int robot_id) const {
            return (robot_id < end_nodes_.size()) ? end_nodes_[robot_id] : nullptr;
        }

        int getNumNodes(int robot_id) const {
            return (robot_id < numNodes_.size()) ? numNodes_[robot_id] : 0;
        }

        void getSolutionTraj(int robot_id, RobotTrajectory &traj) const {
            if (robot_id < solution_.size()) {
                traj = solution_[robot_id];
            }
        }

        double getShortcutTime() const {
            return t_shortcut_;
        }

    protected:
        int getTotalNodes() const;
        int getTotalType2Edges() const;
        void getCollisionCheckMatrix(int robot_i, int robot_j, Eigen::MatrixXi &col_matrix) const;
        void updateCollisionCheckMatrix(int robot_i, int robot_j, const Eigen::MatrixXi &col_matrix);
        void transitiveReduction();
        void findShortcuts(std::shared_ptr<PlanInstance> instance, double runtime_limit);
        void findShortcutsRandom(std::shared_ptr<PlanInstance> instance, double runtime_limit);
        void findEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end);
        void findLatestReachTime(std::vector<std::vector<int>> &reached_t, const std::vector<int> &reached_end);
        void findTightType2Edges(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<int>> &latest_t);
        void findFlowtimeMakespan(double &flowtime, double &makespan);
        bool preCheckShortcuts(std::shared_ptr<PlanInstance> instance, std::shared_ptr<Node> ni, std::shared_ptr<Node> nj,
            const std::vector<int> &earliest_t, const std::vector<int> &latest_t) const;
        bool checkShortcuts(std::shared_ptr<PlanInstance> instance, std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, 
            std::vector<RobotPose> &shortcut_path, std::vector<Eigen::MatrixXi> &col_matrix) const;
        void switchShortcuts();
        void updateTPG(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, 
            const std::vector<RobotPose> &shortcut_path, const std::vector<Eigen::MatrixXi> &col_matrix);
        bool dfs(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, std::vector<std::vector<bool>> &visited) const;
        bool bfs(std::shared_ptr<Node> ni, std::vector<std::vector<bool>> &visited, bool forward) const;
        bool hasCycle() const;
        void setSyncJointTrajectory(trajectory_msgs::JointTrajectory &joint_traj, double &flowtime, double &makespan) const;
        void moveit_async_execute_thread(const std::vector<std::string> &joint_names, ros::ServiceClient &clients, int robot_id);

        TPGConfig config_;
        double dt_ = 0.1;
        std::vector<std::vector<Eigen::MatrixXi>>  collisionCheckMatrix_; // 1 if no collision, 0 if collision
        int num_robots_;
        int idType2Edges_ = 0;
        std::vector<type2Edge> type2Edges_;
        std::vector<std::shared_ptr<Node>> start_nodes_;
        std::vector<std::shared_ptr<Node>> end_nodes_;
        std::vector<int> numNodes_;
        std::vector<RobotTrajectory> solution_;

        std::vector<std::vector<double>> joint_states_;
        std::vector<std::unique_ptr<std::atomic_int>> executed_steps_;

        // stats
        double pre_shortcut_flowtime_ = 0.0;
        double pre_shortcut_makespan_ = 0.0;
        double post_shortcut_flowtime_ = 0.0;
        double post_shortcut_makespan_ = 0.0;
        double t_shortcut_ = 0.0;
        double t_init_ = 0.0;
        double t_simplify_ = 0.0;
        double t_shortcut_check_ = 0.0;
        int num_shortcut_checks_ = 0;
        int num_valid_shortcuts_ = 0;
        double flowtime_improv_ = 0.0;
        
    };

    struct Activity {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & activity_id;
            ar & type;
            ar & type2_deps;
            ar & type1_dep;
            ar & start_node;
            ar & end_node;
        }

        enum Type {
            home = 0,
            pick_tilt_up = 1,
            pick_up = 2,
            pick_down = 3,
            pick_twist = 4,
            pick_twist_up = 5,
            drop_tilt_up = 7,
            drop_up = 8,
            drop_down = 9,
            drop_twist = 10,
            drop_twist_up = 11,
            support = 13,
        };

        int robot_id;
        int activity_id;
        Type type;
        std::vector<std::shared_ptr<Activity>> type2_deps;
        std::shared_ptr<Activity> type1_dep;
        std::shared_ptr<Node> start_node;
        std::shared_ptr<Node> end_node;
    };

    class ADG: public TPG {
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<TPG>(*this);
        ar & activities_;
    }
    public:
        ADG(int num_robots);
        
        void add_activity(int robot_id, Activity::Type type);

        void add_activity(int robot_id, Activity::Type type, std::shared_ptr<Activity> type2_dep);
    
        void add_trajectory(int robot_id, int activity_id, const RobotTrajectory &trajectory);

        bool init(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs);

        std::shared_ptr<Activity> get_activity(int robot_id, int activity_id);
        std::shared_ptr<Activity> get_last_activity(int robot_id, Activity::Type type);

    private:
        std::vector<std::vector<std::shared_ptr<Activity>>> activities_;
    };
}

#endif // MR_PLANNER_EXECUTION_H