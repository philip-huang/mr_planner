#ifndef MR_PLANNER_EXECUTION_H
#define MR_PLANNER_EXECUTION_H

#include <planner.h>
#include <task.h>
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
        bool biased_sample = false;
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

    enum class CollisionType {
        UNKNOWN = -1,
        NONE = 0,
        STATIC = 1,
        ROBOT = 2,
        NO_NEED = 3,
        UNTIGHT = 4,
    };
    
    struct Shortcut {
        std::weak_ptr<Node> ni;
        std::weak_ptr<Node> nj;
        std::vector<RobotPose> path;
        std::weak_ptr<Node> n_robot_col;
        std::shared_ptr<Activity> activity;
        CollisionType col_type = CollisionType::UNKNOWN;

        Shortcut() = default;
        Shortcut(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj) {
            this->ni = ni;
            this->nj = nj;
        }

        bool expired() const {
            return ni.expired() || nj.expired();
        };
        
        int robot_id() const {
            return (!expired()) ? ni.lock()->robotId : -1;
        };

    };

    class ShortcutSampler {
    public:
        ShortcutSampler(const TPGConfig &config);
        virtual void init(const std::vector<std::shared_ptr<Node>> &start_nodes, const std::vector<int> &numNodes);

        virtual bool sample(Shortcut &shortcut);
        virtual void updateFailedShortcut(const Shortcut &shortcut);
    
    protected:
        virtual bool sampleUniform(Shortcut &shortcut);
        virtual bool sampleBiased(Shortcut &shortcut);
        virtual void resetFailedShortcuts();

        bool biased_ = false;
        int num_robots_ = 0;
        std::vector<std::vector<std::shared_ptr<Node>>> nodes_;
        std::vector<int> numNodes_;
        std::vector<Shortcut> failed_shortcuts_;
        std::vector<Shortcut> failed_shortcuts_static_;
        std::vector<Shortcut> already_shortcuts_;
        std::vector<Shortcut> untight_shortcuts_;
        std::vector<std::vector<double>> sample_prob_;
        double scale_ = 15.0;
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
        ar & num_valid_shortcuts_;
        ar & flowtime_improv_;
        ar & collisionCheckMatrix_;
    }
    public:
        TPG() = default;
        // copy constructor
        TPG(const TPG &tpg);
        virtual void reset();
        virtual bool init(std::shared_ptr<PlanInstance> instance, const std::vector<RobotTrajectory> &solution, const TPGConfig &config);
        virtual bool optimize(std::shared_ptr<PlanInstance> instance, const TPGConfig &config);
        virtual bool saveToDotFile(const std::string &filename) const;
        virtual bool moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group);
        virtual bool actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client);
        virtual bool moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients);
        virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id);
        virtual void saveStats(const std::string &filename, const std::string &start_pose, const std::string &goal_pose) const;
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
        virtual void findShortcutsRandom(std::shared_ptr<PlanInstance> instance, double runtime_limit);
        void findEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end);
        void findLatestReachTime(std::vector<std::vector<int>> &reached_t, const std::vector<int> &reached_end);
        void findTightType2Edges(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<int>> &latest_t);
        void findFlowtimeMakespan(double &flowtime, double &makespan);
        void preCheckShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
            const std::vector<int> &earliest_t, const std::vector<int> &latest_t) const;
        virtual void checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut, std::vector<Eigen::MatrixXi> &col_matrix) const;
        void switchShortcuts();
        void updateTPG(const Shortcut &shortcut, const std::vector<Eigen::MatrixXi> &col_matrix);
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
        std::unique_ptr<ShortcutSampler> shortcut_sampler_;

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

}

#endif // MR_PLANNER_EXECUTION_H