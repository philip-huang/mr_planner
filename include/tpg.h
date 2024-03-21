#ifndef MR_PLANNER_EXECUTION_H
#define MR_PLANNER_EXECUTION_H

#include <planner.h>
#include <queue>

typedef actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> TrajectoryClient;

namespace TPG {
    struct type2Edge;
    struct Node
    {
        RobotPose pose; // < The pose of the robot at this Node
        std::shared_ptr<Node> Type1Next;                    ///< Pointer to the next Node of type 1
        std::vector<type2Edge> Type2Next; ///< Vector of pointers to the next Nodes of type 2
        std::shared_ptr<Node> Type1Prev;                    ///< Pointer to the previous Node of type 1
        std::vector<type2Edge> Type2Prev; ///< Vector of pointers to the previous Nodes of type 2
        int timeStep = -1;                       ///< The time step at which this Node exists
        int robotId = -1;                        ///< The ID of the robot at this Node
        int nodeId = -1;                         ///< The ID of the Node
        
        Node(int robot_id, int t)
        {
            this->timeStep = t;
            this->robotId = robot_id;
        };

    };

    struct type2Edge
    {
        int edgeId = -1; ///< The ID of the edge

        std::shared_ptr<Node> nodeFrom; ///< Pointer to the Node from which this edge originates
        std::shared_ptr<Node> nodeTo;   ///< Pointer to the Node to which this edge leads
    };


    class TPG {
    public:
        TPG() = default;
        bool init(std::shared_ptr<PlanInstance> instance, const std::vector<RobotTrajectory> &solution);
        bool saveToDotFile(const std::string &filename) const;
        bool moveit_execute(std::shared_ptr<PlanInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) const;
        bool actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client) const;

    private:
        int getTotalNodes() const;
        int getTotalType2Edges() const;
        void getCollisionCheckMatrix(int robot_i, int robot_j, Eigen::MatrixXi &col_matrix) const;
        void updateCollisionCheckMatrix(int robot_i, int robot_j, const Eigen::MatrixXi &col_matrix);
        void transitiveReduction();
        void findShortcuts(std::shared_ptr<PlanInstance> instance);
        bool checkShortcuts(std::shared_ptr<PlanInstance> instance, std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, 
            std::vector<RobotPose> &shortcut_path, std::vector<Eigen::MatrixXi> &col_matrix) const;
        void updateTPG(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, 
            const std::vector<RobotPose> &shortcut_path, const std::vector<Eigen::MatrixXi> &col_matrix);
        bool dfs(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, std::vector<std::vector<bool>> &visited) const;
        bool bfs(std::shared_ptr<Node> ni, std::vector<std::vector<bool>> &visited, bool forward) const;
        void setSyncJointTrajectory(trajectory_msgs::JointTrajectory &joint_traj) const;
        
        double dt_ = 0.01;
        std::vector<std::vector<Eigen::MatrixXi>>  collisionCheckMatrix_; // 1 if no collision, 0 if collision
        int num_robots_;
        int idType2Edges_ = 0;
        std::vector<type2Edge> type2Edges_;
        std::vector<std::shared_ptr<Node>> start_nodes_;
        std::vector<std::shared_ptr<Node>> end_nodes_;
        std::vector<int> numNodes_;
        std::vector<RobotTrajectory> solution_;
        
    };
}

#endif // MR_PLANNER_EXECUTION_H