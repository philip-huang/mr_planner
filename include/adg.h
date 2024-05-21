#ifndef MR_PLANNER_ADG_H
#define MR_PLANNER_ADG_H

#include "tpg.h"
#include "task.h"

namespace TPG {
    
    class ShortcutSamplerADG : public ShortcutSampler {
    public:
        ShortcutSamplerADG(const TPGConfig &config, const ActivityGraph &act_graph,       
            const std::vector<std::vector<std::shared_ptr<Node>>> &intermediate_nodes);
        void init(const std::vector<std::shared_ptr<Node>> &start_nodes,
            const std::vector<int> &numNodes) override;
    
    private:
        bool sampleUniform(Shortcut &shortcut) override;

        std::vector<std::vector<std::shared_ptr<Node>>> nodes_;
        std::vector<std::vector<int>> act_lengths_;
        std::vector<std::vector<int>> act_ids_;
        ActivityGraph act_graph_;
        std::vector<std::vector<std::shared_ptr<Node>>> intermediate_nodes_;
    };

    class ADG: public TPG {
    friend class boost::serialization::access;
    template <class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & boost::serialization::base_object<TPG>(*this);
        ar & act_graph_;
    }
    public:
        ADG() = default;
        ADG (const ActivityGraph &activity_graph) : act_graph_(activity_graph) {};

        bool init_from_tpgs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs);
        virtual void checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut, std::vector<Eigen::MatrixXi> &col_matrix) const override;
        virtual void update_joint_states(const std::vector<double> &joint_states, int robot_id) override;
        virtual bool moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) override;
        virtual bool moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) override;

    private:
        ActivityGraph act_graph_;
        std::shared_ptr<MoveitInstance> instance_;
        std::vector<std::vector<std::shared_ptr<Node>>> intermediate_nodes_;
        std::vector<std::unique_ptr<std::atomic_int>> executed_acts_;
    };
}

#endif //MR_PLANNER_ADG_H