#ifndef MR_PLANNER_ADG_H
#define MR_PLANNER_ADG_H

#include "tpg.h"

namespace TPG {
    
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

#endif //MR_PLANNER_ADG_H