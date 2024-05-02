#ifndef MR_PLANNER_ADG_H
#define MR_PLANNER_ADG_H

#include "tpg.h"

namespace TPG {
    
    struct ObjectNode;
    typedef std::shared_ptr<ObjectNode> ObjPtr;
    struct Activity {
        friend class boost::serialization::access;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & robot_id;
            ar & activity_id;
            ar & type;
            ar & type2_prev;
            ar & type2_next;
            ar & type1_prev;
            ar & type1_next;
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

        static const std::map<Type, std::string> enumStringMap;

        Activity() = default;
        Activity(int robot_id, Type type) : robot_id(robot_id), type(type) {}
        void add_type2_dep(std::shared_ptr<Activity> type2_dep) {
            this->type2_prev.push_back(type2_dep);
        }
        void add_type2_next(std::shared_ptr<Activity> type2_next) {
            this->type2_next.push_back(type2_next);
        }

        std::string type_string() const {
            return enumStringMap.at(type);
        }

        int robot_id;
        int activity_id;
        Type type;
        std::vector<std::shared_ptr<Activity>> type2_prev;
        std::vector<std::shared_ptr<Activity>> type2_next;
        std::shared_ptr<Activity> type1_prev;
        std::shared_ptr<Activity> type1_next;
        std::shared_ptr<Node> start_node;
        std::shared_ptr<Node> end_node;
        std::vector<ObjPtr> obj_placed;
        std::vector<ObjPtr> obj_picked;
    };

    struct ObjectNode {
        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & obj;
            ar & prev_place;
            ar & next_pick;
        }
        ObjectNode(const Object &obj, int id) : obj(obj), obj_node_id(id) {}

        Object obj;
        int obj_node_id;
        std::shared_ptr<Activity> prev_place;
        std::shared_ptr<Activity> next_pick;
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
    
        void add_activity(int robot_id, Activity::Type type, const Object &obj);

        void add_trajectory(int robot_id, int activity_id, const RobotTrajectory &trajectory);

        bool init(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs);

        std::shared_ptr<Activity> get_activity(int robot_id, int activity_id);
        std::shared_ptr<Activity> get_last_activity(int robot_id, Activity::Type type);

        bool saveADGToDotFile(const std::string &filename);

    private:
        std::vector<std::vector<std::shared_ptr<Activity>>> activities_;
        std::vector<ObjPtr> obj_nodes_;
    };
}

#endif //MR_PLANNER_ADG_H