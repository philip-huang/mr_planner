#include "tpg.h"
#include "logger.h"

namespace TPG {

ADG::ADG(int num_robots) {
    num_robots_ = num_robots;
    activities_.resize(num_robots);

}

void ADG::add_activity(int robot_id, Activity::Type type) {
    std::shared_ptr<Activity> activity = std::make_shared<Activity>();
    activity->robot_id = robot_id;
    activity->type = type;
    activity->activity_id = activities_[robot_id].size();
    activities_[robot_id].push_back(activity);
}

void ADG::add_activity(int robot_id, Activity::Type type, std::shared_ptr<Activity> type2_dep) {
    assert(type2_dep != nullptr);
    std::shared_ptr<Activity> activity = std::make_shared<Activity>();
    activity->robot_id = robot_id;
    activity->type = type;
    activity->type2_deps.push_back(type2_dep);
    activity->activity_id = activities_[robot_id].size();
    activities_[robot_id].push_back(activity);
}

std::shared_ptr<Activity> ADG::get_activity(int robot_id, int activity_id) {
    if (activity_id >= activities_[robot_id].size()) {
        return nullptr;
    }
    return activities_[robot_id][activity_id];
}

std::shared_ptr<Activity> ADG::get_last_activity(int robot_id, Activity::Type type) {
    for (int i = activities_[robot_id].size() - 1; i >= 0; i--) {
        if (activities_[robot_id][i]->type == type) {
            return activities_[robot_id][i];
        }
    }
    return nullptr;
}

void ADG::add_trajectory(int robot_id, int activity_id, const RobotTrajectory &trajectory) {
    //activities_[robot_id][activity_id]->trajectory = trajectory;
}

bool ADG::init(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs) {
    dt_ = config.dt;
    config_ = config;

    start_nodes_.resize(num_robots_);
    end_nodes_.resize(num_robots_);
    numNodes_.resize(num_robots_, 0);
    solution_.resize(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        for (int activity_id = 0; activity_id < activities_[i].size(); activity_id++) {
            auto act = get_activity(i, activity_id);
            act->start_node = tpgs[activity_id]->getStartNode(i);
            act->end_node = tpgs[activity_id]->getEndNode(i);
            assert(act->start_node != nullptr && act->end_node != nullptr);

            std::shared_ptr<Node> iter_node = act->start_node;
            while (iter_node != act->end_node) {
                iter_node->timeStep += numNodes_[i];
                iter_node = iter_node->Type1Next;
            }
            act->end_node->timeStep += numNodes_[i];

            if (activity_id == 0) {
                start_nodes_[i] = act->start_node;
                end_nodes_[i] = act->end_node;
            }
            else {
                end_nodes_[i]->Type1Next = act->start_node;
                act->start_node->Type1Prev = end_nodes_[i];
                end_nodes_[i] = act->end_node;
            }
            numNodes_[i] += tpgs[activity_id]->getNumNodes(i);
        }
    } 

    // add task dependencies type 2 edges
    idType2Edges_ = 10000;
    for (int i = 0; i < num_robots_; i++) {
        for (int activity_id = 0; activity_id < activities_[i].size(); activity_id++) {
            auto act = get_activity(i, activity_id);
            for (auto dep : act->type2_deps) {
                std::cout << "activity: " << act->robot_id << " " << act->activity_id << " " << act->type
                     << " depends on " << dep->robot_id << " " << dep->activity_id << " " << dep->type << std::endl;
                type2Edge edge;
                edge.edgeId = idType2Edges_++;
                assert(dep->end_node != nullptr);
                if (dep->end_node->Type1Next == nullptr) {
                    log("activity depend on another terminal activity, this is deadlock!", LogLevel::ERROR);
                    return false;
                }
                edge.nodeFrom = dep->end_node->Type1Next;
                edge.nodeTo = act->start_node;
                dep->end_node->Type1Next->Type2Next.push_back(edge);
                act->start_node->Type2Prev.push_back(edge);
            }
        }
    }
    
    return true;
}


}