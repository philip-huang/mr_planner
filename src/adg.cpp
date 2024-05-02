#include "adg.h"
#include "logger.h"

namespace TPG {

const std::map<Activity::Type, std::string> Activity::enumStringMap = {
    {Activity::Type::home, "home"},
    {Activity::Type::pick_tilt_up, "pick_tilt_up"},
    {Activity::Type::pick_up, "pick_up"},
    {Activity::Type::pick_down, "pick_down"},
    {Activity::Type::pick_twist, "pick_twist"},
    {Activity::Type::pick_twist_up, "pick_twist_up"},
    {Activity::Type::drop_tilt_up, "drop_tilt_up"},
    {Activity::Type::drop_up, "drop_up"},
    {Activity::Type::drop_down, "drop_down"},
    {Activity::Type::drop_twist, "drop_twist"},
    {Activity::Type::drop_twist_up, "drop_twist_up"},
    {Activity::Type::support, "support"},
};

ADG::ADG(int num_robots) {
    num_robots_ = num_robots;
    activities_.resize(num_robots);

}

void ADG::add_activity(int robot_id, Activity::Type type) {
    std::shared_ptr<Activity> activity = std::make_shared<Activity>(robot_id, type);
    activity->activity_id = activities_[robot_id].size();
    
    activities_[robot_id].push_back(activity);
}

void ADG::add_activity(int robot_id, Activity::Type type, std::shared_ptr<Activity> type2_dep) {
    assert(type2_dep != nullptr);
    std::shared_ptr<Activity> activity = std::make_shared<Activity>(robot_id, type);
    activity->activity_id = activities_[robot_id].size();
    activity->add_type2_dep(type2_dep);
    type2_dep->add_type2_next(activity);

    activities_[robot_id].push_back(activity);
}

void ADG::add_activity(int robot_id, Activity::Type type, const Object &obj) {
    std::shared_ptr<Activity> activity = std::make_shared<Activity>(robot_id, type);
    activity->activity_id = activities_[robot_id].size();
    auto obj_node = std::make_shared<ObjectNode>(obj, obj_nodes_.size());

    if (type <= Activity::Type::pick_twist_up) {
        activity->obj_picked.push_back(obj_node);
        obj_node->next_pick = activity;
    }
    else {
        activity->obj_placed.push_back(obj_node);
        obj_node->prev_place = activity;
    }
    
    obj_nodes_.push_back(obj_node);
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
                auto prev_act = get_activity(i, activity_id - 1);
                prev_act->type1_next = act;
                act->type1_prev = prev_act;
            }
            numNodes_[i] += tpgs[activity_id]->getNumNodes(i);
        }
    } 

    // add task dependencies type 2 edges
    idType2Edges_ = 10000;
    for (int i = 0; i < num_robots_; i++) {
        for (int activity_id = 0; activity_id < activities_[i].size(); activity_id++) {
            auto act = get_activity(i, activity_id);
            for (auto dep : act->type2_prev) {
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

    // add collision dependencies type 2 edges

    // for each pair of activities between each pair of robot
    // if the activity does not depend on each other, then
    // update the planning scene (add objects, attach objects, or detach objects)
    // run collision check for each pair of nodes between the two activities
    // add type 2 edge if there is collision
    // idType2Edges_ = 20000;

    // for (int i = 0; i < num_robots_; i++) {
    //     for (int j = i + 1; j < num_robots_; j++) {
    //         for (int activity_id_i = 0; activity_id_i < activities_[i].size(); activity_id_i++) {
    //             for (int activity_id_j = 0; activity_id_j < activities_[j].size(); activity_id_j++) {
    //                 if (activity_id_i == activity_id_j) {
    //                     continue;
    //                 }
    //                 auto act_i = get_activity(i, activity_id_i);
    //                 auto act_j = get_activity(j, activity_id_j);
                    
                    
    //                 if (act_i->end_node->timeStep >= act_j->start_node->timeStep) {
    //                     // check collision
    //                     std::shared_ptr<Node> iter_node_i = act_i->start_node;
    //                     while (iter_node_i != act_j->end_node) {
    //                         std::shared_ptr<Node> iter_node_j = act_j->start_node;
    //                         bool inCollision = false;
    //                         while (iter_node_j != act_j->end_node && iter_node_j->timeStep <= iter_node_i->timeStep) {
    //                             bool has_collision = instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true);
    //                             if (has_collision) {
    //                                 inCollision = true;
    //                             }
    //                             else {
    //                                 inCollision = false;
    //                                 type2Edge edge;
    //                                 edge.edgeId = idType2Edges_++;
    //                                 edge.nodeFrom = iter_node_j;
    //                                 edge.nodeTo = iter_node_i;
    //                                 iter_node_j->Type2Next.push_back(edge);
    //                                 iter_node_i->Type2Prev.push_back(edge);
    //                             }
    //                             iter_node_j = iter_node_j->Type1Next;
    //                         }
    //                         iter_node_i = iter_node_i->Type1Next;
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    transitiveReduction();
    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }
    log("ADG initialized", LogLevel::HLINFO);
    
    return true;
}


bool ADG::saveADGToDotFile(const std::string &filename) {
    std::ofstream out(filename);
    out << "digraph G {" << std::endl;

    // define node attributes here
    out << "node [shape=ellipse];" << std::endl;
    out << "rankdir=LR;" << std::endl;

    // define all the nodes
    for (int i = 0; i < num_robots_; i++) {
        out << "subgraph cluster_" << i << " {" << std::endl;
        out << "label = \"Robot " << i << "\";" << std::endl;
        out << "rank=same;" << std::endl;
        for (int j = 0; j < activities_[i].size(); j++) {
            std::shared_ptr<Activity> act = get_activity(i, j);
            out << "a" << i << "_" << act->activity_id << " [label=\"" << act->type_string() << "\"];" << std::endl;
        }
        // activity type-1 edges
        std::shared_ptr<Activity> act = get_activity(i, 0);
        out << "a" << i << "_" << act->activity_id;
        for (int j = 1; j < activities_[i].size(); j++) {
            std::shared_ptr<Activity> act = get_activity(i, j);
            out << " -> " << "a" << i << "_" << act->activity_id;
        }

        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    // define type2 edges
    for (int i = 0; i < num_robots_; i++) {
        for (int activity_id = 0; activity_id < activities_[i].size(); activity_id++) {
            auto act = get_activity(i, activity_id);
            for (auto dep : act->type2_prev) {
                out << "a" << dep->robot_id << "_" << dep->activity_id << " -> " << "a" << act->robot_id << "_" << act->activity_id << ";" << std::endl;
            }
        }
    }

    // define object nodes
    for (int i = 0; i < obj_nodes_.size(); i++) {
        out << "o" << i << " [label=\"" << obj_nodes_[i]->obj.name << "\"];" << std::endl;
        if (obj_nodes_[i]->prev_place != nullptr) {
            out << "a" << obj_nodes_[i]->prev_place->robot_id << "_" << obj_nodes_[i]->prev_place->activity_id << " -> o" << i << ";" << std::endl;
        }
        if (obj_nodes_[i]->next_pick != nullptr) {
            out << "o" << i << " -> a" << obj_nodes_[i]->next_pick->robot_id << "_" << obj_nodes_[i]->next_pick->activity_id << ";" << std::endl;
        }
    }
    

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    system(command.c_str());

    return true;

}

}