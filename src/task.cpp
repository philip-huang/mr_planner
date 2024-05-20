#include "task.h"
#include <fstream>
#include <queue>

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
    {Activity::Type::support_pre, "support_pre"},
};

ActivityGraph::ActivityGraph(int num_robots) {
    num_robots_ = num_robots;
    activities_.resize(num_robots);

}

void ActivityGraph::add_act(int robot_id, Activity::Type type) {
    assert(robot_id < num_robots_);

    std::shared_ptr<Activity> activity = std::make_shared<Activity>(robot_id, type);
    activity->act_id = activities_[robot_id].size();
    
    activities_[robot_id].push_back(activity);

    if (activity->act_id > 0) {
        auto prev_act = activities_[robot_id][activity->act_id - 1];
        prev_act->type1_next = activity;
        activity->type1_prev = prev_act;
    }
}

void ActivityGraph::add_act(int robot_id, Activity::Type type, std::shared_ptr<Activity> type2_dep) {
    assert(robot_id < num_robots_ && type2_dep != nullptr);
    std::shared_ptr<Activity> activity = std::make_shared<Activity>(robot_id, type);
    activity->act_id = activities_[robot_id].size();
    activity->add_type2_dep(type2_dep);
    type2_dep->add_type2_next(activity);

    activities_[robot_id].push_back(activity);

    if (activity->act_id > 0) {
        auto prev_act = activities_[robot_id][activity->act_id - 1];
        prev_act->type1_next = activity;
        activity->type1_prev = prev_act;
    }
}

/* add a static object to the scene (no attached parent)*/
ObjPtr ActivityGraph::add_obj(const Object &obj) {
    ObjPtr obj_node = std::make_shared<ObjectNode>(obj, obj_nodes_.size());
    obj_nodes_.push_back(obj_node);
    return obj_node;
}

/* set the object node to be attached to a robot at the onset of selected activity */
void ActivityGraph::attach_obj(ObjPtr obj, const std::string &link_name, std::shared_ptr<Activity> act) {
    obj->next_attach = act;
    obj->next_attach_link = link_name;
    act->obj_attached.push_back(obj);
}

/* set the object node to be detached from a robot at the onset of selected activity */
void ActivityGraph::detach_obj(ObjPtr obj, std::shared_ptr<Activity> act) {
    obj->prev_detach = act;
    act->obj_detached.push_back(obj);
}

/* enable or disable collision checking between the object_node and the robot at the onset of selected activity */
void ActivityGraph::set_collision(const std::string &obj_name, const std::string &name, std::shared_ptr<Activity> act, bool allow) {
    SetCollisionNode node(obj_name, name, allow);
    if (allow) {
        act->disable_collision.push_back(node);
    }
    else {
        act->enable_collision.push_back(node);
    }
}

std::shared_ptr<Activity> ActivityGraph::get(int robot_id, int act_id) {
    if (robot_id >= num_robots_ || act_id >= activities_[robot_id].size()) {
        return nullptr;
    }
    return activities_[robot_id][act_id];
}

std::shared_ptr<Activity> ActivityGraph::get_last_act(int robot_id) {
    if (robot_id < num_robots_) {
        return activities_[robot_id].back();
    }
    return nullptr;
}

std::shared_ptr<Activity> ActivityGraph::get_last_act(int robot_id, Activity::Type type) {
    if (robot_id < num_robots_) {
        for (int i = activities_[robot_id].size() - 1; i >= 0; i--) {
            if (activities_[robot_id][i]->type == type) {
                return activities_[robot_id][i];
            }
        }
    }
    return nullptr;
}

ObjPtr ActivityGraph::get_last_obj(const std::string &obj_name) {
    for (int i = obj_nodes_.size() - 1; i >= 0; i--) {
        if (obj_nodes_[i]->obj.name == obj_name) {
            return obj_nodes_[i];
        }
    }
    return nullptr;
}



bool ActivityGraph::saveGraphToFile(const std::string &filename) const {
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
            std::shared_ptr<Activity> act = activities_[i][j];
            out << "a" << i << "_" << act->act_id << " [label=\"" << act->type_string() << "\"];" << std::endl;
        }
        // activity type-1 edges
        std::shared_ptr<Activity> act = activities_[i][0];
        out << "a" << i << "_" << act->act_id;
        for (int j = 1; j < activities_[i].size(); j++) {
            std::shared_ptr<Activity> act = activities_[i][j];
            out << " -> " << "a" << i << "_" << act->act_id;
        }

        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    // define type2 edges
    for (int i = 0; i < num_robots_; i++) {
        for (int act_id = 0; act_id < activities_[i].size(); act_id++) {
            auto act = activities_[i][act_id];
            for (auto dep : act->type2_prev) {
                out << "a" << dep->robot_id << "_" << dep->act_id << " -> " << "a" << act->robot_id << "_" << act->act_id << ";" << std::endl;
            }
        }
    }

    // define object nodes
    for (int i = 0; i < obj_nodes_.size(); i++) {
        out << "o" << i << " [label=\"" << obj_nodes_[i]->obj.name << "\"];" << std::endl;
        if (obj_nodes_[i]->prev_detach != nullptr) {
            out << "a" << obj_nodes_[i]->prev_detach->robot_id << "_" << obj_nodes_[i]->prev_detach->act_id << " -> o" << i << ";" << std::endl;
        }
        if (obj_nodes_[i]->next_attach != nullptr) {
            out << "o" << i << " -> a" << obj_nodes_[i]->next_attach->robot_id << "_" << obj_nodes_[i]->next_attach->act_id << ";" << std::endl;
        }
    }
    

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    system(command.c_str());

    return true;

}

bool ActivityGraph::bfs(std::shared_ptr<Activity> act_i, std::vector<std::vector<bool>> &visited, bool forward) const
{
    // BFS function to find all the dependent Activitys of act_i
    std::queue<std::shared_ptr<Activity>> q;
    q.push(act_i);
    visited[act_i->robot_id][act_i->act_id] = true;
    
    while (!q.empty()) {
        std::shared_ptr<Activity> act = q.front();
        q.pop();
        if (forward) {
            if (act->type1_next != nullptr && !visited[act->type1_next->robot_id][act->type1_next->act_id]) {
                q.push(act->type1_next);
                visited[act->type1_next->robot_id][act->type1_next->act_id] = true;
            }
            
            for (auto dep_act : act->type2_next) {
                if (!visited[dep_act->robot_id][dep_act->act_id]) {
                    q.push(dep_act);
                    visited[dep_act->robot_id][dep_act->act_id] = true;
                }
            }
        } 
        else {
            if (act->type1_prev != nullptr && !visited[act->type1_prev->robot_id][act->type1_prev->act_id]) {
                q.push(act->type1_prev);
                visited[act->type1_prev->robot_id][act->type1_prev->act_id] = true;
            }
            
            for (auto dep_act : act->type2_prev) {
                if (!visited[dep_act->robot_id][dep_act->act_id]) {
                    q.push(dep_act);
                    visited[dep_act->robot_id][dep_act->act_id] = true;
                }
            }
        }
    }
    return true;
}

bool ActivityGraph::obj_dependent(std::shared_ptr<Activity> act, ObjPtr obj_node) const {
    if (obj_node->prev_detach == nullptr && obj_node->next_attach == nullptr) {
        return false;
    }

    std::vector<std::vector<bool>> visited(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        visited[i].resize(activities_[i].size(), false);
    }

    if (obj_node->prev_detach != nullptr) {
        bfs(obj_node->prev_detach, visited, true);
        if (visited[act->robot_id][act->act_id]) {
            return true;
        }
    }
    else if (obj_node->next_attach != nullptr) {
        bfs(obj_node->next_attach, visited, false);
        if (visited[act->robot_id][act->act_id]) {
            return true;
        }
    }
    if (obj_node->prev_detach != nullptr) {
        if (obj_node->prev_detach->type1_prev != nullptr) {
            return true;
        }
        for (auto dep : obj_node->prev_detach->type2_prev) {
            if (dep->type1_prev != nullptr) {
                return true;
            }
        }
    }
    else if (obj_node->next_attach != nullptr) {
        if (obj_node->next_attach->type1_next != nullptr) {
            return true;
        }
        for (auto dep : obj_node->next_attach->type2_next) {
            if (dep->type1_next != nullptr) {
                return true;
            }
        }
    }
    return false;
}