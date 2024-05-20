#include "adg.h"
#include "logger.h"

namespace TPG {

bool ADG::init_from_tpgs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs) {
    dt_ = config.dt;
    config_ = config;
    num_robots_ = act_graph_.num_robots();

    start_nodes_.resize(num_robots_);
    end_nodes_.resize(num_robots_);
    intermediate_nodes_.resize(num_robots_);
    numNodes_.resize(num_robots_, 0);
    solution_.resize(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        std::vector<std::shared_ptr<Node>> inter_nodes;
        for (int act_id = 0; act_id < act_graph_.num_activities(i); act_id++) {
            auto act = act_graph_.get(i, act_id);
            auto inter_start_node = tpgs[act_id]->getStartNode(i);
            auto inter_end_node = tpgs[act_id]->getEndNode(i);

            std::shared_ptr<Node> iter_node = inter_start_node;
            while (iter_node != inter_end_node) {
                iter_node->timeStep += numNodes_[i];
                iter_node = iter_node->Type1Next;
            }
            inter_end_node->timeStep += numNodes_[i];

            if (act_id > 0) {
                inter_nodes.back()->Type1Next = inter_start_node;
                inter_start_node->Type1Prev = inter_nodes.back();
            }

            inter_nodes.push_back(inter_start_node);
            inter_nodes.push_back(inter_end_node);
            numNodes_[i] += tpgs[act_id]->getNumNodes(i);
        }
        intermediate_nodes_[i] = inter_nodes;
        start_nodes_[i] = inter_nodes.front();
        end_nodes_[i] = inter_nodes.back();
    }

    // add task dependencies type 2 edges
    idType2Edges_ = 10000;
    for (int i = 0; i < num_robots_; i++) {
        for (int act_id = 0; act_id < act_graph_.num_activities(i); act_id++) {
            auto act = act_graph_.get(i, act_id);
            for (auto dep : act->type2_prev) {
                type2Edge edge;
                edge.edgeId = idType2Edges_++;
                auto dep_end_node = intermediate_nodes_[dep->robot_id][dep->act_id * 2 + 1];
                auto cur_start_node = intermediate_nodes_[i][act_id * 2];
                assert(dep_end_node != nullptr);
                if (dep_end_node->Type1Next == nullptr) {
                    log("activity depend on another terminal activity, this is deadlock!", LogLevel::ERROR);
                    return false;
                }
                edge.nodeFrom = dep_end_node->Type1Next;
                edge.nodeTo = cur_start_node;
                dep_end_node->Type1Next->Type2Next.push_back(edge);
                cur_start_node->Type2Prev.push_back(edge);
            }
        }
    }
    
    // recompute the time step for each node
    std::vector<std::vector<int>> reached_t;
    std::vector<int> reached_end;
    findEarliestReachTime(reached_t, reached_end);
    // add collision dependencies type 2 edges

    // for each pair of activities between each pair of robot
    // if the activity does not depend on each other, then
    // update the planning scene (add objects, attach objects, or detach objects)
    // run collision check for each pair of nodes between the two activities
    // add type 2 edge if there is collision
    idType2Edges_ = 20000;
 
    for (int i = 0; i < num_robots_; i++) {
        for (int j = i + 1; j < num_robots_; j++) {
            for (int act_id_i = 0; act_id_i < act_graph_.num_activities(i); act_id_i++) {
                auto act_i = act_graph_.get(i, act_id_i);
                
                // updated attached / detached object
                for (auto obj : act_i->obj_attached) {
                    instance->moveObject(obj->obj);
                    instance->updateScene();
                    instance->attachObjectToRobot(obj->obj.name, i, obj->next_attach_link, act_i->start_pose);
                    instance->updateScene();
                }
                for (auto obj : act_i->obj_detached) {
                    instance->detachObjectFromRobot(obj->obj.name, act_i->start_pose);
                    instance->updateScene();
                }

                // run bfs on the task graph
                std::vector<std::vector<bool>> visited;
                for (int k = 0; k < num_robots_; k++) {
                    visited.push_back(std::vector<bool>(act_graph_.num_activities(i), false));
                }
                act_graph_.bfs(act_i, visited, true);
                act_graph_.bfs(act_i, visited, false);

                for (int act_id_j = 0; act_id_j < act_graph_.num_activities(j); act_id_j++) {
                    // updated attached / detached object
                    auto act_j = act_graph_.get(j, act_id_j);
                    for (auto obj : act_j->obj_attached) {
                        instance->attachObjectToRobot(obj->obj.name, j, obj->next_attach_link, act_j->start_pose);
                        instance->updateScene();
                    }
                    for (auto obj : act_j->obj_detached) {
                        instance->detachObjectFromRobot(obj->obj.name, act_j->start_pose);
                        instance->updateScene();
                    }

                    if (act_id_i == act_id_j) {
                        // skip if they are in the same tpg because type-2 dependencies would have already been build
                        continue;
                    }
                    if (visited[j][act_id_j]) {
                        // skip if the two activities are dependent
                        continue;
                    }
                    
                    auto act_i_start_node = intermediate_nodes_[i][act_id_i * 2];
                    auto act_i_end_node = intermediate_nodes_[i][act_id_i * 2 + 1];
                    auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                    auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                    
                    if (reached_t[j][act_j_start_node->timeStep] <= reached_t[i][act_i_end_node->timeStep]) {
                        // check collision
                        std::shared_ptr<Node> iter_node_i = act_i_start_node;
                        std::shared_ptr<Node> iter_node_j_start = act_j_start_node;
                        while (iter_node_i != nullptr && iter_node_i->timeStep <= act_i_end_node->timeStep) {
                            std::shared_ptr<Node> iter_node_j = iter_node_j_start;
                            bool inCollision = false;
                            while (iter_node_j != nullptr &&
                                iter_node_j->timeStep <= act_j_end_node->timeStep &&
                                reached_t[j][iter_node_j->timeStep] <= reached_t[i][iter_node_i->timeStep]) 
                            {
                                bool has_collision = instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true);
                                if (has_collision) {
                                    inCollision = true;
                                }
                                else if (inCollision) {
                                    inCollision = false;
                                    type2Edge edge;
                                    edge.edgeId = idType2Edges_++;
                                    edge.nodeFrom = iter_node_j;
                                    edge.nodeTo = iter_node_i;
                                    iter_node_j->Type2Next.push_back(edge);
                                    iter_node_i->Type2Prev.push_back(edge);
                                    iter_node_j_start = iter_node_j->Type1Next;

                                    // std::cout << "add type 2 edge from robot " << j << " timestep " << iter_node_j->timeStep
                                    //     << " to robot " << i << " timestep " << iter_node_i->timeStep << std::endl;
                                }
                                iter_node_j = iter_node_j->Type1Next;
                            }
                            iter_node_i = iter_node_i->Type1Next;
                        }
                    }
                }
            }
        }
    }

    transitiveReduction();
    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }
    log("ADG initialized", LogLevel::HLINFO);
    
    return true;
}

void ADG::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    TPG::update_joint_states(joint_states, robot_id);
    
    if (num_robots_ > executed_acts_.size()) {
        return;
    }
    // check if the activity is completed
    for (int robot_id = 0; robot_id < num_robots_; robot_id++) {
        int act_id = executed_acts_[robot_id]->load();
        if (act_id >= act_graph_.num_activities(robot_id)) {
            continue;
        }
        auto act = act_graph_.get(robot_id, act_id);
        // compare the joint values
        double error = 0;
        for (int i = 0; i < joint_states_[robot_id].size(); i++) {
            error += std::abs(joint_states_[robot_id][i] - act->end_pose.joint_values[i]);
        }
        if (error < 0.1) {
            executed_acts_[robot_id]->fetch_add(1);
            log("Robot " + std::to_string(robot_id) + " finished activity " + act->type_string(), LogLevel::DEBUG);

            // update any attached object
            if (instance_) {
                for (auto obj : act->obj_attached) {
                    instance_->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act->start_pose);
                    instance_->updateScene();
                }
                for (auto obj : act->obj_detached) {
                    instance_->detachObjectFromRobot(obj->obj.name, act->start_pose);
                    instance_->updateScene();
                }
            }
            
        }
    }
}

bool ADG::moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) 
{
    instance_ = instance;
    for (int i = 0; i < num_robots_; i++) {
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(0));
    }
    return TPG::moveit_execute(instance, move_group);
}

bool ADG::moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) 
{
    for (int i = 0; i < num_robots_; i++) {
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(0));
    }
    return TPG::moveit_mt_execute(joint_names, clients);
}

}