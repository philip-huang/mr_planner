#include "adg.h"
#include "logger.h"

namespace TPG {

ShortcutSamplerADG::ShortcutSamplerADG(const TPGConfig &config, const ActivityGraph &act_graph,
                    const std::vector<std::vector<std::shared_ptr<Node>>> &intermediate_nodes)
    : ShortcutSampler(config)
{
    act_graph_ = act_graph;
    intermediate_nodes_ = intermediate_nodes;
}

void ShortcutSamplerADG::init(const std::vector<std::shared_ptr<Node>> &start_nodes,
    const std::vector<int> &numNodes)
{
    num_robots_ = start_nodes.size();
    nodes_.clear();
    act_ids_.clear();
    act_lengths_.clear();

    nodes_.resize(num_robots_);
    act_ids_.resize(num_robots_);
    act_lengths_.resize(num_robots_);

    // save all the nodes, and activity ids, and activity lengths

    for (int i = 0; i < num_robots_; i++) {
        auto node = start_nodes[i];
        int j = 0;

        std::vector<std::shared_ptr<Node>> nodes_i;
        std::vector<int> act_ids_i;
        int act_id = 0;

        while (node != nullptr) {
            // add node
            nodes_i.push_back(node);

            // add activity id
            auto act_i = act_graph_.get(i, act_id);
            // we skip home activity
            while (act_id < act_graph_.num_activities(i) - 1 && act_i->type == Activity::Type::home) {
                act_id++;
                act_i = act_graph_.get(i, act_id);
            }
            act_ids_i.push_back(act_id);

            // check if we need to switch to next activity
            if (intermediate_nodes_[i][act_id * 2 + 1]->timeStep == node->timeStep) {
                act_id++;
            }

            // iterate to next node
            j++;
            node = node->Type1Next;
        }
        nodes_[i] = nodes_i;
        act_ids_[i] = act_ids_i;

        act_lengths_[i].resize(act_graph_.num_activities(i), 0);
        for (int act_id : act_ids_[i]) {
            act_lengths_[i][act_id]++;
        }

    }
    numNodes_ = numNodes;

    resetFailedShortcuts();
}


bool ShortcutSamplerADG::sampleUniform(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;

    int startNode = std::rand() % numNodes_[i];
    int act_id = act_ids_[i][startNode];
    int act_length = 0;
    shortcut.activity = act_graph_.get(i, act_id);

    for (int j = 0; j <= act_id; j++) {
        act_length += act_lengths_[i][j];
    }

    if (startNode >= act_length - 2) {
        return false;
    }
    int length = std::rand() % (act_length - startNode - 2) + 2;
    int endNode = startNode + length;
    // if (endNode <= 1 || startNode >= endNode - 1) {
    //     return false;
    // }

    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    assert(shortcut.activity != nullptr);

    log("Sampled shortcut from robot " + std::to_string(i) + " activity " + shortcut.activity->type_string() + 
        " timestep " + std::to_string(shortcut.ni.lock()->timeStep) +
        " to timestep " + std::to_string(shortcut.nj.lock()->timeStep), LogLevel::DEBUG);
    
    return true;
}

void ADG::reset()
{
    TPG::reset();
    intermediate_nodes_.clear();
}

bool ADG::init_from_tpgs(std::shared_ptr<PlanInstance> instance, const TPGConfig &config, const std::vector<std::shared_ptr<TPG>> &tpgs) {
    dt_ = config.dt;
    config_ = config;
    num_robots_ = act_graph_.num_robots();

    start_nodes_.resize(num_robots_);
    end_nodes_.resize(num_robots_);
    intermediate_nodes_.resize(num_robots_);
    numNodes_.resize(num_robots_, 0);
    solution_.resize(num_robots_);

    auto t_start = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < num_robots_; i++) {
        std::vector<std::shared_ptr<Node>> inter_nodes;
        for (int act_id = 0; act_id < act_graph_.num_activities(i); act_id++) {
            std::shared_ptr<const Activity> act = act_graph_.get(i, act_id);
            auto inter_start_node = tpgs[act_id]->getStartNode(i);
            auto inter_end_node = tpgs[act_id]->getEndNode(i);

            std::shared_ptr<Node> iter_node = inter_start_node;
            while (iter_node != inter_end_node) {
                iter_node->timeStep += numNodes_[i];
                iter_node->actId = act_id;
                iter_node = iter_node->Type1Next;
            }
            inter_end_node->actId = act_id;
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
            std::shared_ptr<const Activity> act = act_graph_.get(i, act_id);
            for (auto dep : act->type2_prev) {
                std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                edge->edgeId = idType2Edges_++;
                auto dep_end_node = intermediate_nodes_[dep->robot_id][dep->act_id * 2 + 1];
                auto cur_start_node = intermediate_nodes_[i][act_id * 2];
                assert(dep_end_node != nullptr);
                if (dep_end_node->Type1Next == nullptr) {
                    log("activity depend on another terminal activity, this is deadlock!", LogLevel::ERROR);
                    return false;
                }
                edge->nodeFrom = dep_end_node->Type1Next;
                edge->nodeTo = cur_start_node;
                dep_end_node->Type1Next->Type2Next.push_back(edge);
                cur_start_node->Type2Prev.push_back(edge);
            }
        }
    }
    
    // recompute the time step for each node
    std::vector<std::vector<int>> reached_t;
    std::vector<int> reached_end;
    findEarliestReachTimeSyncAct(tpgs.size(), reached_t, reached_end);
    // add collision dependencies type 2 edges

    // for each pair of activities between each pair of robot
    // if the activity does not depend on each other, then
    // update the planning scene (add objects, attach objects, or detach objects)
    // run collision check for each pair of nodes between the two activities
    // add type 2 edge if there is collision
    idType2Edges_ = 20000;
 
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }
            for (int act_id_i = 0; act_id_i < act_graph_.num_activities(i); act_id_i++) {
                auto act_i = act_graph_.get(i, act_id_i);
                
                // updated attached / detached object
                for (auto obj : act_i->obj_attached) {
                    instance->moveObject(obj->obj);
                    //instance->updateScene();
                    instance->attachObjectToRobot(obj->obj.name, i, obj->next_attach_link, act_i->start_pose);
                    //instance->updateScene();
                }
                for (auto obj : act_i->obj_detached) {
                    instance->detachObjectFromRobot(obj->obj.name, act_i->start_pose);
                    //instance->updateScene();
                }

                auto act_i_start_node = intermediate_nodes_[i][act_id_i * 2];
                auto act_i_end_node = intermediate_nodes_[i][act_id_i * 2 + 1];

                // // run bfs on the task graph
                // std::vector<std::vector<bool>> visited;
                // for (int k = 0; k < num_robots_; k++) {
                //     visited.push_back(std::vector<bool>(act_graph_.num_activities(i), false));
                // }
                // act_graph_.bfs(act_i, visited, true);
                // act_graph_.bfs(act_i, visited, false);

                // run bfs on the node graph
                std::vector<std::vector<bool>> visited;
                for (int k = 0; k < num_robots_; k++) {
                    std::vector<bool> v(numNodes_[k], false);
                    visited.push_back(v);
                }
                bfs(act_i_start_node, visited, false);

                for (int act_id_j = 0; act_id_j < act_graph_.num_activities(j); act_id_j++) {
                    // updated attached / detached object
                    auto act_j = act_graph_.get(j, act_id_j);
                    for (auto obj : act_j->obj_attached) {
                        instance->moveObject(obj->obj);
                        //instance->updateScene();
                        instance->attachObjectToRobot(obj->obj.name, j, obj->next_attach_link, act_j->start_pose);
                        //instance->updateScene();
                    }
                    for (auto obj : act_j->obj_detached) {
                        instance->detachObjectFromRobot(obj->obj.name, act_j->start_pose);
                        //instance->updateScene();
                    }

                    if (act_id_j == act_id_i) {
                        // skip if they are in the same tpg because type-2 dependencies would have already been build
                        continue;
                    }
                    // if (visited[j][act_id_j]) {
                    //     // skip if the two activities are dependent
                    //     continue;
                    // }
                    
                    auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                    auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];

                    if (visited[j][act_j_end_node->timeStep]) {
                        // skip if the two activities are dependent
                        continue;
                    }
                    
                    if (reached_t[j][act_j_start_node->timeStep] < reached_t[i][act_i_end_node->timeStep]) {
                        // check collision
                        std::shared_ptr<Node> iter_node_i = act_i_start_node;
                        std::shared_ptr<Node> iter_node_j_start = act_j_start_node;
                        while (iter_node_i != nullptr && iter_node_i->timeStep <= act_i_end_node->timeStep) {
                            std::shared_ptr<Node> iter_node_j = iter_node_j_start;
                            bool inCollision = false;
                            while (iter_node_j != nullptr &&
                                iter_node_j->timeStep <= act_j_end_node->timeStep &&
                                reached_t[j][iter_node_j->timeStep] < reached_t[i][iter_node_i->timeStep]) 
                            {
                                if (visited[j][iter_node_j->timeStep]) {
                                    iter_node_j = iter_node_j->Type1Next;
                                    continue;
                                }
                                bool has_collision = instance->checkCollision({iter_node_i->pose, iter_node_j->pose}, true);
                                if (has_collision) {
                                    inCollision = true;
                                }
                                else if (inCollision) {
                                    inCollision = false;
                                    std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                                    edge->edgeId = idType2Edges_++;
                                    edge->nodeFrom = iter_node_j;
                                    edge->nodeTo = iter_node_i;
                                    iter_node_j->Type2Next.push_back(edge);
                                    iter_node_i->Type2Prev.push_back(edge);
                                    iter_node_j_start = iter_node_j->Type1Next;
                                    log("add type 2 edge from robot " + std::to_string(j) + " activity " 
                                        + act_j->type_string() + " timestep " + std::to_string(iter_node_j->timeStep)
                                        + " " + std::to_string(reached_t[j][iter_node_j->timeStep])
                                        + " to robot " + std::to_string(i) + " activity " 
                                        + act_i->type_string() +  " timestep " + std::to_string(iter_node_i->timeStep)
                                        + " " + std::to_string(reached_t[i][iter_node_i->timeStep]) , LogLevel::INFO);

                                }
                                iter_node_j = iter_node_j->Type1Next;
                            }
                            if (inCollision) {
                                assert(iter_node_j != nullptr);
                                std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                                edge->edgeId = idType2Edges_++;
                                edge->nodeFrom = iter_node_j;
                                edge->nodeTo = iter_node_i;
                                iter_node_j->Type2Next.push_back(edge);
                                iter_node_i->Type2Prev.push_back(edge);
                                iter_node_j_start = iter_node_j;
                                log("add type 2 edge (end act) from robot " + std::to_string(j) + " activity " 
                                        + act_j->type_string() + " timestep " + std::to_string(iter_node_j->timeStep)
                                        + " " + std::to_string(reached_t[j][iter_node_j->timeStep])
                                        + " to robot " + std::to_string(i) + " activity " 
                                        + act_i->type_string() +  " timestep " + std::to_string(iter_node_i->timeStep)
                                        + " " + std::to_string(reached_t[i][iter_node_i->timeStep]) , LogLevel::INFO);

                            }
                            iter_node_i = iter_node_i->Type1Next;
                        }
                    }
                }
            }
        }
    }
    
    t_init_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;
    t_start = std::chrono::high_resolution_clock::now();

    transitiveReduction();

    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }
    t_simplify_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6; 

    int numtype2edges = getTotalType2Edges(); 
    log("ADG initialized with " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges in "
        + std::to_string(t_init_) + "s", LogLevel::HLINFO);

    findFlowtimeMakespan(pre_shortcut_flowtime_, pre_shortcut_makespan_);
    log("Flowtime: " + std::to_string(pre_shortcut_flowtime_) + " Makespan: " + std::to_string(pre_shortcut_makespan_), LogLevel::INFO);

    return true;
}

void ADG::findEarliestReachTimeSyncAct(int num_act, std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end)
{
    reached_t.clear();
    reached_end.clear();
    std::vector<std::shared_ptr<Node>> nodes;
    nodes.resize(num_robots_);
    reached_end.resize(num_robots_, -1);
    
    for (int i = 0; i < num_robots_; i++) {
        std::vector<int> v(numNodes_[i], -1);
        reached_t.push_back(v);
    }

    int flowtime_i = 0;
    int j = 0;
    for (int act_id = 0; act_id < num_act; act_id++) {
        for (int i = 0; i < num_robots_; i++) {
            nodes[i] = intermediate_nodes_[i][act_id * 2];
            reached_end[i] = -1;
        }

        bool allReached = false;
        while(!allReached) {
            for (int i = 0; i < num_robots_; i++) {
                if (reached_t[i][nodes[i]->timeStep] == -1) {
                    reached_t[i][nodes[i]->timeStep] = j;
                }
            }
            for (int i = 0; i < num_robots_; i++) {
                if (nodes[i]->timeStep < intermediate_nodes_[i][act_id * 2 + 1]->timeStep) {
                    bool safe = true;
                    for (auto edge : nodes[i]->Type1Next->Type2Prev) {
                        if (reached_t[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == -1) {
                            safe = false;
                            break;
                        }
                    }
                    if (safe) {
                        nodes[i] = nodes[i]->Type1Next;
                    }
                }
                else if (reached_end[i] == -1) {
                    reached_end[i] = j;
                    flowtime_i += j;
                }
            }

            allReached = true;
            for (int i = 0; i < num_robots_; i++) {
                allReached &= (reached_end[i] != -1);
            }
            j++;
        }
    }
 

}


bool ADG::saveToDotFile(const std::string& filename) const {
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
        std::shared_ptr<Node> node_i = start_nodes_[i];
        std::vector<std::shared_ptr<Node>> salient_nodes;
        while (node_i != nullptr) {
            if (node_i->Type1Prev == nullptr) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_.get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type1Next == nullptr) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_.get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type1Next->actId > node_i->actId &&
                act_graph_.get(i, node_i->Type1Next->actId)->type != act_graph_.get(i, node_i->actId)->type) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_.get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type2Prev.size() > 0) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_.get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            else if (node_i->Type2Next.size() > 0) {
                out << "n" << i << "_" << node_i->timeStep << " [label=\"" << act_graph_.get(i, node_i->actId)->type_string() << node_i->timeStep << "\"];" << std::endl;
                salient_nodes.push_back(node_i);
            }
            
            node_i = node_i->Type1Next;
        }
        assert(salient_nodes.size() > 0);

        node_i = salient_nodes[0];
        out << "n" << i << "_" << node_i->timeStep;
        for (int j = 1; j < salient_nodes.size(); j++) {
            out << " -> " << "n" << i << "_" << salient_nodes[j]->timeStep;
        }
        out << ";" << std::endl;
        out << "}" << std::endl;
    }

    // define all the edges
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            for (auto edge : node_i->Type2Prev) {
                out << "n" << edge->nodeFrom->robotId << "_" << edge->nodeFrom->timeStep << " -> " << "n" << i << "_" << node_i->timeStep << ";" << std::endl;
            }
            node_i = node_i->Type1Next;
        }
    }

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    int result = system(command.c_str());

    return result == 0;
}

void ADG::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    TPG::update_joint_states(joint_states, robot_id);
    
    if (num_robots_ > executed_acts_.size()) {
        return;
    }
   
    int act_id = executed_acts_[robot_id]->load();
    if (act_id >= act_graph_.num_activities(robot_id)) {
        return;
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
        int act_id = executed_acts_[robot_id]->load();
        while (act_id < act_graph_.num_activities(robot_id) - 1 && act_graph_.get(robot_id, act_id)->type == Activity::Type::home){
            act_id ++;
            executed_acts_[robot_id]->fetch_add(1);
        }

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

bool ADG::moveit_execute(std::shared_ptr<MoveitInstance> instance, 
            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) 
{
    instance_ = instance;
    for (int i = 0; i < num_robots_; i++) {
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(0));
        int act_id = 0;
        while (act_id < act_graph_.num_activities(i) - 1 && act_graph_.get(i, act_id)->type == Activity::Type::home){
            act_id ++;
            executed_acts_[i]->fetch_add(1);
        }
    }
    return TPG::moveit_execute(instance, move_group);
}

bool ADG::moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) 
{
    for (int i = 0; i < num_robots_; i++) {
        executed_acts_.push_back(std::make_unique<std::atomic<int>>(0));
            int act_id = 0;
        while (act_id < act_graph_.num_activities(i) - 1 && act_graph_.get(i, act_id)->type == Activity::Type::home){
            act_id ++;
            executed_acts_[i]->fetch_add(1);
        }
    }
    return TPG::moveit_mt_execute(joint_names, clients);
}

void ADG::initSampler() {
    shortcut_sampler_ = std::make_unique<ShortcutSamplerADG>(config_, act_graph_, intermediate_nodes_);
    shortcut_sampler_->init(start_nodes_, numNodes_);
}

void ADG::checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
     std::vector<Eigen::MatrixXi> &col_matrix) const {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    // check if there is a shortcut between ni and nj
    assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);

    int robot_id = ni->robotId;
    double timeNeeded = instance->computeDistance(ni->pose, nj->pose) / instance->getVMax(ni->robotId);
    timeNeeded = std::ceil(timeNeeded / dt_) * dt_;
    int shortcutSteps = timeNeeded / dt_ + 1;

    if (shortcutSteps > (nj->timeStep - ni->timeStep)) {
        shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
        return;
    }

    // build collision environment
    std::shared_ptr<Activity> cur_act = shortcut.activity;
    assert(cur_act != nullptr);
    
    instance->resetScene(false);
    // add all static objects that needs to be collision checked
    std::vector<ObjPtr> indep_objs = act_graph_.find_indep_obj(cur_act);
    for (auto obj : indep_objs) {
        instance->addMoveableObject(obj->obj);
        //instance->updateScene();
    }

    for (int act_id = 0; act_id <= cur_act->act_id; act_id++) {
        // updated attached / detached object
        std::shared_ptr<const Activity> act_j = act_graph_.get(robot_id, act_id);
        for (auto obj : act_j->obj_attached) {
            instance->moveObject(obj->obj);
            //instance->updateScene();
            instance->attachObjectToRobot(obj->obj.name, robot_id, obj->next_attach_link, act_j->start_pose);
            //instance->updateScene();
        }
        for (auto obj : act_j->obj_detached) {
            instance->detachObjectFromRobot(obj->obj.name, act_j->start_pose);
            //instance->updateScene();
        }
        for (auto col_node : act_j->collision_nodes) {
            instance->setCollision(col_node.obj_name, col_node.link_name, col_node.allow);
        }
    }

    for (int i = 1; i < shortcutSteps - 1; i++) {
        double alpha = i * dt_ / timeNeeded;
        RobotPose pose_i = instance->interpolate(ni->pose, nj->pose, alpha);

        // check environment collision
        if (instance->checkCollision({pose_i}, false) == true) {
            shortcut.col_type = CollisionType::STATIC; // collide with the evnrionment
            return;
        }
        shortcut.path.push_back(pose_i);
    }

    auto tic = std::chrono::high_resolution_clock::now();

    // find dependent parent and child nodes
    std::vector<std::vector<bool>> visited_act;
    for (int i = 0; i < num_robots_; i++) {
        visited_act.push_back(std::vector<bool>(act_graph_.num_activities(i), false));
    }
    act_graph_.bfs(cur_act, visited_act, true);
    act_graph_.bfs(cur_act, visited_act, false);

    std::vector<std::vector<bool>> visited;
    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        visited.push_back(v);
    }
    visited[nj->robotId][nj->timeStep] = true;
    bfs(nj, visited, true);

    visited[ni->robotId][ni->timeStep] = true;
    bfs(ni, visited, false);

    if (shortcutSteps <= 2) {
        // special case, check if the static node collides with any other independent nodes
        
        for (int j = 0; j < num_robots_; j++) {
            if (j == ni->robotId) {
                continue;
            }
            for (int act_id_j = 0; act_id_j < act_graph_.num_activities(j); act_id_j++) {

                // updated attached / detached object
                auto act_j = act_graph_.get(j, act_id_j);
                for (auto obj : act_j->obj_attached) {
                    instance->moveObject(obj->obj);
                    //instance->updateScene();
                    instance->attachObjectToRobot(obj->obj.name, j, obj->next_attach_link, act_j->start_pose);
                    //instance->updateScene();
                }
                for (auto obj : act_j->obj_detached) {
                    instance->detachObjectFromRobot(obj->obj.name, act_j->start_pose);
                    //instance->updateScene();
                }
                if (visited_act[j][act_id_j] == true) {
                    continue;
                }
                auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
                auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                while (act_j->type == Activity::Type::home && act_id_j < act_graph_.num_activities(j) - 1) {
                    act_id_j ++;
                    act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                    act_j = act_graph_.get(j, act_id_j);
                }
                std::shared_ptr<Node> node_j = act_j_start_node;
                while (node_j != nullptr && node_j->timeStep <= act_j_end_node->timeStep) {
                    if (visited[j][node_j->timeStep] == false) {
                        if (instance->checkCollision({ni->pose, node_j->pose}, true) ||
                            instance->checkCollision({nj->pose, node_j->pose}, true)) {
                            shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                            shortcut.n_robot_col = node_j;
                            return;
                        }
                    }
                    node_j = node_j->Type1Next;
                }
            }
        }
        shortcut.col_type = CollisionType::NONE;
        return;
    }

    for (int j = 0; j < num_robots_; j++) {
        Eigen::MatrixXi col_matrix_j(shortcutSteps, numNodes_[j]);
        col_matrix_j.setZero();
        col_matrix.push_back(col_matrix_j);
    }

    auto t_bfs = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic).count();

    // check robot-robot collision
   
    for (int j = 0; j < num_robots_; j++) {
        if (j == ni->robotId) {
            continue;
        }
        for (int act_id_j = 0; act_id_j < act_graph_.num_activities(j); act_id_j++) {
            // updated attached / detached object
            auto act_j = act_graph_.get(j, act_id_j);
            for (auto obj : act_j->obj_attached) {
                instance->moveObject(obj->obj);
                //instance->updateScene();
                instance->attachObjectToRobot(obj->obj.name, j, obj->next_attach_link, act_j->start_pose);
                //instance->updateScene();
            }
            for (auto obj : act_j->obj_detached) {
                instance->detachObjectFromRobot(obj->obj.name, act_j->start_pose);
                //instance->updateScene();
            }
            if (visited_act[j][act_id_j] == true) {
                continue;
            }
            auto act_j_start_node = intermediate_nodes_[j][act_id_j * 2];
            auto act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];

            while (act_j->type == Activity::Type::home && act_id_j < act_graph_.num_activities(j) - 1) {
                act_id_j ++;
                act_j_end_node = intermediate_nodes_[j][act_id_j * 2 + 1];
                act_j = act_graph_.get(j, act_id_j);
            }
            for (int i = 1; i < shortcutSteps - 1; i++) {
                RobotPose pose_i = shortcut.path[i - 1];
                std::shared_ptr<Node> node_j = act_j_start_node;
                while (node_j != nullptr && node_j->timeStep <= act_j_end_node->timeStep) {
                    if (config_.ignore_far_collisions) {
                        if (node_j->timeStep < ni->timeStep - config_.ignore_steps) {
                            node_j = node_j->Type1Next;
                            continue;
                        }
                        else if (node_j->timeStep > ni->timeStep + shortcutSteps + config_.ignore_steps) {
                            break;
                        }
                    }
                    if (visited[j][node_j->timeStep] == false) {
                        
                        col_matrix[j](i, node_j->timeStep) = instance->checkCollision({pose_i, node_j->pose}, true);
                        if (col_matrix[j](i, node_j->timeStep)) {
                            shortcut.n_robot_col = node_j;
                            shortcut.col_type = CollisionType::ROBOT; // collide with other robots
                            return;
                        }
                    }
                    node_j = node_j->Type1Next;
                }
            }
        }
    }

    shortcut.col_type = CollisionType::NONE;
    return;
}


}