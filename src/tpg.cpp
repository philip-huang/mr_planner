#include "tpg.h"
#include "logger.h"

namespace TPG {

ShortcutSampler::ShortcutSampler(const TPGConfig &config) {
    biased_ = config.biased_sample; 
    std::srand(config.seed);
}

void ShortcutSampler::init(const std::vector<std::shared_ptr<Node>> &start_nodes, const std::vector<int> &numNodes) {
    num_robots_ = start_nodes.size();
    nodes_.resize(num_robots_);
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node = start_nodes[i];
        std::vector<std::shared_ptr<Node>> nodes_i;
        while (node != nullptr) {
            nodes_i.push_back(node);
            node = node->Type1Next;
        }
        nodes_[i] = nodes_i;
    }
    numNodes_ = numNodes;

    resetFailedShortcuts();
}

bool ShortcutSampler::sample(Shortcut &shortcut) {
    if (!biased_) {
        return sampleUniform(shortcut);
    }
    else {
        return sampleBiased(shortcut);
    }
}

bool ShortcutSampler::sampleUniform(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;
    int startNode = std::rand() % numNodes_[i];
    if (startNode >= numNodes_[i] - 2) {
        return false;
    }
    int length = std::rand() % (numNodes_[i] - startNode - 2) + 2;
    int endNode = startNode + length;
    // if (endNode <= 1 || startNode >= endNode - 1) {
    //     return false;
    // }

    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    return true;
}

bool ShortcutSampler::sampleBiased(Shortcut &shortcut) {
    int i = std::rand() % num_robots_;
    int startNode = std::rand() % numNodes_[i];
    if (startNode >= numNodes_[i] - 2) {
        return false;
    }
    int length = std::rand() % (numNodes_[i] - startNode - 2) + 2;
    int endNode = startNode + length;
    // if (endNode <= 1 || startNode >= endNode - 1) {
    //     return false;
    // }


    for (int k = 0; k < already_shortcuts_.size(); k++) {
        std::shared_ptr<Node> k1 = already_shortcuts_[k].ni.lock();
        std::shared_ptr<Node> k2 = already_shortcuts_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            if (startNode >= s && endNode <= e) {
                return false;
            }
        }
    }
    for (int k = 0; k < untight_shortcuts_.size(); k++) {
        std::shared_ptr<Node> k1 = untight_shortcuts_[k].ni.lock();
        std::shared_ptr<Node> k2 = untight_shortcuts_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            if (startNode >= s && endNode <= e) {
                return false;
            }
        }
    }
    

    double prob = 1.0;

    for (int k = 0; k < failed_shortcuts_.size(); k++) {
        std::shared_ptr<Node> k1 = failed_shortcuts_[k].ni.lock();
        std::shared_ptr<Node> k2 = failed_shortcuts_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            //double prob_k = std::min(1.0, (std::pow(s - startNode, 2) + std::pow(e - endNode, 2))/ (scale_ * scale_));
            double prob_k = std::min(1.0, (std::max(0, startNode -s) + std::max(0, e - endNode)) / scale_);
            prob = prob * prob_k;
        }
    }
    for (int k = 0; k < failed_shortcuts_static_.size(); k++) {
        std::shared_ptr<Node> k1 = failed_shortcuts_static_[k].ni.lock();
        std::shared_ptr<Node> k2 = failed_shortcuts_static_[k].nj.lock();
        if (k1->robotId == i) {
            int s = k1->timeStep;
            int e = k2->timeStep;
            //double prob_k = std::min(1.0, (std::pow(s - startNode, 2) + std::pow(e - endNode, 2))/ (scale_ * scale_));
            double prob_k = std::min(1.0, (std::max(0, startNode -s) + std::max(0, e - endNode)) / scale_);
            prob = prob * prob_k;
        }
    }
    
    // drop the sample based on the probability
    int s = std::rand() % 1000;
    prob = prob * 1000;
    log("Sample prob: " + std::to_string(prob) + " random: " + std::to_string(s), LogLevel::DEBUG);

    if (s >= prob) {
        return false;
    }
    shortcut.ni = nodes_[i][startNode];
    shortcut.nj = nodes_[i][endNode];
    return true;
}

void ShortcutSampler::resetFailedShortcuts() {
    untight_shortcuts_.clear();

    // loop over already_shortcuts_ and failed_shortcuts_static_ to remove any entries with weak pointer
    for (int i = already_shortcuts_.size() - 1; i >=0; i--) {
        if (already_shortcuts_[i].expired()) {
            already_shortcuts_.erase(already_shortcuts_.begin() + i);
        }
    }
    for (int i = failed_shortcuts_static_.size() - 1; i >=0; i--) {
        if (failed_shortcuts_static_[i].expired()) {
            failed_shortcuts_static_.erase(failed_shortcuts_static_.begin() + i);
        }
    }
    for (int i = failed_shortcuts_.size() - 1; i >=0; i--) {
        if (failed_shortcuts_[i].expired() || failed_shortcuts_[i].n_robot_col.expired()) {
            failed_shortcuts_.erase(failed_shortcuts_.begin() + i);
        }
    }
}

void ShortcutSampler::updateFailedShortcut(const Shortcut &shortcut) {
    if (!biased_) {
        if (shortcut.col_type == CollisionType::STATIC) {
            failed_shortcuts_static_.push_back(shortcut);
        }
        else if (shortcut.col_type == CollisionType::NO_NEED) {
            already_shortcuts_.push_back(shortcut);
        }
        else if (shortcut.col_type == CollisionType::ROBOT) {
            failed_shortcuts_.push_back(shortcut);
        }
        else if (shortcut.col_type == CollisionType::UNTIGHT) {
            untight_shortcuts_.push_back(shortcut);
        }
    }
}

void TPG::reset() {
    start_nodes_.clear();
    end_nodes_.clear();
    numNodes_.clear();
    collisionCheckMatrix_.clear();
    type2Edges_.clear();
    solution_.clear();
    idType2Edges_ = 0;
    num_robots_ = 0;
    joint_states_.clear();
    executed_steps_.clear();
    pre_shortcut_flowtime_ = 0.0;
    pre_shortcut_makespan_ = 0.0;
    post_shortcut_flowtime_ = 0.0;
    post_shortcut_makespan_ = 0.0;
    t_shortcut_ = 0.0;
    t_init_ = 0.0;
    t_simplify_ = 0.0;
    t_shortcut_check_ = 0.0;
    num_shortcut_checks_ = 0;
    num_valid_shortcuts_ = 0;
    flowtime_improv_ = 0.0;
        
}

TPG::TPG(const TPG &tpg) {
    config_ = tpg.config_;
    dt_ = tpg.dt_;
    num_robots_ = tpg.num_robots_;
    type2Edges_ = tpg.type2Edges_;
    start_nodes_ = tpg.start_nodes_;
    end_nodes_ = tpg.end_nodes_;
    numNodes_ = tpg.numNodes_;
    solution_ = tpg.solution_;
    pre_shortcut_flowtime_ = tpg.pre_shortcut_flowtime_;
    pre_shortcut_makespan_ = tpg.pre_shortcut_makespan_;
    post_shortcut_flowtime_ = tpg.post_shortcut_flowtime_;
    post_shortcut_makespan_ = tpg.post_shortcut_makespan_;
    t_shortcut_ = tpg.t_shortcut_;
    t_init_ = tpg.t_init_;
    t_simplify_ = tpg.t_simplify_;
    t_shortcut_check_ = tpg.t_shortcut_check_;
    num_shortcut_checks_ = tpg.num_shortcut_checks_;
    collisionCheckMatrix_ = tpg.collisionCheckMatrix_;
}

bool TPG::init(std::shared_ptr<PlanInstance> instance, const std::vector<RobotTrajectory> &solution,
    const TPGConfig &config) {
    dt_ = config.dt;
    config_ = config;
    num_robots_ = instance->getNumberOfRobots();
    solution_ = solution;

    auto t_start = std::chrono::high_resolution_clock::now();

    // 1. populate type 1 nodes
    for (int i = 0; i < num_robots_; i++) {
        double cost = solution_[i].cost;
        int numNodes = std::ceil(cost / dt_) + 1;
        int ind = 0;
        
        std::vector<std::shared_ptr<Node>> nodes_i;
        for (int j = 0; j < numNodes; j++) {
            double time = j * dt_;
            while (ind + 1 < solution_[i].times.size() && solution_[i].times[ind+1] <= time) {
                ind++;
            }
            RobotPose pose_j = instance->initRobotPose(i);
            if (ind + 1 == solution_[i].times.size()) {
                // assuming obstacle stays at the end of the trajectory
                pose_j = solution_[i].trajectory[ind];
            } else {
                double alpha = (time - solution_[i].times[ind]) / (solution_[i].times[ind + 1] - solution_[i].times[ind]);
                pose_j = instance->interpolate(solution_[i].trajectory[ind], solution_[i].trajectory[ind + 1], alpha);
            }

            std::shared_ptr<Node> node = std::make_shared<Node>(i, j);
            node->pose = pose_j;
            nodes_i.push_back(node);
        }

        for (int j = 0; j < numNodes; j++) {
            if (j < numNodes - 1) {
                nodes_i[j]->Type1Next = nodes_i[j + 1];
            }
            if (j > 0) {
                nodes_i[j]->Type1Prev = nodes_i[j - 1];
            }
        }

        start_nodes_.push_back(nodes_i[0]);
        end_nodes_.push_back(nodes_i.back());
        numNodes_.push_back(numNodes);
        pre_shortcut_flowtime_ += (dt_ * (numNodes - 1));
        pre_shortcut_makespan_ = std::max(pre_shortcut_makespan_, dt_ * (numNodes - 1));
    }

    // 2. compute collision matrix
    for (int i = 0; i < num_robots_; i++) {
        std::vector<Eigen::MatrixXi> col_matrices;
        for (int j = i + 1; j < num_robots_; j++) {
            int ni = numNodes_[i];
            int nj = numNodes_[j];
            Eigen::MatrixXi col_matrix(ni, nj);
            col_matrix.setZero(); // 0 is no collision, 1 is collision

            std::shared_ptr<Node> node_i = start_nodes_[i];
            while (node_i != nullptr) {
                std::shared_ptr<Node> node_j = start_nodes_[j];
                while (node_j != nullptr) {
                    col_matrix(node_i->timeStep, node_j->timeStep) = instance->checkCollision({node_i->pose, node_j->pose}, true);
                    node_j = node_j->Type1Next;
                }
                node_i = node_i->Type1Next;
            }

            col_matrices.push_back(col_matrix);
        }
        collisionCheckMatrix_.push_back(col_matrices);
    }

    // 3. populate type 2 edges
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }
            Eigen::MatrixXi col_matrix;
            getCollisionCheckMatrix(i, j, col_matrix);
            std::shared_ptr<Node> node_i = start_nodes_[i];
            while (node_i != nullptr) {
                std::shared_ptr<Node> node_j = start_nodes_[j];
                bool inCollision = false;
                while (node_j != nullptr && node_j->timeStep <= node_i->timeStep) {
                    if (col_matrix(node_i->timeStep, node_j->timeStep) == 1) {
                        inCollision = true;
                    } else if (inCollision) {
                        inCollision = false;
                        std::shared_ptr<type2Edge> edge = std::make_shared<type2Edge>();
                        edge->edgeId = idType2Edges_;
                        edge->nodeFrom = node_j;
                        edge->nodeTo = node_i;
                        node_j->Type2Next.push_back(edge);
                        node_i->Type2Prev.push_back(edge);
                        idType2Edges_++;
                    }
                    node_j = node_j->Type1Next;
                }
                node_i = node_i->Type1Next;
            }
        }
    }

    t_init_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;

    // 4. Simplify the edges with MCP algorithm
    t_start = std::chrono::high_resolution_clock::now();
    
    transitiveReduction();
    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }
    
    int numtype2edges = getTotalType2Edges();
    t_simplify_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;  
    log("TPG initialized with "  + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges.", 
        LogLevel::HLINFO);


    //saveToDotFile("tpg_pre.dot");
    return true;
}


bool TPG::optimize(std::shared_ptr<PlanInstance> instance, const TPGConfig &config) {
    config_ = config;
    auto t_start = std::chrono::high_resolution_clock::now();

    if (config.shortcut) {
        if (config.random_shortcut) {
            findShortcutsRandom(instance, config.shortcut_time);
        } else {
            findShortcuts(instance, config.shortcut_time);
        }

        t_shortcut_ = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start).count() * 1e-6;
                
        transitiveReduction();
        int numtype2edges = getTotalType2Edges();
        log ("TPG after finding shortcuts: " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges.", LogLevel::HLINFO);
        log ("in " + std::to_string(t_shortcut_) + " s.", LogLevel::HLINFO);

        if (config.switch_shortcut) {
            switchShortcuts();
        }

        //saveToDotFile("tpg.dot");

        // trajectory_msgs::JointTrajectory joint_traj;
        // size_t num_joints = 0;
        // for (int i = 0; i < num_robots_; i++ ) {
        //     num_joints += instance->getRobotDOF(i);
        // }
        // joint_traj.joint_names.resize(num_joints);
        // setSyncJointTrajectory(joint_traj, post_shortcut_flowtime_, post_shortcut_makespan_);
        findFlowtimeMakespan(post_shortcut_flowtime_, post_shortcut_makespan_);
    }

    // 5. Print the TPG for debugging purposes
    // Eigen::MatrixXi col_matrix_ij;
    // getCollisionCheckMatrix(0, 1, col_matrix_ij);
    // std::cout << "Collision matrix between 0 and 1" << std::endl;
    // std::cout << col_matrix_ij << std::endl;

    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            log("Robot " + std::to_string(i) + " at time " + std::to_string(node_i->timeStep), LogLevel::DEBUG);
            log("Type 1 Next: " + ((node_i->Type1Next != nullptr) ? std::to_string(node_i->Type1Next->timeStep) : ""), LogLevel::DEBUG);
            log("Type 1 Prev: " + ((node_i->Type1Prev != nullptr) ? std::to_string(node_i->Type1Prev->timeStep) : ""), LogLevel::DEBUG);
            for (auto edge : node_i->Type2Next) {
                log("Type 2 Next: " + std::to_string(edge->nodeTo->robotId) + " " + std::to_string(edge->nodeTo->timeStep), LogLevel::DEBUG);
            }
            for (auto edge : node_i->Type2Prev) {
                log("Type 2 Prev: " + std::to_string(edge->nodeFrom->robotId) + " " + std::to_string(edge->nodeFrom->timeStep), LogLevel::DEBUG);
            }
            node_i = node_i->Type1Next;
        }

    }

    return true;
}

int TPG::getTotalNodes() const {
    int total_nodes = 0;
    for (int i = 0; i < num_robots_; i++) {
        total_nodes += numNodes_[i];
    }
    return total_nodes;
}

int TPG::getTotalType2Edges() const {
    int total_edges = 0;
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            total_edges += node_i->Type2Next.size();
            node_i = node_i->Type1Next;
        }
    }
    return total_edges;
}

void TPG::saveStats(const std::string &filename, const std::string &start_pose, const std::string &goal_pose) const {
    std::ofstream file(filename, std::ios::app);
    file << start_pose << "," << goal_pose << "," 
        << pre_shortcut_flowtime_ << "," << pre_shortcut_makespan_ << "," << post_shortcut_flowtime_ << "," << post_shortcut_makespan_ 
        << "," << t_init_ << "," << t_shortcut_ << "," << t_simplify_ << "," << t_shortcut_check_ << "," << num_shortcut_checks_
        << "," << num_valid_shortcuts_ << std::endl;
    file.close();
}

void TPG::findShortcuts(std::shared_ptr<PlanInstance> instance, double runtime_limit)
{
    // for every robot
    // for every pair of nodes
    // check if this is free of robot-space/robot-object collisions
    // find the dependent parent nodes and child nodes
    // for every other node that is independent with the current pair
    // compute the collision matrix
    // if there is no collision, add the shortcut and remove old nodes
    // add the new nodes to the list of nodes

    double elapsed = 0;
    std::queue<std::shared_ptr<Node>> q_i;
    std::queue<std::shared_ptr<Node>> q_j;
    for (int i = 0; i < num_robots_; i++) {
        q_i.push(start_nodes_[i]);
        if (config_.backward_doubleloop) {
            q_j.push(end_nodes_[i]);
        }
        else {
            q_j.push(start_nodes_[i]->Type1Next);
        }
    }


    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end, updated_reached_end;
    findEarliestReachTime(earliest_t, reached_end);
    if (config_.tight_shortcut) {
        findLatestReachTime(latest_t, reached_end);
        findTightType2Edges(earliest_t, latest_t);
    }
    auto tic = std::chrono::high_resolution_clock::now();


    while (elapsed < runtime_limit) {
        std::shared_ptr<Node> node_i = q_i.front();
        std::shared_ptr<Node> node_j = q_j.front();
        q_i.pop();
        q_j.pop();

        if (config_.backward_doubleloop) {
            if (node_i == node_j) {
                q_i.push(start_nodes_[node_i->robotId]);
                q_j.push(end_nodes_[node_i->robotId]);

                auto toc = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
                continue;
            }
            if (node_i->Type1Next == node_j) {
                q_i.push(node_i->Type1Next);
                q_j.push(end_nodes_[node_i->robotId]);

                auto toc = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
                continue;
            }
        }
        else {

            if (node_j == nullptr) {
                if (node_i->Type1Next == nullptr) {
                    q_i.push(start_nodes_[node_i->robotId]);
                    q_j.push(start_nodes_[node_i->robotId]->Type1Next);
                }
                else {
                    q_i.push(node_i->Type1Next);
                    q_j.push(node_i->Type1Next->Type1Next);
                }
                
                auto toc = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
                continue;
            }
            if (node_i->Type1Next == node_j) {
                q_i.push(node_i);
                q_j.push(node_j->Type1Next);
                
                auto toc = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
                continue;
            }
        }

        Shortcut shortcut(node_i, node_j);

        int robot_id = node_i->robotId;
        preCheckShortcuts(instance, shortcut, earliest_t[robot_id], latest_t[robot_id]);
        if (shortcut.col_type == CollisionType::NONE) {
            std::vector<Eigen::MatrixXi> col_matrix;

            auto tic_inner = std::chrono::high_resolution_clock::now();
            checkShortcuts(instance, shortcut, col_matrix);
            auto inner = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic_inner).count();
            t_shortcut_check_ += (inner * 1e-6);
            num_shortcut_checks_++;

            if (shortcut.col_type == CollisionType::NONE) {
                // add the shortcut
                updateTPG(shortcut, col_matrix);

                // calculate statistics
                findEarliestReachTime(earliest_t, updated_reached_end);

                reached_end = updated_reached_end;
                if (config_.tight_shortcut) {
                    findLatestReachTime(latest_t, reached_end);
                    findTightType2Edges(earliest_t, latest_t);
                }
                num_valid_shortcuts_++;
            }
        }

        if (config_.backward_doubleloop) {
            q_i.push(node_i);
            q_j.push(node_j->Type1Prev);
        }
        else if (config_.forward_doubleloop) {
            q_i.push(node_i);
            q_j.push(node_j->Type1Next);
        }
        else if (config_.forward_singleloop) {
            if (shortcut.col_type == CollisionType::NONE) {
                q_i.push(node_j);
                q_j.push(node_j->Type1Next);
            }
            else {
                q_i.push(node_i);
                q_j.push(node_j->Type1Next);
            }
        }

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
    }
}

void TPG::initSampler() {
    shortcut_sampler_ = std::make_unique<ShortcutSampler>(config_);
    shortcut_sampler_->init(start_nodes_, numNodes_);
}

void TPG::findShortcutsRandom(std::shared_ptr<PlanInstance> instance, double runtime_limit) {

    // randomly sample shortcuts and check if they are valid for time
    double elapsed = 0;
    
    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end, updated_reached_end;
    findEarliestReachTime(earliest_t, reached_end);
    if (config_.tight_shortcut) {
        findLatestReachTime(latest_t, reached_end);
        findTightType2Edges(earliest_t, latest_t);
    }

    initSampler();

    if (config_.progress_file != "") {
        std::ofstream ofs(config_.progress_file, std::ofstream::out);
        ofs << "start_pose,goal_pose,flowtime_pre,makespan_pre,flowtime_post,makespan_post,t_init,t_shortcut,t_mcp,t_check,n_check,n_valid" << std::endl;
        ofs.close();
    }

    auto tic = std::chrono::high_resolution_clock::now();

    while (elapsed < runtime_limit) {
        Shortcut shortcut;
        bool valid = shortcut_sampler_->sample(shortcut);
        if (!valid) {
            auto toc = std::chrono::high_resolution_clock::now();
            elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
            continue;
        }
        assert (!shortcut.expired());

        int robot_id = shortcut.robot_id();
        preCheckShortcuts(instance, shortcut, earliest_t[robot_id], latest_t[robot_id]);
        if (shortcut.col_type == CollisionType::NONE) {
            std::vector<Eigen::MatrixXi> col_matrix;

            auto tic_inner = std::chrono::high_resolution_clock::now();
            checkShortcuts(instance, shortcut, col_matrix);
            auto inner = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - tic_inner).count();
            t_shortcut_check_ += (inner * 1e-6);
            num_shortcut_checks_++;
            log("Time taken for checking shortcuts: " + std::to_string(inner) + " us", LogLevel::DEBUG);

            if (shortcut.col_type == CollisionType::NONE) {
                // add the shortcut
                std::shared_ptr<Node> node_i = shortcut.ni.lock();
                std::shared_ptr<Node> node_j = shortcut.nj.lock();
                log("found shortcut for robot " + std::to_string(robot_id) + " of length " + std::to_string(shortcut.path.size()), LogLevel::DEBUG);
                log("from " + std::to_string(node_i->timeStep) + " to " + std::to_string(node_j->timeStep), LogLevel::DEBUG);
                
                updateTPG(shortcut, col_matrix);
                shortcut_sampler_->init(start_nodes_, numNodes_);

                // calculate statistics
                findEarliestReachTime(earliest_t, updated_reached_end);
                int flowtime_diff = 0;
                for (int ri = 0; ri < num_robots_; ri++) {
                    flowtime_diff += (reached_end[ri] - updated_reached_end[ri]); 
                }
                flowtime_improv_ += (flowtime_diff * dt_);

                reached_end = updated_reached_end;
                if (config_.tight_shortcut) {
                    findLatestReachTime(latest_t, reached_end);
                    findTightType2Edges(earliest_t, latest_t);
                }
                num_valid_shortcuts_++;
                if (config_.progress_file != "") {
                    int flowspan_i = 0;
                    int makespan_i = 0;
                    for (int ri = 0; ri < num_robots_; ri++) {
                        flowspan_i += reached_end[ri];
                        makespan_i = std::max(makespan_i, reached_end[ri]);
                    }

                    post_shortcut_flowtime_ = flowspan_i * dt_;
                    post_shortcut_makespan_ = makespan_i * dt_;
                    saveStats(config_.progress_file);
                }
            }
        }
        if (shortcut.col_type != CollisionType::NONE) {
            shortcut_sampler_->updateFailedShortcut(shortcut);
        }

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
    }
    if (config_.progress_file != "") {
        findFlowtimeMakespan(post_shortcut_flowtime_, post_shortcut_makespan_);
        saveStats(config_.progress_file);
    }

    assert(hasCycle() == false);
}

void TPG::preCheckShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
        const std::vector<int> &earliest_t, const std::vector<int> &latest_t) const {
    
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    // check if there is a shortcut between ni and nj
    assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);

    double timeNeeded = instance->computeDistance(ni->pose, nj->pose) / instance->getVMax(ni->robotId);
    timeNeeded = std::ceil(timeNeeded / dt_) * dt_;
    int shortcutSteps = timeNeeded / dt_ + 1;

    if (shortcutSteps > (nj->timeStep - ni->timeStep)) {
        shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
        log("Shortcut is not needed", LogLevel::DEBUG);
        return;
    }

    // if (config_.helpful_shortcut) {
    //     bool helpful = false;
    //     std::shared_ptr <Node> current = ni->Type1Next;
    //     while (!helpful && current != nj) {
    //         if (current->Type2Next.size() > 0) {
    //             helpful = true;
    //         }
    //         current = current->Type1Next;
    //     }
    //     current = nj;
    //     while (!helpful && current != nullptr) {
    //         if (current->Type2Prev.size() > 0) {
    //             for (auto edge : current->Type2Prev) {
    //                 if (edge->nodeFrom->timeStep >= current->timeStep) {
    //                     break;
    //                 }
    //             }
    //         }
    //         if (current->Type2Next.size() > 0) {
    //             helpful = true;
    //         }
    //         current = current->Type1Next;
    //     }
    //     if (!helpful && (current != nullptr)) {
    //         return false;
    //     }
    // }

    if (config_.tight_shortcut) {
        std::shared_ptr<Node> current = ni;
        bool has_tight_type2_edge = false;
        bool all_tight_type1_edge = (earliest_t[ni->timeStep] == latest_t[ni->timeStep]);
        while (current != nj) {

            for (auto edge : current->Type2Prev) {
                if (edge->tight) {
                    has_tight_type2_edge = true;
                    break;
                }
            }
            for (auto edge : current->Type2Next) {
                if (edge->tight) {
                    has_tight_type2_edge = true;
                    break;
                }
            }
            current = current->Type1Next;
            if (earliest_t[current->timeStep] < latest_t[current->timeStep]) {
                all_tight_type1_edge = false;
            }
        }
        
        if (!has_tight_type2_edge && !all_tight_type1_edge) {
            shortcut.col_type = CollisionType::UNTIGHT;
            log("Shortcut is untight", LogLevel::DEBUG);
            return;
        }
    }

    shortcut.col_type = CollisionType::NONE;
    return;
}

void TPG::checkShortcuts(std::shared_ptr<PlanInstance> instance, Shortcut &shortcut,
     std::vector<Eigen::MatrixXi> &col_matrix) const {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    // check if there is a shortcut between ni and nj
    assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);

    double timeNeeded = instance->computeDistance(ni->pose, nj->pose) / instance->getVMax(ni->robotId);
    timeNeeded = std::ceil(timeNeeded / dt_) * dt_;
    int shortcutSteps = timeNeeded / dt_ + 1;

    if (shortcutSteps > (nj->timeStep - ni->timeStep)) {
        shortcut.col_type = CollisionType::NO_NEED; // no need for shortcut
        return;
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
            std::shared_ptr<Node> node_j = start_nodes_[j];
            while (node_j != nullptr) {
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
    for (int i = 1; i < shortcutSteps - 1; i++) {
        RobotPose pose_i = shortcut.path[i - 1];
        for (int j = 0; j < num_robots_; j++) {
            if (j == ni->robotId) {
                continue;
            }
            std::shared_ptr<Node> node_j = start_nodes_[j];
            while (node_j != nullptr) {
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

    shortcut.col_type = CollisionType::NONE;
    return;
}

void TPG::updateTPG(const Shortcut &shortcut, const std::vector<Eigen::MatrixXi> &col_matrix) {
    auto ni = shortcut.ni.lock();
    auto nj = shortcut.nj.lock();
    std::shared_ptr<Node> n_prev = ni->Type1Next;
    // remove dangling type-2 edge skipped by the shortcut
    while (n_prev != nullptr && n_prev != nj) {
        for (auto edge : n_prev->Type2Next) {
            edge->nodeTo->Type2Prev.erase(std::remove_if(edge->nodeTo->Type2Prev.begin(), edge->nodeTo->Type2Prev.end(),
                [edge](std::shared_ptr<type2Edge> e) { return e->edgeId == edge->edgeId; }), edge->nodeTo->Type2Prev.end());
        }
        for (auto edge : n_prev->Type2Prev) {
            edge->nodeFrom->Type2Next.erase(std::remove_if(edge->nodeFrom->Type2Next.begin(), edge->nodeFrom->Type2Next.end(),
                [edge](std::shared_ptr<type2Edge> e) { return e->edgeId == edge->edgeId; }), edge->nodeFrom->Type2Next.end());
        }
        n_prev = n_prev->Type1Next;
    }

    // attach the shortcut to the current TPG
    n_prev = ni;
    for (int i = 0; i < shortcut.path.size(); i++) {
        std::shared_ptr<Node> node = std::make_shared<Node>(ni->robotId, n_prev->timeStep + 1);
        node->pose = shortcut.path[i];
        n_prev->Type1Next = node;
        node->Type1Prev = n_prev;
        node->actId = nj->actId;
        
        n_prev = node;
    }
    n_prev->Type1Next = nj;
    nj->Type1Prev = n_prev;
    int nj_prevt = nj->timeStep;

    // update timestamp for the rest of the nodes
    while (n_prev->Type1Next != nullptr) {
        n_prev->Type1Next->timeStep = n_prev->timeStep + 1;
        n_prev = n_prev->Type1Next;
    }
    end_nodes_[ni->robotId] = n_prev;
    int numNodes_prev = numNodes_[ni->robotId];
    numNodes_[ni->robotId] = n_prev->timeStep + 1;
    int reducedSteps = numNodes_prev - numNodes_[ni->robotId];

    // update the collision matrix
    // for (int j = 0; j < num_robots_; j++) {
    //     if (ni->robotId == j) {
    //         continue;
    //     }
    //     Eigen::MatrixXi col_matrix_ij;
    //     getCollisionCheckMatrix(ni->robotId, j, col_matrix_ij);

    //     Eigen::MatrixXi new_col_matrix_ij(numNodes_[ni->robotId], numNodes_[j]);
    //     new_col_matrix_ij.block(0, 0, ni->timeStep + 1, numNodes_[j]) = col_matrix_ij.block(0, 0, ni->timeStep + 1, numNodes_[j]);
    //     if (col_matrix.size() > j && col_matrix[j].rows() > 0) {
    //         new_col_matrix_ij.block(ni->timeStep + 1, 0, nj->timeStep - 1 - ni->timeStep, numNodes_[j]) = col_matrix[j].block(1, 0, nj->timeStep - 1 - ni->timeStep, numNodes_[j]);
    //     } 
    //     new_col_matrix_ij.block(nj->timeStep, 0, numNodes_[ni->robotId] - nj->timeStep, numNodes_[j]) = col_matrix_ij.block(nj_prevt, 0, numNodes_prev - nj_prevt, numNodes_[j]);

    //     updateCollisionCheckMatrix(ni->robotId, j, new_col_matrix_ij);

    //     if (config_.ignore_far_collisions) {
    //         // add additional type-2 dependencies for ignored collision checking
    //         std::shared_ptr<Node> node_ignored = start_nodes_[j];
    //         int steps = 0;
    //         if ((ni->timeStep - config_.ignore_steps) > 0) {
            
    //             while (node_ignored != nullptr && steps < (ni->timeStep - config_.ignore_steps)) {
    //                 node_ignored = node_ignored->Type1Next;
    //                 steps++;
    //             }
    //             if (node_ignored != nullptr) {
    //                 type2Edge edge_b;
    //                 edge_b.edgeId = idType2Edges_;
    //                 edge_b.nodeFrom = node_ignored;
    //                 edge_b.nodeTo = nj;
    //                 node_ignored->Type2Next.push_back(edge_b);
    //                 nj->Type2Prev.push_back(edge_b);
    //                 idType2Edges_++;
    //             }
    //         }

    //         if ((nj->timeStep +  config_.ignore_steps) < numNodes_[j]) {
    //             while (node_ignored != nullptr && steps <= (nj->timeStep + config_.ignore_steps)) {
    //                 node_ignored = node_ignored->Type1Next;
    //                 steps++;
    //             }
    //             if (node_ignored != nullptr) {
    //                 type2Edge edge_f;
    //                 edge_f.edgeId = idType2Edges_;
    //                 edge_f.nodeFrom = nj;
    //                 edge_f.nodeTo = node_ignored;
    //                 nj->Type2Next.push_back(edge_f);
    //                 node_ignored->Type2Prev.push_back(edge_f);
    //                 idType2Edges_++;
    //             }
    //         }
    //     }
    // }

    
}

void TPG::switchShortcuts() {
    for (int i = 0; i < num_robots_; i++) {
        for (int j = 0; j < num_robots_; j++) {
            if (i == j) {
                continue;
            }

            Eigen::MatrixXi col_matrix_ij;
            getCollisionCheckMatrix(i, j, col_matrix_ij);
            // print the collision matrix

            // update the type 2 edges
            std::shared_ptr<Node> node_i = start_nodes_[i];
            while (node_i->Type1Next != nullptr) {
                // check all other robots, remove incoming type-2 edges that appear after current timestamp, and add new outgoing type-2 edges 
                for (int k = node_i->Type2Prev.size() - 1; k >= 0; k--) {
                    auto edge = node_i->Type2Prev[k];
                    if (edge->switchable == false) {
                        continue;
                    }
                    auto nodeFrom = edge->nodeFrom;
                    auto edgeId = edge->edgeId;
                    int minWaitTime = nodeFrom->timeStep - node_i->timeStep;
                    if (minWaitTime > 0) {
                        // will switch the type2 dependency from node_i->next to node_j_free
                        std::shared_ptr<Node> node_j_free = nodeFrom; 
                        while (node_j_free->Type1Prev != nullptr) {
                            node_j_free = node_j_free->Type1Prev;
                            if (col_matrix_ij(node_i->timeStep, node_j_free->timeStep) == 0) {
                                int minSwitchedWaitTime = node_i->timeStep + 1 - node_j_free->timeStep;
                                if (minSwitchedWaitTime < minWaitTime) {
                                    std::shared_ptr<type2Edge> newEdge = std::make_shared<type2Edge>();
                                    newEdge->edgeId = idType2Edges_;
                                    newEdge->nodeFrom = node_i->Type1Next;
                                    newEdge->nodeTo = node_j_free;
                                    node_i->Type1Next->Type2Next.push_back(newEdge);
                                    node_j_free->Type2Prev.push_back(newEdge);
                                    idType2Edges_++;
                                
                                    node_i->Type2Prev.erase(node_i->Type2Prev.begin() + k);
                                    nodeFrom->Type2Next.erase(std::remove_if(nodeFrom->Type2Next.begin(), nodeFrom->Type2Next.end(), 
                                        [edgeId](std::shared_ptr<type2Edge> e) { return e->edgeId == edgeId; }), nodeFrom->Type2Next.end());
                                    
                                    // TODO: fix when type2 edges are inconsistent

                                    log("Removed type 2 dependency from Robot " + std::to_string(nodeFrom->robotId) + " at time "
                                        + std::to_string(nodeFrom->timeStep) + " -> Robot " + std::to_string(node_i->robotId) + " at time " 
                                        + std::to_string(node_i->timeStep), LogLevel::INFO);
                                    log("Added type 2 dependency from Robot " + std::to_string(node_i->robotId) + " at time "
                                        + std::to_string(node_i->Type1Next->timeStep) + " -> Robot " + std::to_string(node_j_free->robotId) + " at time " 
                                        + std::to_string(node_j_free->timeStep), LogLevel::INFO);
                                    break;
                                }
                            }
                        }
                    }
                }

                node_i = node_i->Type1Next;
            }
        }

        // assert (hasCycle() == false);
    }

}


bool TPG::hasCycle() const 
{
    // Initialized visited matrix
    std::vector<std::vector<bool>> visited;
    std::vector<std::vector<bool>> inStack;

    std::stack<std::shared_ptr<Node>> stack;

    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        std::vector<bool> s(numNodes_[i], false);
        visited.push_back(v);
        inStack.push_back(s);
    }

    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            if (!visited[i][node_i->timeStep]) {
                stack.push(node_i);
                // a depth first search to check if there is a cycle
                // by checking if an element is in the stack
                while (!stack.empty()) {
                    std::shared_ptr<Node> node_v = stack.top();
                    if (!visited[node_v->robotId][node_v->timeStep]) {
                        visited[node_v->robotId][node_v->timeStep] = true;
                        inStack[node_v->robotId][node_v->timeStep] = true;
                    }
                    else {
                        inStack[node_v->robotId][node_v->timeStep] = false;
                        stack.pop();
                        continue;
                    }

                    if (node_v->Type1Next != nullptr) {
                        std::shared_ptr<Node> node_next = node_v->Type1Next;
                        if (!visited[node_next->robotId][node_next->timeStep]) {
                            stack.push(node_next);
                        } else if (inStack[node_next->robotId][node_next->timeStep]) {
                            return true;
                        }
                    }
                    for (auto edge : node_v->Type2Next) {
                        if (!visited[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
                            stack.push(edge->nodeTo);
                        } else if (inStack[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
                            return true;
                        }
                    }
                }
            }
            node_i = node_i->Type1Next;
        }
    }
    return false;
}

bool TPG::dfs(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, std::vector<std::vector<bool>> &visited) const
{
    // DFS function to check if there is a path from ni to nj
    visited[ni->robotId][ni->timeStep] = true;
    if (ni->Type1Next != nullptr) {
        std::shared_ptr<Node> ni_next = ni->Type1Next;
        
        // search the next type1 node
        if (ni_next == nj) {
            return true;
        }
        if (!visited[ni_next->robotId][ni_next->timeStep]) {
            if (dfs(ni_next, nj, visited)) {
                return true;
            }
        }
    }

    // search the type2 node neighbors
    for (auto edge : ni->Type2Next) {
        if (edge->nodeTo == nj) {
            return true;
        }
        if (!visited[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
            if (dfs(edge->nodeTo, nj, visited)) {
                return true;
            }
        }
    }
    return false;
}

void TPG::findFlowtimeMakespan(double &flowtime, double &makespan)
{
    // trajectory_msgs::JointTrajectory joint_traj;
    // size_t num_joints = 0;
    // for (int i = 0; i < num_robots_; i++ ) {
    //     num_joints += instance->getRobotDOF(i);
    // }
    // joint_traj.joint_names.resize(num_joints);
    // setSyncJointTrajectory(joint_traj, flowtime, makespan);

    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end;
    findEarliestReachTime(earliest_t, reached_end);
    int flowspan_i = 0, makespan_i = 0;
    for (int i = 0; i < num_robots_; i++) {
        flowspan_i += reached_end[i];
        makespan_i = std::max(makespan_i, reached_end[i]);
    }

    flowtime = flowspan_i * dt_;
    makespan = makespan_i * dt_;
}

void TPG::findEarliestReachTime(std::vector<std::vector<int>> &reached_t, std::vector<int> &reached_end)
{
    reached_t.clear();
    reached_end.clear();
    std::vector<std::shared_ptr<Node>> nodes;
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        nodes.push_back(node_i);

        std::vector<int> v(numNodes_[i], -1);
        reached_t.push_back(v);
        reached_end.push_back(-1);
    }

    int flowtime_i = 0;
    bool allReached = false;
    int j = 0;
    while(!allReached) {

        for (int i = 0; i < num_robots_; i++) {
            if (reached_t[i][nodes[i]->timeStep] == -1) {
                reached_t[i][nodes[i]->timeStep] = j;
            }
        }
        for (int i = 0; i < num_robots_; i++) {
            if (nodes[i]->Type1Next != nullptr) {
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

    // print all the reached times
    // std::cout << "Earliest reach time:\n";
    // for (int i = 0; i < num_robots_; i++) {
    //     for (int j = 0; j < numNodes_[i]; j++) {
    //         std::cout << reached_t[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }

}

void TPG::findLatestReachTime(std::vector<std::vector<int>> &reached_t, const std::vector<int> &reached_end)
{
    reached_t.clear();
    std::vector<std::shared_ptr<Node>> nodes;
    int j = 0;
    for (int i = 0; i < num_robots_; i++) { 
        std::shared_ptr<Node> node_i = end_nodes_[i];
        nodes.push_back(node_i);

        std::vector<int> v(numNodes_[i], -1);
        v.back() = reached_end[i];
        reached_t.push_back(v);
        j = std::max(j, reached_end[i]);
    }
    if (config_.tight_shortcut_makespan) {
        for (int i = 0; i < num_robots_; i++) {
            reached_t[i].back() = j;
        }
    }

    bool allReached = false;

    while (!allReached) {
        for (int i = 0; i < num_robots_; i++) {

            if (reached_t[i][nodes[i]->timeStep] == -1) {
                reached_t[i][nodes[i]->timeStep] = j;
            }
        }

        for (int i = 0; i < num_robots_; i++) {
            if ((reached_t[i][nodes[i]->timeStep] >= j) && (nodes[i]->Type1Prev != nullptr)) {
                bool safe = true;
                for (auto edge : nodes[i]->Type1Prev->Type2Next) {
                    if (reached_t[edge->nodeTo->robotId][edge->nodeTo->timeStep] == -1) {
                        safe = false;
                        break;
                    }
                }
                if (safe) {
                    nodes[i] = nodes[i]->Type1Prev;
                }
            }
        }
        allReached = true;
        for (int i = 0; i < num_robots_; i++) {
            allReached &= (reached_t[i][0] != -1);
        }
        j--;
    }
    
    // print all the reached times
    // std::cout << "Latest reach time:\n";
    // for (int i = 0; i < num_robots_; i++) {
    //     for (int j = 0; j < numNodes_[i]; j++) {
    //         std::cout << reached_t[i][j] << " ";
    //     }
    //     std::cout << std::endl;
    // }
}

void TPG::findTightType2Edges(const std::vector<std::vector<int>> &earliest_t, const std::vector<std::vector<int>> &latest_t)
{
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node = start_nodes_[i];

        while (node->Type1Next != nullptr) {
            int dt = earliest_t[i][node->Type1Next->timeStep] - earliest_t[i][node->timeStep];
    
            for (auto &edge : node->Type1Next->Type2Prev) {
                edge->tight = false;
            }
    
            if (dt > 1) {
                for (auto &edge : node->Type1Next->Type2Prev) {
                    int t_edge_start = earliest_t[edge->nodeFrom->robotId][edge->nodeFrom->timeStep];
                    int dt_edge = earliest_t[i][node->Type1Next->timeStep] - t_edge_start;
                    if (dt_edge == 1) {
                        edge->tight = true;
                    }
                    else {
                        edge->tight = false;
                    }
                }
            }
            node = node->Type1Next;
        }
    }
}

bool TPG::bfs(std::shared_ptr<Node> ni, std::vector<std::vector<bool>> &visited, bool forward) const
{
    // BFS function to find all the dependent nodes of ni
    std::queue<std::shared_ptr<Node>> q;
    q.push(ni);
    visited[ni->robotId][ni->timeStep] = true;
    
    while (!q.empty()) {
        std::shared_ptr<Node> node = q.front();
        q.pop();
        if (forward) {
            if (node->Type1Next != nullptr && !visited[node->Type1Next->robotId][node->Type1Next->timeStep]) {
                q.push(node->Type1Next);
                visited[node->Type1Next->robotId][node->Type1Next->timeStep] = true;
            }
            for (auto edge : node->Type2Next) {
                if (!visited[edge->nodeTo->robotId][edge->nodeTo->timeStep]) {
                    edge->switchable = false;
                    q.push(edge->nodeTo);
                    visited[edge->nodeTo->robotId][edge->nodeTo->timeStep] = true;
                }
            }
        } else {
            if (node->Type1Prev != nullptr && !visited[node->Type1Prev->robotId][node->Type1Prev->timeStep]) {
                q.push(node->Type1Prev);
                visited[node->Type1Prev->robotId][node->Type1Prev->timeStep] = true;
            }
            for (auto edge : node->Type2Prev) {
                if (!visited[edge->nodeFrom->robotId][edge->nodeFrom->timeStep]) {
                    edge->switchable = false;
                    q.push(edge->nodeFrom);
                    visited[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] = true;
                }
            }
        }
    }
    return true;
}


void TPG::transitiveReduction() {
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            for (int k = node_i->Type2Next.size() - 1; k >= 0; k--) {
                
                std::shared_ptr<type2Edge> edge = node_i->Type2Next[k];
                std::shared_ptr<Node> n_from = node_i;
                std::shared_ptr<Node> n_to = edge->nodeTo;
                
                // Remove this edge temporarily
                int idToRemove = edge->edgeId;
                n_from->Type2Next.erase(std::remove_if(n_from->Type2Next.begin(), n_from->Type2Next.end(), 
                    [idToRemove](std::shared_ptr<type2Edge> element) {return element->edgeId == idToRemove;}), n_from->Type2Next.end());
                n_to->Type2Prev.erase(std::remove_if(n_to->Type2Prev.begin(), n_to->Type2Prev.end(), 
                    [idToRemove](std::shared_ptr<type2Edge> element) {return element->edgeId == idToRemove;}), n_to->Type2Prev.end());

                // Initialized visited matrix
                std::vector<std::vector<bool>> visited;
                for (int i = 0; i < num_robots_; i++) {
                    std::vector<bool> v(numNodes_[i], false);
                    visited.push_back(v);
                }

                // If v is still reachable from u, then the edge (u, v) is transitive
                if (dfs(n_from, n_to, visited)) {
                    // Edge is transitive, remove it permanently
                } else {
                    // Edge is not transitive, add it back
                    n_to->Type2Prev.push_back(edge);
                    n_from->Type2Next.insert(n_from->Type2Next.begin() + k, edge);
                }
            }

            node_i = node_i->Type1Next;
        }
        
    }

}

void TPG::getCollisionCheckMatrix(int robot_i, int robot_j, Eigen::MatrixXi &col_matrix) const {
    if (robot_i > robot_j) {
        col_matrix = collisionCheckMatrix_[robot_j][robot_i - robot_j - 1].transpose();
    }
    else {
        col_matrix = collisionCheckMatrix_[robot_i][robot_j - robot_i - 1];
    }
}

void TPG::updateCollisionCheckMatrix(int robot_i, int robot_j, const Eigen::MatrixXi &col_matrix) {
    if (robot_i > robot_j) {
        collisionCheckMatrix_[robot_j][robot_i - robot_j - 1] = col_matrix.transpose();
    }
    else {
        collisionCheckMatrix_[robot_i][robot_j - robot_i - 1] = col_matrix;
    }
}

bool TPG::saveToDotFile(const std::string& filename) const {
    std::ofstream out(filename);
    out << "digraph G {" << std::endl;

    // define node attributes here
    out << "node [shape=circle];" << std::endl;
    out << "rankdir=LR;" << std::endl;

    // define all the nodes
    for (int i = 0; i < num_robots_; i++) {
        out << "subgraph cluster_" << i << " {" << std::endl;
        out << "label = \"Robot " << i << "\";" << std::endl;
        out << "rank=same;" << std::endl;
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            out << "n" << i << "_" << node_i->timeStep << " [label=\"" << i << "_" << node_i->timeStep << "\"];" << std::endl;
            node_i = node_i->Type1Next;
        }
        node_i = start_nodes_[i];
        out << "n" << i << "_" << node_i->timeStep;
        while (node_i != nullptr) {
            if (node_i->Type1Next != nullptr) {
                out << " -> " << "n" << i << "_" << node_i->Type1Next->timeStep;
            }
            node_i = node_i->Type1Next;
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

bool TPG::moveit_execute(std::shared_ptr<MoveitInstance> instance, 
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) {
    // convert solution to moveit plan and execute
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    trajectory_msgs::JointTrajectory &joint_traj = my_plan.trajectory_.joint_trajectory;
    joint_traj.joint_names = move_group->getVariableNames();
    double flowtime, makespan;
    setSyncJointTrajectory(joint_traj, flowtime, makespan);

    // execute the plan
    move_group->execute(my_plan);

    return true;
}

bool TPG::actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client) {
    moveit_msgs::ExecuteTrajectoryGoal goal;
    
    goal.trajectory.joint_trajectory.joint_names = joint_names;
    double flowtime, makespan;
    setSyncJointTrajectory(goal.trajectory.joint_trajectory, flowtime, makespan);

    auto doneCb = [](const actionlib::SimpleClientGoalState& state,
        const moveit_msgs::ExecuteTrajectoryResultConstPtr& result) {
        log("Trajectory execution action finished: " + state.toString(), LogLevel::INFO);
    };

    client.sendGoal(goal, doneCb);

    bool finished = client.waitForResult(ros::Duration(30.0));
    if (finished) {
        actionlib::SimpleClientGoalState state = client.getState();
        return true;
    } else {
        log("Action did not finish before the time out.", LogLevel::ERROR);
        return false;
    }
}

void TPG::setSyncJointTrajectory(trajectory_msgs::JointTrajectory &joint_traj, double &flowtime, double &makespan) const {
    std::vector<std::shared_ptr<Node>> nodes;
    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        nodes.push_back(node_i);
    }

    std::vector<std::vector<bool>> reached;
    std::vector<bool> reached_end;
    for (int i = 0; i < num_robots_; i++) {
        std::vector<bool> v(numNodes_[i], false);
        reached.push_back(v);
        reached_end.push_back(false);
    }

    int flowtime_i = 0;
    bool allReached = false;
    int j = 0;
    while(!allReached) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions.resize(joint_traj.joint_names.size());
        point.velocities.resize(joint_traj.joint_names.size());
        point.accelerations.resize(joint_traj.joint_names.size());
        point.time_from_start = ros::Duration(j * dt_);
        joint_traj.points.push_back(point);

        for (int i = 0; i < num_robots_; i++) {
            RobotPose pose_j_t = nodes[i]->pose;
            
            for (int d = 0; d < pose_j_t.joint_values.size(); d++) {
                joint_traj.points[j].positions[i*7 + d] = pose_j_t.joint_values[d];
            }
            reached[i][nodes[i]->timeStep] = true;
        }
        for (int i = 0; i < num_robots_; i++) {

            if (nodes[i]->Type1Next != nullptr) {
                bool safe = true;
                for (auto edge : nodes[i]->Type1Next->Type2Prev) {
                    if (reached[edge->nodeFrom->robotId][edge->nodeFrom->timeStep] == false) {
                        safe = false;
                        break;
                    }
                }
                if (safe) {
                    nodes[i] = nodes[i]->Type1Next;
                }
            }
            else if (!reached_end[i]) {
                reached_end[i] = true;
                flowtime_i += j;
            }
        }

        allReached = true;
        for (int i = 0; i < num_robots_; i++) {
            allReached &= reached_end[i];
        }
        j++;
    }

    flowtime = flowtime_i * dt_;
    makespan = (j-1) * dt_;

    // compute velocities and accelerations with central difference
    for (int i = 1; i < joint_traj.points.size() - 1; i++) {
        for (int j = 0; j < joint_traj.joint_names.size(); j++) {
            joint_traj.points[i].velocities[j] = (joint_traj.points[i+1].positions[j] - joint_traj.points[i-1].positions[j]) / (2 * dt_);
            joint_traj.points[i].accelerations[j] = (joint_traj.points[i+1].positions[j] - 2 * joint_traj.points[i].positions[j] + joint_traj.points[i-1].positions[j]) / (dt_ * dt_);
        }
    }

    return;

}

bool TPG::moveit_mt_execute(const std::vector<std::vector<std::string>> &joint_names, std::vector<ros::ServiceClient> &clients) {
    // create one thread for each robot
    std::vector<std::thread> threads;

    for (int i = 0; i < num_robots_; i++) {
        executed_steps_.push_back(std::make_unique<std::atomic<int>>(0));
    }

    for (int i = 0; i < num_robots_; i++) {
        threads.emplace_back(&TPG::moveit_async_execute_thread, this, std::ref(joint_names[i]), std::ref(clients[i]), i);
    }

    log("Waiting for all threads to finish...", LogLevel::INFO);
    for (auto &thread : threads) {
        thread.join();
    }

    return true;
}

void TPG::moveit_async_execute_thread(const std::vector<std::string> &joint_names, ros::ServiceClient &clients, int robot_id) {
    std::shared_ptr<Node> node_i = start_nodes_[robot_id];
    

    while (ros::ok()) {
        
        if (node_i->Type1Next == nullptr) {
            log("Robot " + std::to_string(robot_id) + " reached the end at step " + std::to_string(node_i->timeStep), LogLevel::INFO);
            return;
        }

        // check if we can execute the current node
        bool safe = true;
        for (auto edge : node_i->Type1Next->Type2Prev) {
            if (executed_steps_[edge->nodeFrom->robotId]->load() < edge->nodeFrom->timeStep) {
                safe = false;
            }
        }
        if (!safe) {
            ros::Duration(0.03).sleep();
            continue;
        }
        
        int j = 0;

        moveit_msgs::ExecuteKnownTrajectory srv;
        srv.request.wait_for_execution = true;
        
        auto &joint_traj = srv.request.trajectory.joint_trajectory;
        joint_traj.joint_names = joint_names;

        joint_traj.points.clear();
        bool stop = false;
        do {
            trajectory_msgs::JointTrajectoryPoint point;
            point.time_from_start = ros::Duration(j * dt_);
            point.positions.resize(joint_names.size());
            point.velocities.resize(joint_names.size());
            point.accelerations.resize(joint_names.size());

            for (int d = 0; d < node_i->pose.joint_values.size(); d++) {
                point.positions[d] = node_i->pose.joint_values[d];
            }
            joint_traj.points.push_back(point);

            j++;
            node_i = node_i->Type1Next;
            
            //stop = (node_i->Type1Next == nullptr) || (node_i->Type1Next->Type2Prev.size() > 0) || (node_i->Type2Next.size() > 0);
            stop = (node_i->Type1Next == nullptr);
            if (!stop) {
                for (auto edge : node_i->Type1Next->Type2Prev) {
                    if (executed_steps_[edge->nodeFrom->robotId]->load() < edge->nodeFrom->timeStep) {
                        stop = true;
                    }
                }
            }
        } while (!stop);
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start = ros::Duration(j * dt_);
        point.positions.resize(joint_names.size());
        point.velocities.resize(joint_names.size());
        point.accelerations.resize(joint_names.size());
        for (int d = 0; d < node_i->pose.joint_values.size(); d++) {
            point.positions[d] = node_i->pose.joint_values[d];
        }
        joint_traj.points.push_back(point);

        // compute velocities and accelerations with central difference
        for (int i = 1; i < joint_traj.points.size() - 1; i++) {
            for (int j = 0; j < joint_names.size(); j++) {
                joint_traj.points[i].velocities[j] = (joint_traj.points[i+1].positions[j] - joint_traj.points[i-1].positions[j]) / (2 * dt_);
                joint_traj.points[i].accelerations[j] = (joint_traj.points[i+1].positions[j] - 2 * joint_traj.points[i].positions[j] + joint_traj.points[i-1].positions[j]) / (dt_ * dt_);
            }
        }

        
        // execute the plan now
        bool retry = true;
        while (retry) {
            retry = false;
            // compute th error of the current joint state vs the start state
            double error = 0;
            if (joint_states_.size() > robot_id && joint_states_[robot_id].size() >= joint_traj.points[0].positions.size()) {
                log("Robot " + std::to_string(robot_id) + " start/current errors ", LogLevel::DEBUG);

                std::string error_str = "Error: ";
                for (int d = 0; d < joint_states_[robot_id].size(); d++) {
                    double error_d = std::abs(joint_states_[robot_id][d] - joint_traj.points[0].positions[d]);
                    error += error_d;
                    error_str += std::to_string(error_d) + " ";
                }
                log(error_str, LogLevel::DEBUG);
                //TODO: check if this hack that set the joint_states to the start state is still necessary
                for (int d = 0; d < joint_traj.points[0].positions.size(); d++) {
                    joint_traj.points[0].positions[d] = joint_states_[robot_id][d];
                }
            }

            // Call the service to execute the trajectory
            bool result = clients.call(srv);
            if (!result) {
                log("Failed to call service for robot " + std::to_string(robot_id), LogLevel::ERROR);
                return;
            }
            
            int error_code = srv.response.error_code.val;
            log("Robot " + std::to_string(robot_id) + " traj execute service, code " + std::to_string(error_code), LogLevel::INFO);
            if (error_code == moveit_msgs::MoveItErrorCodes::TIMED_OUT) {
                log("Timeout, retrying...", LogLevel::INFO);
                retry = true;
                ros::Duration(0.01).sleep();
            }
            else if (error_code < 0) {
                return;
            }
            else {
                log("Robot " + std::to_string(robot_id) + " success, moving to the next segment", LogLevel::INFO);
                //executed_steps_[robot_id]->fetch_add(j); // allow following conflict
            }
            
        }
    }
}

void TPG::update_joint_states(const std::vector<double> &joint_states, int robot_id)
{
    while (joint_states_.size() <= robot_id) {
        joint_states_.push_back(std::vector<double>());
    }
    for (int i = 0; i < joint_states.size(); i++) {
        if (joint_states_[robot_id].size() <= i) {
            joint_states_[robot_id].push_back(joint_states[i]);
        } else {
            joint_states_[robot_id][i] = joint_states[i];
        }
    }

    if (executed_steps_.size() > robot_id && executed_steps_[robot_id] != nullptr) {
        int next_step = executed_steps_[robot_id]->load() + 1;
        std::shared_ptr<Node> node_i = start_nodes_[robot_id];
        for (int j = 0; j < next_step; j++) {
            if (node_i->Type1Next != nullptr) {
                node_i = node_i->Type1Next;
            }
        }

        double error = 0;
        for (int i = 0; i < joint_states_[robot_id].size(); i++) {
            error += std::abs(joint_states_[robot_id][i] - node_i->pose.joint_values[i]);
        }
        if (error < 0.1) {
            executed_steps_[robot_id]->fetch_add(1);
            log("Robot " + std::to_string(robot_id) + " reached step " + std::to_string(next_step), LogLevel::INFO);
        }
    }

}


}
