#include "tpg.h"
#include "logger.h"

namespace TPG {

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
        
}

bool TPG::init(std::shared_ptr<PlanInstance> instance, const std::vector<RobotTrajectory> &solution,
    const TPGConfig &config) {
    dt_ = config.dt;
    config_ = config;
    num_robots_ = instance->getNumberOfRobots();
    solution_ = solution;

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
                        type2Edge edge;
                        edge.edgeId = idType2Edges_;
                        edge.nodeFrom = node_j;
                        edge.nodeTo = node_i;
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

    // 4. Simplify the edges with MCP algorithm
    auto t_start = std::chrono::high_resolution_clock::now();
    
    transitiveReduction();
    if (hasCycle()) {
        log("Naive TPG already has cycle", LogLevel::ERROR);
        return false;
    }
    
    int numtype2edges = getTotalType2Edges();
    double t_simplify = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();  
    log("TPG initialized with "  + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges.", 
        LogLevel::HLINFO);


    //saveToDotFile("tpg_pre.dot");

    t_start = std::chrono::high_resolution_clock::now();

    if (config.shortcut) {
        if (config.random_shortcut) {
            findShortcutsRandom(instance, config.random_shortcut_time);
        } else {
            findShortcuts(instance);
        }

        t_shortcut_ = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - t_start).count();
                
        transitiveReduction();
        numtype2edges = getTotalType2Edges();
        log ("TPG after finding shortcuts: " + std::to_string(getTotalNodes()) + " nodes and " + std::to_string(numtype2edges) + " type 2 edges.", LogLevel::HLINFO);
        log ("in " + std::to_string(t_shortcut_) + " ms.", LogLevel::HLINFO);

        if (config.switch_shortcut) {
            switchShortcuts();
        }

        //saveToDotFile("tpg.dot");

        trajectory_msgs::JointTrajectory joint_traj;
        size_t num_joints = 0;
        for (int i = 0; i < num_robots_; i++ ) {
            num_joints += instance->getRobotDOF(i);
        }
        joint_traj.joint_names.resize(num_joints);
        setSyncJointTrajectory(joint_traj, post_shortcut_flowtime_, post_shortcut_makespan_);
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
                log("Type 2 Next: " + std::to_string(edge.nodeTo->robotId) + " " + std::to_string(edge.nodeTo->timeStep), LogLevel::DEBUG);
            }
            for (auto edge : node_i->Type2Prev) {
                log("Type 2 Prev: " + std::to_string(edge.nodeFrom->robotId) + " " + std::to_string(edge.nodeFrom->timeStep), LogLevel::DEBUG);
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

void TPG::saveStats(const std::string &filename) const {
    std::ofstream file(filename, std::ios::app);
    file << pre_shortcut_flowtime_ << ", " << pre_shortcut_makespan_ << ", " << post_shortcut_flowtime_ << ", " << post_shortcut_makespan_ 
        << ", " << t_shortcut_ << std::endl;
    file.close();
}

void TPG::findShortcuts(std::shared_ptr<PlanInstance> instance)
{
    // for every robot
    // for every pair of nodes
    // check if this is free of robot-space/robot-object collisions
    // find the dependent parent nodes and child nodes
    // for every other node that is independent with the current pair
    // compute the collision matrix
    // if there is no collision, add the shortcut and remove old nodes
    // add the new nodes to the list of nodes

    for (int i = 0; i < num_robots_; i++) {
        std::shared_ptr<Node> node_i = start_nodes_[i];
        while (node_i != nullptr) {
            std::shared_ptr<Node> node_j = end_nodes_[i];
            while (node_j != nullptr && ((node_j->timeStep - node_i->timeStep) > 1)) {
                std::vector<Eigen::MatrixXi> col_matrix;
                std::vector<RobotPose> shortcut_path;
                auto tic = std::chrono::high_resolution_clock::now();
                bool valid = checkShortcuts(instance, node_i, node_j, shortcut_path, col_matrix);
                auto toc = std::chrono::high_resolution_clock::now();
                log("Time taken for checking shortcuts: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(toc - tic).count()) + " ms", LogLevel::DEBUG);
                if (valid) {
                    // add the shortcut
                    log("found shortcut for robot " + std::to_string(i) + " of length " + std::to_string(shortcut_path.size()), LogLevel::DEBUG);
                    log("from " + std::to_string(node_i->timeStep) + " to " + std::to_string(node_j->timeStep), LogLevel::DEBUG);
                    
                    updateTPG(node_i, node_j, shortcut_path, col_matrix);
                    break;
                }
                node_j = node_j->Type1Prev;
            }
            node_i = node_i->Type1Next;
        }
    }
}

void TPG::findShortcutsRandom(std::shared_ptr<PlanInstance> instance, double runtime_limit) {

    // randomly sample shortcuts and check if they are valid for time
    double elapsed = 0;
    auto tic = std::chrono::high_resolution_clock::now();
    
    std::vector<std::vector<int>> earliest_t, latest_t;
    std::vector<int> reached_end;
    if (config_.tight_shortcut) {
        findEarliestReachTime(earliest_t, reached_end);
        findLatestReachTime(latest_t, reached_end);
        findTightType2Edges(earliest_t, latest_t);
    }

    
    while (elapsed < runtime_limit) {
        int i = std::rand() % num_robots_;
        int startNode = std::rand() % numNodes_[i];
        if (startNode >= numNodes_[i] - 2) {
            auto toc = std::chrono::high_resolution_clock::now();
            elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
            continue;
        }
        int length = std::rand() % (numNodes_[i] - startNode - 2) + 2;
        
        std::shared_ptr<Node> node_i = start_nodes_[i];
        for (int j = 0; j < startNode; j++) {
            node_i = node_i->Type1Next;
            assert(node_i != nullptr);
        }
        std::shared_ptr<Node> node_j = node_i;
        for (int j = 0; j < length; j++) {
            node_j = node_j->Type1Next;
            assert(node_j != nullptr);
        }

        if (config_.helpful_shortcut) {
            bool helpful = false;
            std::shared_ptr <Node> current = node_i->Type1Next;
            while (!helpful && current != node_j) {
                if (current->Type2Next.size() > 0) {
                    helpful = true;
                }
                current = current->Type1Next;
            }
            current = node_j;
            while (!helpful && current != nullptr) {
                if (current->Type2Prev.size() > 0) {
                    for (auto edge : current->Type2Prev) {
                        if (edge.nodeFrom->timeStep >= current->timeStep) {
                            break;
                        }
                    }
                }
                if (current->Type2Next.size() > 0) {
                    helpful = true;
                }
                current = current->Type1Next;
            }
            if (!helpful && (current != nullptr)) {
                auto toc = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
                continue;
            }
        }

        if (config_.tight_shortcut) {
            std::shared_ptr<Node> current = node_i->Type1Next;
            bool has_tight_type2_edge = false;
            bool all_tight_type1_edge = (earliest_t[i][node_i->timeStep] == latest_t[i][node_i->timeStep]);
            while (current != node_j) {
                if (earliest_t[i][current->timeStep] < latest_t[i][current->timeStep]) {
                    all_tight_type1_edge = false;
                }
                for (auto edge : current->Type2Prev) {
                    // std::cout << "tight type 2 edge from robot " << edge.nodeFrom->robotId << " at time " 
                    //     << edge.nodeFrom->timeStep << " to robot " << edge.nodeTo->robotId << " at time " 
                    //     << edge.nodeTo->timeStep << " is " << ((edge.tight) ? "tight" : "loose") << std::endl;
                    if (edge.tight) {
                        has_tight_type2_edge = true;
                        break;
                    }
                }
                current = current->Type1Next;
            }
            if (!has_tight_type2_edge && !all_tight_type1_edge) {
                auto toc = std::chrono::high_resolution_clock::now();
                elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
                continue;
            }
        }

        std::vector<Eigen::MatrixXi> col_matrix;
        std::vector<RobotPose> shortcut_path;

        auto tic_inner = std::chrono::high_resolution_clock::now();
        bool valid = checkShortcuts(instance, node_i, node_j, shortcut_path, col_matrix);
        auto toc_inner = std::chrono::high_resolution_clock::now();        
        log("Time taken for checking shortcuts: " + std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(toc_inner - tic_inner).count()) + " us", LogLevel::DEBUG);

        if (valid) {
            // add the shortcut
            log("found shortcut for robot " + std::to_string(i) + " of length " + std::to_string(shortcut_path.size()), LogLevel::DEBUG);
            log("from " + std::to_string(node_i->timeStep) + " to " + std::to_string(node_j->timeStep), LogLevel::DEBUG);
            
            updateTPG(node_i, node_j, shortcut_path, col_matrix);
            if (config_.tight_shortcut) {
                findEarliestReachTime(earliest_t, reached_end);
                findLatestReachTime(latest_t, reached_end);
                findTightType2Edges(earliest_t, latest_t);
            }
        }

        auto toc = std::chrono::high_resolution_clock::now();
        elapsed = std::chrono::duration_cast<std::chrono::microseconds>(toc - tic).count() * 1e-6;
    }

    std::cout << "after random shortcuts ";
    std::cout << hasCycle() << std::endl;
}

bool TPG::checkShortcuts(std::shared_ptr<PlanInstance> instance, std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, 
    std::vector<RobotPose> &shortcut_path, std::vector<Eigen::MatrixXi> &col_matrix) const {
    // check if there is a shortcut between ni and nj
    assert(ni->robotId == nj->robotId && ni->timeStep < nj->timeStep - 1);

    double timeNeeded = instance->computeDistance(ni->pose, nj->pose) / instance->getVMax(ni->robotId);
    timeNeeded = std::ceil(timeNeeded / dt_) * dt_;
    int shortcutSteps = timeNeeded / dt_ + 1;

    if (shortcutSteps > (nj->timeStep - ni->timeStep)) {
        return false; // no need for shortcut
    }

    for (int i = 1; i < shortcutSteps - 1; i++) {
        double alpha = i * dt_ / timeNeeded;
        RobotPose pose_i = instance->interpolate(ni->pose, nj->pose, alpha);

        // check environment collision
        if (instance->checkCollision({pose_i}, false) == true) {
            return false; // collide with the evnrionment
        }
        shortcut_path.push_back(pose_i);
    }

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
                        return false; // collide with other robots
                    }
                }
                node_j = node_j->Type1Next;
            }
        }
        return true;
    }

    for (int j = 0; j < num_robots_; j++) {
        Eigen::MatrixXi col_matrix_j(shortcutSteps, numNodes_[j]);
        col_matrix_j.setZero();
        col_matrix.push_back(col_matrix_j);
    }

    // check robot-robot collision
    for (int i = 1; i < shortcutSteps - 1; i++) {
        RobotPose pose_i = shortcut_path[i - 1];
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
                        return false; // collide with other robots
                    }
                }
                node_j = node_j->Type1Next;
            }
        }
    }

    return true;
}

void TPG::updateTPG(std::shared_ptr<Node> ni, std::shared_ptr<Node> nj, 
        const std::vector<RobotPose> &shortcut_path, const std::vector<Eigen::MatrixXi> &col_matrix) {
    
    std::shared_ptr<Node> n_prev = ni->Type1Next;
    // remove dangling type-2 edge skipped by the shortcut
    while (n_prev != nullptr && n_prev != nj) {
        for (auto edge : n_prev->Type2Next) {
            edge.nodeTo->Type2Prev.erase(std::remove_if(edge.nodeTo->Type2Prev.begin(), edge.nodeTo->Type2Prev.end(), 
                [edge](const type2Edge &element) {return element.edgeId == edge.edgeId;}), edge.nodeTo->Type2Prev.end());
        }
        for (auto edge : n_prev->Type2Prev) {
            edge.nodeFrom->Type2Next.erase(std::remove_if(edge.nodeFrom->Type2Next.begin(), edge.nodeFrom->Type2Next.end(), 
                [edge](const type2Edge &element) {return element.edgeId == edge.edgeId;}), edge.nodeFrom->Type2Next.end());
        }
        n_prev = n_prev->Type1Next;
    }

    // attach the shortcut to the current TPG
    n_prev = ni;
    for (int i = 0; i < shortcut_path.size(); i++) {
        std::shared_ptr<Node> node = std::make_shared<Node>(ni->robotId, n_prev->timeStep + 1);
        node->pose = shortcut_path[i];
        n_prev->Type1Next = node;
        node->Type1Prev = n_prev;
        
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
    for (int j = 0; j < num_robots_; j++) {
        if (ni->robotId == j) {
            continue;
        }
        Eigen::MatrixXi col_matrix_ij;
        getCollisionCheckMatrix(ni->robotId, j, col_matrix_ij);

        Eigen::MatrixXi new_col_matrix_ij(numNodes_[ni->robotId], numNodes_[j]);
        new_col_matrix_ij.block(0, 0, ni->timeStep + 1, numNodes_[j]) = col_matrix_ij.block(0, 0, ni->timeStep + 1, numNodes_[j]);
        if (col_matrix.size() > j && col_matrix[j].rows() > 0) {
            new_col_matrix_ij.block(ni->timeStep + 1, 0, nj->timeStep - 1 - ni->timeStep, numNodes_[j]) = col_matrix[j].block(1, 0, nj->timeStep - 1 - ni->timeStep, numNodes_[j]);
        } 
        new_col_matrix_ij.block(nj->timeStep, 0, numNodes_[ni->robotId] - nj->timeStep, numNodes_[j]) = col_matrix_ij.block(nj_prevt, 0, numNodes_prev - nj_prevt, numNodes_[j]);

        updateCollisionCheckMatrix(ni->robotId, j, new_col_matrix_ij);

        if (config_.ignore_far_collisions) {
            // add additional type-2 dependencies for ignored collision checking
            std::shared_ptr<Node> node_ignored = start_nodes_[j];
            int steps = 0;
            if ((ni->timeStep - config_.ignore_steps) > 0) {
            
                while (node_ignored != nullptr && steps < (ni->timeStep - config_.ignore_steps)) {
                    node_ignored = node_ignored->Type1Next;
                    steps++;
                }
                if (node_ignored != nullptr) {
                    type2Edge edge_b;
                    edge_b.edgeId = idType2Edges_;
                    edge_b.nodeFrom = node_ignored;
                    edge_b.nodeTo = nj;
                    node_ignored->Type2Next.push_back(edge_b);
                    nj->Type2Prev.push_back(edge_b);
                    idType2Edges_++;
                }
            }

            if ((nj->timeStep +  config_.ignore_steps) < numNodes_[j]) {
                while (node_ignored != nullptr && steps <= (nj->timeStep + config_.ignore_steps)) {
                    node_ignored = node_ignored->Type1Next;
                    steps++;
                }
                if (node_ignored != nullptr) {
                    type2Edge edge_f;
                    edge_f.edgeId = idType2Edges_;
                    edge_f.nodeFrom = nj;
                    edge_f.nodeTo = node_ignored;
                    nj->Type2Next.push_back(edge_f);
                    node_ignored->Type2Prev.push_back(edge_f);
                    idType2Edges_++;
                }
            }
        }
    }

    
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
                    if (edge.switchable == false) {
                        continue;
                    }
                    auto nodeFrom = edge.nodeFrom;
                    auto edgeId = edge.edgeId;
                    int minWaitTime = nodeFrom->timeStep - node_i->timeStep;
                    if (minWaitTime > 0) {
                        // will switch the type2 dependency from node_i->next to node_j_free
                        std::shared_ptr<Node> node_j_free = nodeFrom; 
                        while (node_j_free->Type1Prev != nullptr) {
                            node_j_free = node_j_free->Type1Prev;
                            if (col_matrix_ij(node_i->timeStep, node_j_free->timeStep) == 0) {
                                int minSwitchedWaitTime = node_i->timeStep + 1 - node_j_free->timeStep;
                                if (minSwitchedWaitTime < minWaitTime) {
                                    type2Edge newEdge;
                                    newEdge.edgeId = idType2Edges_;
                                    newEdge.nodeFrom = node_i->Type1Next;
                                    newEdge.nodeTo = node_j_free;
                                    node_i->Type1Next->Type2Next.push_back(newEdge);
                                    node_j_free->Type2Prev.push_back(newEdge);
                                    idType2Edges_++;
                                
                                    node_i->Type2Prev.erase(node_i->Type2Prev.begin() + k);
                                    nodeFrom->Type2Next.erase(std::remove_if(nodeFrom->Type2Next.begin(), nodeFrom->Type2Next.end(), 
                                        [edgeId](const type2Edge &element) {return element.edgeId == edgeId;}), nodeFrom->Type2Next.end());
                                    
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
                        if (!visited[edge.nodeTo->robotId][edge.nodeTo->timeStep]) {
                            stack.push(edge.nodeTo);
                        } else if (inStack[edge.nodeTo->robotId][edge.nodeTo->timeStep]) {
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
        if (edge.nodeTo == nj) {
            return true;
        }
        if (!visited[edge.nodeTo->robotId][edge.nodeTo->timeStep]) {
            if (dfs(edge.nodeTo, nj, visited)) {
                return true;
            }
        }
    }
    return false;
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
                    if (reached_t[edge.nodeFrom->robotId][edge.nodeFrom->timeStep] == -1) {
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
                    if (reached_t[edge.nodeTo->robotId][edge.nodeTo->timeStep] == -1) {
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
                edge.tight = false;
            }
    
            if (dt > 1) {
                for (auto &edge : node->Type1Next->Type2Prev) {
                    int t_edge_start = earliest_t[edge.nodeFrom->robotId][edge.nodeFrom->timeStep];
                    int dt_edge = earliest_t[i][node->Type1Next->timeStep] - t_edge_start;
                    if (dt_edge == 1) {
                        edge.tight = true;
                    }
                    else {
                        edge.tight = false;
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
    
    while (!q.empty()) {
        std::shared_ptr<Node> node = q.front();
        q.pop();
        if (forward) {
            if (node->Type1Next != nullptr && !visited[node->Type1Next->robotId][node->Type1Next->timeStep]) {
                q.push(node->Type1Next);
                visited[node->Type1Next->robotId][node->Type1Next->timeStep] = true;
            }
            for (auto edge : node->Type2Next) {
                if (!visited[edge.nodeTo->robotId][edge.nodeTo->timeStep]) {
                    edge.switchable = false;
                    q.push(edge.nodeTo);
                    visited[edge.nodeTo->robotId][edge.nodeTo->timeStep] = true;
                }
            }
        } else {
            if (node->Type1Prev != nullptr && !visited[node->Type1Prev->robotId][node->Type1Prev->timeStep]) {
                q.push(node->Type1Prev);
                visited[node->Type1Prev->robotId][node->Type1Prev->timeStep] = true;
            }
            for (auto edge : node->Type2Prev) {
                if (!visited[edge.nodeFrom->robotId][edge.nodeFrom->timeStep]) {
                    edge.switchable = false;
                    q.push(edge.nodeFrom);
                    visited[edge.nodeFrom->robotId][edge.nodeFrom->timeStep] = true;
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
                
                type2Edge edge = node_i->Type2Next[k];
                std::shared_ptr<Node> n_from = node_i;
                std::shared_ptr<Node> n_to = edge.nodeTo;
                
                // Remove this edge temporarily
                int idToRemove = edge.edgeId;
                n_from->Type2Next.erase(std::remove_if(n_from->Type2Next.begin(), n_from->Type2Next.end(), 
                    [idToRemove](const type2Edge &element) {return element.edgeId == idToRemove;}), n_from->Type2Next.end());
                n_to->Type2Prev.erase(std::remove_if(n_to->Type2Prev.begin(), n_to->Type2Prev.end(), 
                    [idToRemove](const type2Edge &element) {return element.edgeId == idToRemove;}), n_to->Type2Prev.end());

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
                out << "n" << edge.nodeFrom->robotId << "_" << edge.nodeFrom->timeStep << " -> " << "n" << i << "_" << node_i->timeStep << ";" << std::endl;
            }
            node_i = node_i->Type1Next;
        }
    }

    out << "}" << std::endl;
    out.close();

    std::string command = "dot -Tpng " + filename + " -o " + filename + ".png";
    system(command.c_str());

    return true;
}

bool TPG::moveit_execute(std::shared_ptr<PlanInstance> instance, 
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group) const {
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

bool TPG::actionlib_execute(const std::vector<std::string> &joint_names, TrajectoryClient &client) const {
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
                    if (reached[edge.nodeFrom->robotId][edge.nodeFrom->timeStep] == false) {
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
            if (executed_steps_[edge.nodeFrom->robotId]->load() < edge.nodeFrom->timeStep) {
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
                    if (executed_steps_[edge.nodeFrom->robotId]->load() < edge.nodeFrom->timeStep) {
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
        if (error < 0.05) {
            executed_steps_[robot_id]->fetch_add(1);
            log("Robot " + std::to_string(robot_id) + " reached step " + std::to_string(next_step), LogLevel::DEBUG);
        }
    }

}


}