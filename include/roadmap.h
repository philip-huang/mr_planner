#ifndef ROADMAP_H
#define ROADMAP_H

#include "instance.h"
#include <vector>
#include <memory>
#include <chrono>
#include <set>
#include <unordered_set>


/**
 * @brief Roadmap options
 * @param num_samples: Number of samples to generate
 * @param max_dist: Maximum distance for connecting vertices
 
 */
struct RoadmapOptions {
    int num_samples = 1000;
    double max_dist = 1.0;

    RoadmapOptions() = default;
    RoadmapOptions(int num_samples, double max_dist)
    {
        this->num_samples = num_samples;
        this->max_dist = max_dist;
    }
};

/**
* @brief Vertex class for the roadmap graph
* @param pose: RobotPose of the vertex
* @param id: Id of the vertex
*/
class Vertex {
    public:
        Vertex(const RobotPose &pose, int id) : pose(pose), id(id) {}

        int id;
        RobotPose pose;
};

/**
 * @brief UnionFind class for the roadmap
 */
class UnionFind {
    public: 
        UnionFind() : parent(0), rank(0, 0) {}

        int find(int x) {
            if (parent[x] != x) parent[x] = find(parent[x]);
            return parent[x];
        }

        void unionSet(int u, int v) {
            int pu = find(u), pv = find(v);
            if (pu != pv) {
                if (rank[pu] > rank[pv]) parent[pv] = pu;
                else if (rank[pu] < rank[pv]) parent[pu] = pv;
                else {
                    parent[pv] = pu;
                    rank[pu]++;
                }
            }
        }

        void addNode(int x) {
            parent.push_back(x);
            rank.push_back(0);
        }

    private:
        std::vector<int> parent;
        std::vector<int> rank;
};

/**
 * @brief Graph class for the roadmap
 * @param vertices: Vertices in the graph
 * @param adjList: Adjacency list for the graph
 * @param uf: UnionFind data structure for the graph
 * @param size: Number of vertices in the graph
 */
class Graph {
    public:
        Graph() {
            uf = UnionFind();
        }

        std::shared_ptr<Vertex> addVertex(const RobotPose &pose) {
            auto vertex = std::make_shared<Vertex>(pose, size);
            vertices.push_back(vertex);
            adjList.emplace_back();
            uf.addNode(size);
            size++;
            return vertex;
        }

        void addEdge(std::shared_ptr<Vertex> u, std::shared_ptr<Vertex> v) {
            adjList[u->id].insert(v);
            adjList[v->id].insert(u);
            uf.unionSet(u->id, v->id);
        }

        std::unordered_set<std::shared_ptr<Vertex>> getNeighbors(std::shared_ptr<Vertex> v) {
            return adjList[v->id];
        }

        int getComponent(std::shared_ptr<Vertex> v) {
            return uf.find(v->id);
        }

        bool sameComponent(std::shared_ptr<Vertex> u, std::shared_ptr<Vertex> v) {
            return getComponent(u) == getComponent(v);
        }

        int size = 0;
        std::vector<std::shared_ptr<Vertex>> vertices;
        std::vector<std::unordered_set<std::shared_ptr<Vertex>>> adjList;
        UnionFind uf;
};

/**
 * @brief Roadmap class for the instance
 * @param instance_: PlanInstance for the roadmap
 * @param robot_id_: Id of the robot
 */
class Roadmap {
    public:
        Roadmap(std::shared_ptr<PlanInstance> instance, int robot_id) :
            instance_(instance), robot_id_(robot_id) {}

        virtual bool init(const RoadmapOptions &options);

        void buildRoadmap();

        std::vector<std::shared_ptr<Vertex>> nearestNeighbors(const std::shared_ptr<Vertex> &sample);
        
        bool validateMotion(const std::shared_ptr<Vertex> &a, 
                            const std::shared_ptr<Vertex> &b);

        bool queryRoadmap(const RobotPose &start_pose, 
                          const RobotPose &goal_pose);

    protected:
        int robot_id_;
        int num_samples_; 
        double max_dist_;
        RobotPose start_pose_;
        RobotPose goal_pose_;
        std::shared_ptr<PlanInstance> instance_;
        std::shared_ptr<Graph> roadmap_;
};  

#endif // ROADMAP_H