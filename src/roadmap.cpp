#include "roadmap.h"

/**
 * @brief Initialize the roadmap with the given options
 */
bool Roadmap::init(const RoadmapOptions &options) {
    num_samples_ = options.num_samples;
    max_dist_ = options.max_dist;
    roadmap_ = std::make_shared<Graph>();
}

/**
 * @brief Build the roadmap
 */
bool Roadmap::buildRoadmap() {
    RobotPose newpose;
    std::vector<std::shared_ptr<Vertex>> neighbors;
    
    //add vertices and edges
    while (roadmap_->size < num_samples_) {
        if (instance_->sample(newpose)) {
            
            auto sample = roadmap_->addVertex(newpose);
            neighbors = nearestNeighbors(sample);

            for (auto neighbor: neighbors) {
                if (validateMotion(sample, neighbor)) {
                    roadmap_->addEdge(sample, neighbor);
                }
            }
        }
    }
    return true;
}

/**
 * @brief Finds the nearest neighbors of a vertex from each connected component
 * @param sample: Sample vertex
 */
std::vector<std::shared_ptr<Vertex>> Roadmap::nearestNeighbors(const std::shared_ptr<Vertex> &sample) {
    std::unordered_map<int, int> closest;
    std::unordered_map<int, double> minDist;

    for (auto neighbor : roadmap_->vertices) {
        int comp = roadmap_->getComponent(neighbor);
        double dist = instance_->computeDistance(neighbor->pose, sample->pose);
        if (minDist.find(comp) == minDist.end() || dist < minDist[comp]) {
            minDist[comp] = dist;
            closest[comp] = neighbor->id;
        }
    }

    std::vector<std::shared_ptr<Vertex>> closestNeighbors;
    for (const auto& [comp, id] : closest) {
        closestNeighbors.push_back(roadmap_->vertices[id]);
    }

    return closestNeighbors;
}

/**
 * @brief Validate the motion between two vertices
 * @param v: Vertex u
 * @param neighbor: Vertex v
 */
bool Roadmap::validateMotion(const std::shared_ptr<Vertex> &u, 
                             const std::shared_ptr<Vertex> &v) {
    return (!roadmap_->sameComponent(u, v) &&
            instance_->connect(v->pose, u->pose) && 
            instance_->computeDistance(v->pose, u->pose) < max_dist_);
}

/**
 * @brief Query the roadmap
 */
std::shared_ptr<Graph> Roadmap::queryRoadmap() {
    return roadmap_;
}