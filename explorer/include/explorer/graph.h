
#pragma once

#include <nav_msgs/Path.h>

#include <Eigen/Core>
#include <vector>

namespace explorer {
    class Graph {
    public:
        explicit Graph(int node_number);
        ~Graph() = default;

        void AddNode(const Eigen::Vector3d& position);
        void SetNodePosition(int node_index, const Eigen::Vector3d& position);
        void AddOneWayEdge(int from_node_index, int to_node_index, double distance);
        void AddTwoWayEdge(int from_node_index, int to_node_index, double distance);
        double GetShortestPath(int from_node_index, int to_node_index, bool get_path,
                               nav_msgs::Path& shortest_path, std::vector<int>& node_indices);

    private:
        bool NodeIndexInRange(int node_index) const {
            return node_index >= 0 && node_index < connection_.size();
        }

        double AStarSearch(int from_node_index, int to_node_index, bool get_path,
                           std::vector<int>& node_indices) const;
        // Node connectivity
        std::vector<std::vector<int>> connection_;
        // Distances between two nodes
        std::vector<std::vector<double>> distance_;
        // Node positions
        std::vector<Eigen::Vector3d> positions_;
    };
}  // namespace explorer
