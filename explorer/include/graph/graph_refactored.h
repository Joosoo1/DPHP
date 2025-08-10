/**
 * @file graph_refactored.h
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that implements a graph with A* pathfinding
 * @version 0.2
 * @date 2025-07-28
 *
 * @copyright Copyright (c) 2021-2025
 *
 */
#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <nav_msgs/Path.h>

namespace explorer
{
    /**
     * @brief A graph class that supports pathfinding algorithms
     *
     * This class implements a directed graph with support for A* pathfinding.
     * It stores nodes with 3D positions and weighted edges between nodes.
     */
    class GraphRefactored
    {
    public:
        /**
         * @brief Construct a new Graph object
         *
         * @param node_number Initial number of nodes in the graph
         */
        explicit GraphRefactored(int node_number);

        /**
         * @brief Destroy the Graph object
         */
        ~GraphRefactored() = default;

        /**
         * @brief Add a node to the graph
         *
         * @param position 3D position of the node
         */
        void AddNode(const Eigen::Vector3d& position);

        /**
         * @brief Set or update the position of a node
         *
         * @param node_index Index of the node to update
         * @param position New 3D position of the node
         */
        void SetNodePosition(int node_index, const Eigen::Vector3d& position);

        /**
         * @brief Add a directed edge between two nodes
         *
         * @param from_node_index Index of the source node
         * @param to_node_index Index of the target node
         * @param distance Weight of the edge
         */
        void AddOneWayEdge(int from_node_index, int to_node_index, double distance);

        /**
         * @brief Add an undirected edge between two nodes
         *
         * @param from_node_index Index of the first node
         * @param to_node_index Index of the second node
         * @param distance Weight of the edge
         */
        void AddTwoWayEdge(int from_node_index, int to_node_index, double distance);

        /**
         * @brief Find the shortest path between two nodes using A* algorithm
         *
         * @param from_node_index Index of the start node
         * @param to_node_index Index of the end node
         * @param get_path Whether to return the actual path
         * @param shortest_path Output path as a sequence of poses
         * @param node_indices Output path as a sequence of node indices
         * @return double Length of the shortest path
         */
        double GetShortestPath(int from_node_index, int to_node_index, bool get_path,
                               nav_msgs::Path& shortest_path, std::vector<int>& node_indices) const;

    private:
        /**
         * @brief Check if a node index is within valid range
         *
         * @param node_index Index to check
         * @return true If index is valid
         * @return false If index is invalid
         */
        bool NodeIndexInRange(int node_index) const;

        /**
         * @brief Implementation of A* pathfinding algorithm
         *
         * @param from_node_index Index of the start node
         * @param to_node_index Index of the end node
         * @param get_path Whether to return the actual path
         * @param node_indices Output path as a sequence of node indices
         * @return double Length of the shortest path
         */
        double AStarSearch(int from_node_index, int to_node_index, bool get_path,
                           std::vector<int>& node_indices) const;

        /// Node connectivity - adjacency list representation
        std::vector<std::vector<int>> connection_;

        /// Distances between connected nodes
        std::vector<std::vector<double>> distance_;

        /// 3D positions of nodes
        std::vector<Eigen::Vector3d> positions_;
    };
} // namespace explorer
