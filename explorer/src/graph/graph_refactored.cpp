/**
 * @file graph_refactored.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.2
 * @date 2025-07-28
 *
 * @copyright Copyright (c) 2021-2025
 *
 */

#include <queue>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "utils/misc_utils.h"
#include "graph/graph_refactored.h"

namespace explorer
{
  GraphRefactored::GraphRefactored(int node_number)
  {
    if (node_number < 0)
    {
      ROS_WARN("Graph initialized with negative node number. Setting to 0.");
      node_number = 0;
    }

    connection_.resize(node_number);
    distance_.resize(node_number);
    positions_.resize(node_number);
  }

  void GraphRefactored::AddNode(const Eigen::Vector3d& position)
  {
    connection_.emplace_back();
    distance_.emplace_back();
    positions_.push_back(position);
  }

  void GraphRefactored::SetNodePosition(const int node_index, const Eigen::Vector3d& position)
  {
    if (NodeIndexInRange(node_index))
    {
      positions_[node_index] = position;
    }
    else if (node_index == static_cast<int>(positions_.size()))
    {
      AddNode(position);
    }
    else
    {
      ROS_ERROR_STREAM("Graph::SetNodePosition: node_index: " << node_index
        << " not in range [0, " << positions_.size() - 1 << "]");
    }
  }

  void GraphRefactored::AddOneWayEdge(int from_node_index, int to_node_index, double distance)
  {
    if (!NodeIndexInRange(from_node_index))
    {
      ROS_ERROR_STREAM("Graph::AddOneWayEdge: from_node_index: " << from_node_index
        << " not in range [0, " << connection_.size() - 1 << "]");
      return;
    }

    if (!NodeIndexInRange(to_node_index))
    {
      ROS_ERROR_STREAM("Graph::AddOneWayEdge: to_node_index: " << to_node_index
        << " not in range [0, " << connection_.size() - 1 << "]");
      return;
    }

    if (distance < 0)
    {
      ROS_WARN_STREAM("Graph::AddOneWayEdge: negative distance " << distance << " provided");
    }

    connection_[from_node_index].push_back(to_node_index);
    distance_[from_node_index].push_back(distance);
  }

  void GraphRefactored::AddTwoWayEdge(const int from_node_index, const int to_node_index, const double distance)
  {
    AddOneWayEdge(from_node_index, to_node_index, distance);
    AddOneWayEdge(to_node_index, from_node_index, distance);
  }

  double GraphRefactored::GetShortestPath(const int from_node_index, const int to_node_index, const bool get_path,
                                          nav_msgs::Path& shortest_path, std::vector<int>& node_indices) const
  {
    node_indices.clear();
    shortest_path.poses.clear();

    const double path_length = AStarSearch(from_node_index, to_node_index, get_path, node_indices);

    if (get_path)
    {
      shortest_path.poses.reserve(node_indices.size());
      for (const auto node_index : node_indices)
      {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = positions_[node_index].x();
        pose.pose.position.y = positions_[node_index].y();
        pose.pose.position.z = positions_[node_index].z();
        shortest_path.poses.push_back(pose);
      }
    }

    return path_length;
  }

  bool GraphRefactored::NodeIndexInRange(const int node_index) const
  {
    return node_index >= 0 && node_index < static_cast<int>(connection_.size());
  }

  double GraphRefactored::AStarSearch(int from_node_index, int to_node_index, bool get_path,
                                      std::vector<int>& node_indices) const
  {
    // Validate input parameters
    if (!NodeIndexInRange(from_node_index))
    {
      ROS_ERROR_STREAM("AStarSearch: from_node_index " << from_node_index
        << " out of range [0, " << connection_.size() - 1 << "]");
      return -1.0;
    }

    if (!NodeIndexInRange(to_node_index))
    {
      ROS_ERROR_STREAM("AStarSearch: to_node_index " << to_node_index
        << " out of range [0, " << connection_.size() - 1 << "]");
      return -1.0;
    }

    constexpr double kInf = 9999.0;
    using iPair = std::pair<double, int>;

    std::priority_queue<iPair, std::vector<iPair>, std::greater<>> open_set;
    std::vector<double> g_score(connection_.size(), kInf);
    std::vector<double> f_score(connection_.size(), kInf);
    std::vector<int> previous_node(connection_.size(), -1);
    std::vector<bool> in_open_set(connection_.size(), false);

    // Initialize starting node
    g_score[from_node_index] = 0.0;
    f_score[from_node_index] = (positions_[from_node_index] - positions_[to_node_index]).norm();

    open_set.emplace(f_score[from_node_index], from_node_index);
    in_open_set[from_node_index] = true;

    double shortest_distance = -1.0;

    while (!open_set.empty())
    {
      const int current_node = open_set.top().second;
      open_set.pop();
      in_open_set[current_node] = false;

      // Found the target
      if (current_node == to_node_index)
      {
        shortest_distance = g_score[current_node];
        break;
      }

      // Process neighbors
      const std::vector<int>& neighbors = connection_[current_node];
      const std::vector<double>& distances = distance_[current_node];

      for (size_t i = 0; i < neighbors.size(); i++)
      {
        const int neighbor = neighbors[i];

        // Ensure neighbor index is valid
        if (!NodeIndexInRange(neighbor))
        {
          ROS_ERROR_STREAM("AStarSearch: Invalid neighbor index " << neighbor);
          continue;
        }

        MY_ASSERT(misc_utils_ns::InRange<std::vector<int>>(connection_, neighbor));

        const double tentative_g_score = g_score[current_node] + distances[i];

        if (tentative_g_score < g_score[neighbor])
        {
          previous_node[neighbor] = current_node;
          g_score[neighbor] = tentative_g_score;
          f_score[neighbor] = g_score[neighbor] +
            (positions_[neighbor] - positions_[to_node_index]).norm();

          if (!in_open_set[neighbor])
          {
            open_set.emplace(f_score[neighbor], neighbor);
            in_open_set[neighbor] = true;
          }
        }
      }
    }

    // Reconstruct a path if requested
    if (get_path)
    {
      node_indices.clear();

      // Check if a path was found
      if (previous_node[to_node_index] != -1 || from_node_index == to_node_index)
      {
        // Backtrack from goal to start
        int current = to_node_index;
        while (current != -1)
        {
          node_indices.push_back(current);
          current = previous_node[current];
        }

        // Reverse to get a path from start to goal
        std::reverse(node_indices.begin(), node_indices.end());
      }
    }

    return shortest_distance;
  }
} // namespace explorer
