/**
 * @file IKeyposeGraph.h
 * @author your name (you@domain.com)
 * @brief Interface for KeyposeGraph
 * @version 0.1
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Include concrete classes instead of forward declaring
#include "explorer/implementation/planning_env.h"
#include "explorer/implementation/viewpoint_manager.h"

namespace keypose_graph_ns
{
    class IKeyposeGraph
    {
    public:
        virtual ~IKeyposeGraph() = default;

        // Initialization methods
        virtual void ReadParameters(ros::NodeHandle& nh) = 0;

        // Node management methods
        virtual int AddKeyposeNode(const nav_msgs::Odometry& keypose, const planning_env_ns::PlanningEnv& planning_env) = 0;
        virtual int GetClosestNodeInd(const geometry_msgs::Point& point) = 0;
        virtual geometry_msgs::Point GetClosestNodePosition(const geometry_msgs::Point& point) = 0;

        // Connectivity methods
        virtual void CheckLocalCollision(const geometry_msgs::Point& robot_position,
                                        const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager) = 0;
        virtual void CheckConnectivity(const geometry_msgs::Point& robot_position) = 0;

        // Path methods
        virtual bool GetShortestPathWithMaxLength(const geometry_msgs::Point& start_point, 
                                                  const geometry_msgs::Point& target_point,
                                                  double max_path_length, 
                                                  bool get_path, 
                                                  nav_msgs::Path& path) = 0;

        // Visualization methods
        virtual void GetMarker(visualization_msgs::Marker& node_marker, visualization_msgs::Marker& edge_marker) = 0;
        virtual void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) = 0;

        // Utility methods
        virtual void SetAllowVerticalEdge(const bool allow_vertical_edge) = 0;
    };
}