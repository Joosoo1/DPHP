/**
 * @file IViewpointManager.h
 * @author your name (you@domain.com)
 * @brief Interface for ViewpointManager
 * @version 0.1
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "explorer/utility/CellStatus.h"

// Forward declarations
namespace grid_world_ns
{
    class GridWorld;
}

namespace viewpoint_manager_ns
{
    class IViewpointManager
    {
    public:
        virtual ~IViewpointManager() = default;

        // Initialization and update methods
        virtual bool UpdateRobotPosition(const Eigen::Vector3d& robot_position) = 0;
        virtual void UpdateOrigin() = 0;
        virtual void UpdateViewPointBoundary(const geometry_msgs::Polygon& polygon) = 0;

        // Viewpoint management methods
        virtual int GetViewPointCandidate() = 0;
        virtual Eigen::Vector3d GetOrigin() = 0;
        virtual Eigen::Vector3d GetResolution() const = 0;
        virtual int GetViewPointNum() const = 0;

        // Collision and visibility checks
        virtual bool InCollision(const Eigen::Vector3d& position) = 0;
        virtual bool InCurrentFrameLineOfSight(const Eigen::Vector3d& position) = 0;
        virtual void CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud) = 0;
        virtual void CheckViewPointLineOfSight() = 0;
        virtual void CheckViewPointConnectivity() = 0;

        // Coverage methods
        virtual void UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions) = 0;
        virtual void UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world) = 0;

        // Template methods cannot be virtual in interfaces
        // These will be implemented in the concrete class
        // We'll declare them without implementation to avoid static_assert
        template <class PCLPointType>
        void UpdateViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
        {
            // This is a placeholder - will be implemented in derived class
        }

        template <class PCLPointType>
        void UpdateRolledOverViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
        {
            // This is a placeholder - will be implemented in derived class
        }

        // Utility methods
        virtual bool InLocalPlanningHorizon(const Eigen::Vector3d& position) = 0;
        virtual bool InFOVAndRange(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position) const = 0;

        // Visualization methods
        virtual void GetVisualizationCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) = 0;
        virtual void GetCollisionViewPointVisCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const = 0;
    };
}