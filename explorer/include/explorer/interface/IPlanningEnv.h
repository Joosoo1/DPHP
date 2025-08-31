#pragma once

#include <memory>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Forward declarations
namespace lidar_model_ns
{
    class LiDARModel;
}

// Include the concrete class instead of forward declaring
#include "explorer/implementation/viewpoint_manager.h"

namespace planning_env_ns
{
    class IPlanningEnv
    {
    public:
        virtual ~IPlanningEnv() = default;

        // Initialization methods
        virtual double GetPlannerCloudResolution() const = 0;
        virtual void SetUseFrontier(const bool use_frontier) = 0;

        // Update methods
        virtual void UpdateRobotPosition(const geometry_msgs::Point& robot_position) = 0;
        virtual void UpdateCoverageBoundary(const geometry_msgs::Polygon& polygon) = 0;

        // Template methods cannot be virtual in interfaces
        // These will be implemented in the concrete class
        template <class PCLPointType>
        void UpdateRegisteredCloud(typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
        {
            // This is a placeholder - will be implemented in derived class
        }

        template <class PCLPointType>
        void UpdateKeyposeCloud(typename pcl::PointCloud<PCLPointType>::Ptr& keypose_cloud)
        {
            // This is a placeholder - will be implemented in derived class
        }

        // Collision methods
        virtual pcl::PointCloud<pcl::PointXYZI>::Ptr GetCollisionCloud() = 0;

        // Coverage methods
        virtual void UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                      const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager) const = 0;

        virtual void GetUncoveredArea(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                     int& uncovered_point_num, int& uncovered_frontier_point_num) const = 0;

        // Point cloud methods
        virtual pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr GetStackedCloud() = 0;
        virtual pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr GetDiffCloud() = 0;
        virtual pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr GetPlannerCloud() = 0;

        virtual Eigen::Vector3d GetPointCloudManagerNeighborCellsOrigin() = 0;

        // Visualization methods
        virtual void GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const = 0;
        virtual void PublishUncoveredCloud() const = 0;
        virtual void PublishUncoveredFrontierCloud() const = 0;
    };
}