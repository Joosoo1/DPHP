#pragma once

#include <memory>
#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "explorer/utility/CellStatus.h"
#include "explorer/utility/exploration_path.h"

// Include concrete classes instead of forward declaring
#include "explorer/implementation/keypose_graph.h"
#include "explorer/implementation/viewpoint_manager.h"

namespace grid_world_ns
{
    // Forward declaration of enum class - the actual enum is defined elsewhere
    enum class CellStatus;

    class IGridWorld
    {
    public:
        virtual ~IGridWorld() = default;

        // Initialization methods
        virtual void ReadParameters(ros::NodeHandle& nh) = 0;
        virtual bool Initialized() const = 0;

        // Update methods
        virtual void UpdateNeighborCells(const geometry_msgs::Point& robot_position) = 0;
        virtual void UpdateRobotPosition(const geometry_msgs::Point& robot_position) = 0;
        virtual void UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager) = 0;

        // Cell management methods
        virtual int GetCellInd(double qx, double qy, double qz) const = 0;
        virtual grid_world_ns::CellStatus GetCellStatus(int cell_ind) const = 0;
        virtual void SetCellStatus(int cell_ind, grid_world_ns::CellStatus status) const = 0;
        virtual geometry_msgs::Point GetCellPosition(int cell_ind) const = 0;
        virtual void SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position) const = 0;
        virtual geometry_msgs::Point GetCellRobotPosition(int cell_ind) const = 0;
        virtual void CellAddVisitCount(int cell_ind) const = 0;
        virtual int GetCellStatusCount(grid_world_ns::CellStatus status) const = 0;
        virtual bool IsRobotPositionSet(int cell_ind) const = 0;
        virtual void Reset() const = 0;

        // Path and connectivity methods
        virtual exploration_path_ns::ExplorationPath
        SolveGlobalTSP(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                       std::vector<int>& ordered_cell_indices,
                       const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph = nullptr) = 0;

        virtual void AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                            const std::shared_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph) const = 0;

        // Visualization methods
        virtual void GetMarker(visualization_msgs::Marker& marker) const = 0;
        virtual void GetVisualizationCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const = 0;

        // Utility methods
        virtual void SetHomePosition(const Eigen::Vector3d& home_position) = 0;
        virtual bool HomeSet() const = 0;
        virtual bool IsReturningHome() const = 0;
    };
}