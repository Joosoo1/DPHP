/**
 * @file ILocalCoveragePlanner.h
 * @author your name (you@domain.com)
 * @brief Interface for LocalCoveragePlanner
 * @version 0.1
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <memory>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "explorer/utility/exploration_path.h"

// Include concrete classes instead of forward declaring
#include "explorer/implementation/viewpoint_manager.h"

namespace local_coverage_planner_ns
{
    class ILocalCoveragePlanner
    {
    public:
        virtual ~ILocalCoveragePlanner() = default;

        // Configuration methods
        virtual void SetViewPointManager(std::shared_ptr<viewpoint_manager_ns::ViewPointManager> const& viewpoint_manager) = 0;
        virtual void SetRobotPosition(const Eigen::Vector3d& robot_position) = 0;
        virtual void SetLookAheadPoint(const Eigen::Vector3d& lookahead_point) = 0;

        // Planning methods
        virtual exploration_path_ns::ExplorationPath SolveLocalCoverageProblem(
            const exploration_path_ns::ExplorationPath& global_path,
            int uncovered_point_num,
            int uncovered_frontier_point_num) = 0;

        // Utility methods
        virtual bool IsLocalCoverageComplete() const = 0;
        virtual int GetViewPointSamplingRuntime() const = 0;
        virtual int GetFindPathRuntime() const = 0;
        virtual int GetTSPRuntime() const = 0;

        // Visualization methods
        virtual void GetSelectedViewPointVisCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) const = 0;
    };
}