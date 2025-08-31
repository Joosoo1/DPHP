/**
 * @file ExplorerFactory.h
 * @author your name (you@domain.com)
 * @brief Factory class for creating explorer components
 * @version 0.1
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#pragma once

#include <memory>
#include "explorer/interface/IGridWorld.h"
#include "explorer/interface/IKeyposeGraph.h"
#include "explorer/interface/ILocalCoveragePlanner.h"
#include "explorer/interface/IPlanningEnv.h"
#include "explorer/interface/IViewpointManager.h"

namespace explorer_factory_ns
{
    class ExplorerFactory
    {
    public:
        static std::unique_ptr<grid_world_ns::IGridWorld> CreateGridWorld(ros::NodeHandle& nh);
        static std::shared_ptr<keypose_graph_ns::IKeyposeGraph> CreateKeyposeGraph(ros::NodeHandle& nh);
        static std::unique_ptr<local_coverage_planner_ns::ILocalCoveragePlanner> CreateLocalCoveragePlanner(ros::NodeHandle& nh);
        static std::unique_ptr<planning_env_ns::IPlanningEnv> CreatePlanningEnv(ros::NodeHandle nh, ros::NodeHandle nh_private);
        static std::shared_ptr<viewpoint_manager_ns::IViewpointManager> CreateViewpointManager(ros::NodeHandle& nh);
    };
}