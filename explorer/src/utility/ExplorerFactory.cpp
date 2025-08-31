/**
 * @file ExplorerFactory.cpp
 * @author your name (you@domain.com)
 * @brief Implementation of factory class for creating explorer components
 * @version 0.1
 * @date 2025-08-31
 * 
 * @copyright Copyright (c) 2025
 * 
 */
#include "explorer/utility/ExplorerFactory.h"
#include "explorer/implementation/grid_world.h"
#include "explorer/implementation/keypose_graph.h"
#include "explorer/implementation/local_coverage_planner.h"
#include "explorer/implementation/planning_env.h"
#include "explorer/implementation/viewpoint_manager.h"

namespace explorer_factory_ns
{
    std::unique_ptr<grid_world_ns::IGridWorld> ExplorerFactory::CreateGridWorld(ros::NodeHandle& nh)
    {
        return std::make_unique<grid_world_ns::GridWorld>(nh);
    }

    std::shared_ptr<keypose_graph_ns::IKeyposeGraph> ExplorerFactory::CreateKeyposeGraph(ros::NodeHandle& nh)
    {
        return std::make_unique<keypose_graph_ns::KeyposeGraph>(nh);
    }

    std::unique_ptr<local_coverage_planner_ns::ILocalCoveragePlanner> ExplorerFactory::CreateLocalCoveragePlanner(ros::NodeHandle& nh)
    {
        return std::make_unique<local_coverage_planner_ns::LocalCoveragePlanner>(nh);
    }

    std::unique_ptr<planning_env_ns::IPlanningEnv> ExplorerFactory::CreatePlanningEnv(ros::NodeHandle nh, ros::NodeHandle nh_private)
    {
        return std::make_unique<planning_env_ns::PlanningEnv>(nh, nh_private);
    }

    std::shared_ptr<viewpoint_manager_ns::IViewpointManager> ExplorerFactory::CreateViewpointManager(ros::NodeHandle& nh)
    {
        return std::make_shared<viewpoint_manager_ns::ViewPointManager>(nh);
    }
}