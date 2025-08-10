/**
 * @file tsp_solver_refactored.cpp
 * @author caochao (caochao@buct.edu.cn)
 * @brief TSP (Traveling Salesman Problem) Solver using OR-Tools
 * @version 0.2
 * @date 2025-07-28
 *
 * @copyright Copyright (c) 2019-2025
 *
 */

#include "tsp_solver/tsp_solver_refactored.h"
#include <algorithm>
#include <glog/logging.h>
#include <sstream>

namespace tsp_solver_ns
{

    TSPSolverRefactored::TSPSolverRefactored(DataModel data) : data_(std::move(data)), solution_(nullptr)
    {
        // Validate input data
        if (data_.distance_matrix.empty())
        {
            LOG(WARNING) << "Empty distance matrix provided to TSPSolver";
        }
        else
        {
            // Check that matrix is square
            size_t matrix_size = data_.distance_matrix.size();
            for (const auto& row : data_.distance_matrix)
            {
                if (row.size() != matrix_size)
                {
                    LOG(WARNING) << "Distance matrix is not square";
                    break;
                }
            }
        }

        // Create Routing Index Manager
        manager_ = std::make_unique<RoutingIndexManager>(data_.distance_matrix.size(), data_.num_vehicles, data_.depot);

        // Create Routing Model.
        routing_ = std::make_unique<RoutingModel>(*manager_);
    }

    void TSPSolverRefactored::Solve()
    {
        // Register transit callback
        const int transit_callback_index = routing_->RegisterTransitCallback(
            [this](int64 from_index, int64 to_index) -> int64
            {
                // Convert from routing variable Index to distance matrix NodeIndex.
                const auto from_node = manager_->IndexToNode(from_index).value();
                const auto to_node = manager_->IndexToNode(to_index).value();

                // Bounds checking
                if (from_node >= static_cast<int>(data_.distance_matrix.size()) ||
                    to_node >= static_cast<int>(data_.distance_matrix.size()))
                {
                    LOG(ERROR) << "Node index out of bounds in transit callback";
                    return 0;
                }

                if (from_node < 0 || to_node < 0)
                {
                    LOG(ERROR) << "Negative node index in transit callback";
                    return 0;
                }

                return data_.distance_matrix[from_node][to_node];
            });

        // Define the cost of each arc.
        routing_->SetArcCostEvaluatorOfAllVehicles(transit_callback_index);

        // Setting the first solution heuristic.
        RoutingSearchParameters searchParameters = DefaultRoutingSearchParameters();
        searchParameters.set_first_solution_strategy(FirstSolutionStrategy::PATH_CHEAPEST_ARC);
        searchParameters.set_log_search(true); // Enable search logging

        // Solve the problem.
        solution_ = routing_->SolveWithParameters(searchParameters);

        if (solution_ == nullptr)
        {
            LOG(ERROR) << "Failed to solve TSP problem";
        }
    }

    void TSPSolverRefactored::PrintSolution() const
    {
        if (solution_ == nullptr)
        {
            LOG(WARNING) << "No solution available to print";
            return;
        }

        // Inspect solution.
        LOG(INFO) << "Objective: " << (solution_->ObjectiveValue()) / 10.0 << " meters";
        int64 index = routing_->Start(0);
        LOG(INFO) << "Route:";
        int64 distance{0};
        std::stringstream route;
        while (routing_->IsEnd(index) == false)
        {
            route << manager_->IndexToNode(index).value() << " -> ";
            const int64 previous_index = index;
            index = solution_->Value(routing_->NextVar(index));
            distance += routing_->GetArcCostForVehicle(previous_index, index, 0LL);
        }
        LOG(INFO) << route.str() << manager_->IndexToNode(index).value();
        LOG(INFO) << "Route distance: " << distance / 10.0 << " meters";
        LOG(INFO) << "Problem solved in " << routing_->solver()->wall_time() << "ms";
    }

    int TSPSolverRefactored::getComputationTime() const
    {
        if (routing_ && routing_->solver())
        {
            return routing_->solver()->wall_time();
        }
        return -1; // Indicate error
    }

    void TSPSolverRefactored::getSolutionNodeIndex(std::vector<int>& node_index, bool has_dummy) const
    {
        node_index.clear();

        if (solution_ == nullptr || routing_ == nullptr || manager_ == nullptr)
        {
            LOG(WARNING)
                << "Cannot get solution node indices: solver not properly initialized or no solution available";
            return;
        }

        int index = routing_->Start(0);
        while (routing_->IsEnd(index) == false)
        {
            node_index.push_back(manager_->IndexToNode(index).value());
            index = solution_->Value(routing_->NextVar(index));
        }

        if (has_dummy)
        {
            if (data_.distance_matrix.empty())
            {
                LOG(WARNING) << "Cannot process dummy node: empty distance matrix";
                return;
            }

            const int dummy_node_index = static_cast<int>(data_.distance_matrix.size()) - 1;
            if (node_index.size() > 1 && node_index[1] == dummy_node_index)
            {
                // delete dummy node
                node_index.erase(node_index.begin() + 1);
                // push the start node to the end
                node_index.push_back(node_index[0]);
                // remove the start node at the beginning
                node_index.erase(node_index.begin());
                // reverse the whole array
                std::reverse(node_index.begin(), node_index.end());
            }
            else if (!node_index.empty())
            { // the last node is fake node
                node_index.pop_back(); // pop the last element
            }
        }
    }

    double TSPSolverRefactored::getPathLength() const
    {
        if (solution_ == nullptr)
        {
            LOG(WARNING) << "No solution available to get path length";
            return -1.0;
        }
        return solution_->ObjectiveValue() / 10.0;
    }

} // namespace tsp_solver_ns
