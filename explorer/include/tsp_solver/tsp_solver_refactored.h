/**
 * @file tsp_solver_refactored.h
 * @author caochao (caochao@buct.edu.cn)
 * @brief TSP (Traveling Salesman Problem) Solver using OR-Tools
 * @version 0.2
 * @date 2025-07-28
 *
 * @copyright Copyright (c) 2019-2025
 *
 */
#pragma once

#include <cmath>
#include <vector>
#include <memory>
#include <ortools/constraint_solver/routing.h>
#include <ortools/constraint_solver/routing_enums.pb.h>
#include <ortools/constraint_solver/routing_index_manager.h>
#include <ortools/constraint_solver/routing_parameters.h>

using namespace operations_research;

namespace tsp_solver_ns {

struct DataModel;
class TSPSolverRefactored;

/**
 * @brief Print the solution to the TSP problem
 * 
 * @param manager Routing index manager
 * @param routing Routing model
 * @param solution Assignment solution
 */
void PrintSolution(const RoutingIndexManager& manager, const RoutingModel& routing, const Assignment& solution);

}  // namespace tsp_solver_ns

/**
 * @brief Data model for the TSP problem
 * 
 * Contains the distance matrix, number of vehicles, and depot information
 */
struct tsp_solver_ns::DataModel {
  /// Distance matrix between nodes
  std::vector<std::vector<int>> distance_matrix;
  
  /// Number of vehicles (typically 1 for TSP)
  int num_vehicles = 1;
  
  /// Depot node index (starting point)
  RoutingIndexManager::NodeIndex depot{ 0 };
};

/**
 * @brief TSP Solver class using Google OR-Tools
 * 
 * This class provides an interface to solve the Traveling Salesman Problem
 * using Google's Operations Research tools.
 */
class tsp_solver_ns::TSPSolverRefactored {
 private:
  /// Problem data
  DataModel data_;
  
  /// Routing index manager
  std::unique_ptr<RoutingIndexManager> manager_;
  
  /// Routing model
  std::unique_ptr<RoutingModel> routing_;
  
  /// Solution assignment
  const Assignment* solution_;

 public:
  /**
   * @brief Construct a new TSPSolver object
   * 
   * @param data Problem data including distance matrix
   */
  explicit TSPSolverRefactored(DataModel data);
  
  /**
   * @brief Destroy the TSPSolver object
   */
  ~TSPSolverRefactored() = default;

  /**
   * @brief Solve the TSP problem
   * 
   * Uses the PATH_CHEAPEST_ARC heuristic to find an initial solution
   */
  void Solve();

  /**
   * @brief Print the solution details to log
   */
  void PrintSolution();

  /**
   * @brief Get the computation time in milliseconds
   * 
   * @return int Computation time in milliseconds
   */
  int getComputationTime();

  /**
   * @brief Get the solution node indices
   * 
   * @param node_index Output vector of node indices in solution order
   * @param has_dummy Whether the problem includes a dummy node that should be removed
   */
  void getSolutionNodeIndex(std::vector<int>& node_index, bool has_dummy);

  /**
   * @brief Get the path length of the solution
   * 
   * @return double Path length in meters (divided by 10.0 from internal units)
   */
  double getPathLength();
};