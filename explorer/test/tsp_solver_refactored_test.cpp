#include <gtest/gtest.h>
#include <vector>
#include "tsp_solver/tsp_solver_refactored.h"

using namespace tsp_solver_ns;

class TSPSolverRefactoredTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup code if needed
  }

  void TearDown() override {
    // Teardown code if needed
  }
};

TEST_F(TSPSolverRefactoredTest, ConstructorTest) {
  // Test TSPSolver constructor
  DataModel data;
  data.distance_matrix = {
    { 0, 1, 1, 1 },
    { 1, 0, 1, 1 },
    { 1, 1, 0, 1 },
    { 1, 1, 1, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  
  // Just verify that the solver was constructed without crashing
  SUCCEED();
}

TEST_F(TSPSolverRefactoredTest, EmptyMatrixTest) {
  // Test TSPSolver constructor with empty matrix
  DataModel data;
  data.distance_matrix = {};
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  
  // Should not crash even with empty matrix
  SUCCEED();
}

TEST_F(TSPSolverRefactoredTest, SimpleSolutionTest) {
  // Create a simple TSP problem
  DataModel data;
  data.distance_matrix = {
    { 0, 1, 1, 1 },
    { 1, 0, 1, 1 },
    { 1, 1, 0, 1 },
    { 1, 1, 1, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  
  // Solve the TSP
  solver.Solve();
  
  // Get the solution
  std::vector<int> node_indices;
  solver.getSolutionNodeIndex(node_indices, false);
  
  // Check that we have a solution
  EXPECT_FALSE(node_indices.empty());
}

TEST_F(TSPSolverRefactoredTest, PathLengthTest) {
  DataModel data;
  data.distance_matrix = {
    { 0, 10, 15, 20 },
    { 10, 0, 35, 25 },
    { 15, 35, 0, 30 },
    { 20, 25, 30, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  solver.Solve();
  
  // Check that we can get a path length
  double path_length = solver.getPathLength();
  EXPECT_GE(path_length, 0.0);
}

TEST_F(TSPSolverRefactoredTest, ComputationTimeTest) {
  DataModel data;
  data.distance_matrix = {
    { 0, 10, 15, 20 },
    { 10, 0, 35, 25 },
    { 15, 35, 0, 30 },
    { 20, 25, 30, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  solver.Solve();
  
  // Check that we can get computation time
  int computation_time = solver.getComputationTime();
  EXPECT_GE(computation_time, 0);
}

TEST_F(TSPSolverRefactoredTest, DummyNodeTest) {
  DataModel data;
  data.distance_matrix = {
    { 0, 10, 15, 20 },
    { 10, 0, 35, 25 },
    { 15, 35, 0, 30 },
    { 20, 25, 30, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  solver.Solve();
  
  // Get the solution with dummy node processing
  std::vector<int> node_indices;
  solver.getSolutionNodeIndex(node_indices, true);
  
  // Should not crash
  SUCCEED();
}

TEST_F(TSPSolverRefactoredTest, InvalidSolutionTest) {
  // Test behavior when no solution is available
  DataModel data;
  data.distance_matrix = {
    { 0, 1000000, 1000000 },
    { 1000000, 0, 1000000 },
    { 1000000, 1000000, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  // Note: Not calling Solve() to test behavior with no solution
  
  // Get the solution without solving first
  std::vector<int> node_indices;
  solver.getSolutionNodeIndex(node_indices, false);
  
  // Should handle gracefully
  SUCCEED();
  
  // Check path length without solving
  double path_length = solver.getPathLength();
  EXPECT_DOUBLE_EQ(path_length, -1.0); // Should return -1 to indicate error
  
  // Check computation time without solving
  int computation_time = solver.getComputationTime();
  EXPECT_EQ(computation_time, -1); // Should return -1 to indicate error
}

TEST_F(TSPSolverRefactoredTest, PrintSolutionTest) {
  DataModel data;
  data.distance_matrix = {
    { 0, 10, 15 },
    { 10, 0, 20 },
    { 15, 20, 0 }
  };
  data.num_vehicles = 1;
  data.depot = 0;
  
  TSPSolverRefactored solver(data);
  solver.Solve();
  
  // Should not crash when printing solution
  testing::internal::CaptureStderr();
  solver.PrintSolution();
  std::string output = testing::internal::GetCapturedStderr();
  // Should produce some output
  SUCCEED();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}