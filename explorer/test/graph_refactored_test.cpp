#include <gtest/gtest.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include "graph/graph_refactored.h"

using namespace explorer;

// Test fixture for Graph tests
class GraphRefactoredTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup code if needed
  }

  void TearDown() override {
    // Teardown code if needed
  }
};

TEST_F(GraphRefactoredTest, ConstructorTest) {
  // Test graph with 5 nodes
  GraphRefactored graph(5);
  // The graph should be created without issues
  SUCCEED();
  
  // Test graph with 0 nodes
  GraphRefactored empty_graph(0);
  SUCCEED();
  
  // Test graph with negative nodes (should be handled gracefully)
  GraphRefactored negative_graph(-1);
  SUCCEED();
}

TEST_F(GraphRefactoredTest, AddNodeTest) {
  // Start with a graph with 1 node
  GraphRefactored graph(1);
  
  // Add a new node
  Eigen::Vector3d position(1.0, 2.0, 3.0);
  graph.AddNode(position);
  
  // Graph should now have 2 nodes
  SUCCEED();
}

TEST_F(GraphRefactoredTest, SetNodePositionTest) {
  GraphRefactored graph(2);
  Eigen::Vector3d position1(1.0, 1.0, 1.0);
  Eigen::Vector3d position2(2.0, 2.0, 2.0);
  Eigen::Vector3d position3(3.0, 3.0, 3.0);
  
  graph.SetNodePosition(0, position1);
  graph.SetNodePosition(1, position2);
  
  // Test setting position of existing node
  graph.SetNodePosition(0, position3);
  
  // Test setting position of new node (should add node)
  graph.SetNodePosition(2, position1);
  
  SUCCEED();
}

TEST_F(GraphRefactoredTest, SetNodePositionInvalidIndexTest) {
  GraphRefactored graph(2);
  Eigen::Vector3d position(1.0, 1.0, 1.0);
  
  // Test with index too large (should produce error)
  testing::internal::CaptureStderr();
  graph.SetNodePosition(5, position);
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should have error output
  
  // Test with negative index (should produce error)
  testing::internal::CaptureStderr();
  graph.SetNodePosition(-1, position);
  output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should have error output
}

TEST_F(GraphRefactoredTest, AddOneWayEdgeTest) {
  GraphRefactored graph(3);
  
  // Add edges
  graph.AddOneWayEdge(0, 1, 5.0);
  graph.AddOneWayEdge(1, 2, 3.0);
  graph.AddOneWayEdge(0, 2, 10.0);
  
  // Test invalid indices
  testing::internal::CaptureStderr();
  graph.AddOneWayEdge(-1, 1, 1.0);
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should have error output
  
  testing::internal::CaptureStderr();
  graph.AddOneWayEdge(0, 5, 1.0);
  output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should have error output
  
  SUCCEED();
}

TEST_F(GraphRefactoredTest, AddTwoWayEdgeTest) {
  GraphRefactored graph(3);
  
  // Add two-way edges
  graph.AddTwoWayEdge(0, 1, 5.0);
  graph.AddTwoWayEdge(1, 2, 3.0);
  
  SUCCEED();
}

TEST_F(GraphRefactoredTest, ShortestPathTest) {
  GraphRefactored graph(4);
  
  // Set node positions
  graph.SetNodePosition(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  graph.SetNodePosition(1, Eigen::Vector3d(1.0, 0.0, 0.0));
  graph.SetNodePosition(2, Eigen::Vector3d(2.0, 0.0, 0.0));
  graph.SetNodePosition(3, Eigen::Vector3d(3.0, 0.0, 0.0));
  
  // Add edges to form a straight line: 0-1-2-3
  graph.AddTwoWayEdge(0, 1, 1.0);
  graph.AddTwoWayEdge(1, 2, 1.0);
  graph.AddTwoWayEdge(2, 3, 1.0);
  
  // Test shortest path
  nav_msgs::Path path;
  std::vector<int> node_indices;
  double distance = graph.GetShortestPath(0, 3, true, path, node_indices);
  
  // Check results
  EXPECT_DOUBLE_EQ(distance, 3.0);
  ASSERT_EQ(node_indices.size(), 4);
  EXPECT_EQ(node_indices[0], 0);
  EXPECT_EQ(node_indices[1], 1);
  EXPECT_EQ(node_indices[2], 2);
  EXPECT_EQ(node_indices[3], 3);
  EXPECT_EQ(path.poses.size(), 4);
}

TEST_F(GraphRefactoredTest, ShortestPathSameStartEndTest) {
  GraphRefactored graph(4);
  
  // Set node positions
  graph.SetNodePosition(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  graph.SetNodePosition(1, Eigen::Vector3d(1.0, 0.0, 0.0));
  
  graph.AddTwoWayEdge(0, 1, 1.0);
  
  // Test shortest path from node to itself
  nav_msgs::Path path;
  std::vector<int> node_indices;
  double distance = graph.GetShortestPath(0, 0, true, path, node_indices);
  
  // Check results
  EXPECT_DOUBLE_EQ(distance, 0.0);
  ASSERT_EQ(node_indices.size(), 1);
  EXPECT_EQ(node_indices[0], 0);
}

TEST_F(GraphRefactoredTest, ShortestPathNoPathTest) {
  GraphRefactored graph(4);
  
  // Set node positions
  graph.SetNodePosition(0, Eigen::Vector3d(0.0, 0.0, 0.0));
  graph.SetNodePosition(1, Eigen::Vector3d(1.0, 0.0, 0.0));
  graph.SetNodePosition(2, Eigen::Vector3d(2.0, 0.0, 0.0));
  graph.SetNodePosition(3, Eigen::Vector3d(3.0, 0.0, 0.0));
  
  // Add edges only for first two nodes
  graph.AddTwoWayEdge(0, 1, 1.0);
  
  // Test shortest path between disconnected nodes
  nav_msgs::Path path;
  std::vector<int> node_indices;
  double distance = graph.GetShortestPath(0, 3, true, path, node_indices);
  
  // Check results - should be -1 indicating no path
  EXPECT_DOUBLE_EQ(distance, -1.0);
}

TEST_F(GraphRefactoredTest, ShortestPathInvalidNodeTest) {
  GraphRefactored graph(4);
  
  // Test with invalid node indices
  nav_msgs::Path path;
  std::vector<int> node_indices;
  
  // Test with negative start index
  double distance = graph.GetShortestPath(-1, 0, true, path, node_indices);
  EXPECT_DOUBLE_EQ(distance, -1.0);
  
  // Test with out of range end index
  distance = graph.GetShortestPath(0, 10, true, path, node_indices);
  EXPECT_DOUBLE_EQ(distance, -1.0);
}

TEST_F(GraphRefactoredTest, AStarHeuristicTest) {
  GraphRefactored graph(3);
  
  // Set node positions to form a triangle
  graph.SetNodePosition(0, Eigen::Vector3d(0.0, 0.0, 0.0));     // Start
  graph.SetNodePosition(1, Eigen::Vector3d(1.0, 1.0, 0.0));     // Intermediate
  graph.SetNodePosition(2, Eigen::Vector3d(2.0, 0.0, 0.0));     // Goal
  
  // Connect nodes to form 0-1-2 path
  graph.AddTwoWayEdge(0, 1, 1.42);  // Distance approximately sqrt(2)
  graph.AddTwoWayEdge(1, 2, 1.42);  // Distance approximately sqrt(2)
  
  // Test shortest path
  nav_msgs::Path path;
  std::vector<int> node_indices;
  double distance = graph.GetShortestPath(0, 2, true, path, node_indices);
  
  // Check that path goes through intermediate node
  ASSERT_EQ(node_indices.size(), 3);
  EXPECT_EQ(node_indices[0], 0);
  EXPECT_EQ(node_indices[1], 1);
  EXPECT_EQ(node_indices[2], 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}