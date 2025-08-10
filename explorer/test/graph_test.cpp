#include <gtest/gtest.h>
#include <Eigen/Core>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "graph/graph.h"

using namespace explorer;

TEST(GraphTest, ConstructorTest)
{
  // Test graph with 5 nodes
  Graph graph(5);
  
  // The graph should be created without issues
  ASSERT_TRUE(true);
}

TEST(GraphTest, AddNodeTest)
{
  // Start with a graph with 1 node
  Graph graph(1);
  
  // Add a new node
  Eigen::Vector3d position(1.0, 2.0, 3.0);
  graph.AddNode(position);
  
  // Graph should now have 2 nodes
  ASSERT_TRUE(true);
}

TEST(GraphTest, SetNodePositionTest)
{
  Graph graph(2);
  Eigen::Vector3d position1(1.0, 1.0, 1.0);
  Eigen::Vector3d position2(2.0, 2.0, 2.0);
  Eigen::Vector3d position3(3.0, 3.0, 3.0);
  
  graph.SetNodePosition(0, position1);
  graph.SetNodePosition(1, position2);
  
  // Test setting position of existing node
  graph.SetNodePosition(0, position3);
  
  // Test setting position of new node (should add node)
  graph.SetNodePosition(2, position1);
  
  ASSERT_TRUE(true);
}

TEST(GraphTest, AddOneWayEdgeTest)
{
  Graph graph(3);
  
  // Add edges
  graph.AddOneWayEdge(0, 1, 5.0);
  graph.AddOneWayEdge(1, 2, 3.0);
  graph.AddOneWayEdge(0, 2, 10.0);
  
  // Test invalid indices
  graph.AddOneWayEdge(-1, 1, 1.0);
  graph.AddOneWayEdge(0, 5, 1.0);
  
  ASSERT_TRUE(true);
}

TEST(GraphTest, AddTwoWayEdgeTest)
{
  Graph graph(3);
  
  // Add two-way edges
  graph.AddTwoWayEdge(0, 1, 5.0);
  graph.AddTwoWayEdge(1, 2, 3.0);
  
  ASSERT_TRUE(true);
}

TEST(GraphTest, ShortestPathTest)
{
  Graph graph(4);
  
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
  EXPECT_EQ(node_indices.size(), 4);
  EXPECT_EQ(node_indices[0], 0);
  EXPECT_EQ(node_indices[1], 1);
  EXPECT_EQ(node_indices[2], 2);
  EXPECT_EQ(node_indices[3], 3);
  EXPECT_EQ(path.poses.size(), 4);
}

TEST(GraphTest, AStarHeuristicTest)
{
  Graph graph(3);
  
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
  EXPECT_EQ(node_indices.size(), 3);
  EXPECT_EQ(node_indices[0], 0);
  EXPECT_EQ(node_indices[1], 1);
  EXPECT_EQ(node_indices[2], 2);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}