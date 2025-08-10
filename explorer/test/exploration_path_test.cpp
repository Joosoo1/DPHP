#include <gtest/gtest.h>
#include <Eigen/Core>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include "exploration_path/exploration_path.h"

using namespace exploration_path_ns;

TEST(NodeTest, ConstructorTest)
{
  // Test default constructor
  Node node1;
  EXPECT_EQ(node1.type_, NodeType::LOCAL_VIA_POINT); // 默认构造函数设置为LOCAL_VIA_POINT而不是ROBOT
  EXPECT_EQ(node1.local_viewpoint_ind_, -1);
  EXPECT_EQ(node1.keypose_graph_node_ind_, -1);
  EXPECT_EQ(node1.global_subspace_index_, -1);
  EXPECT_FALSE(node1.nonstop_);

  // Test constructor with position
  Eigen::Vector3d position(1.0, 2.0, 3.0);
  Node node2(position);
  EXPECT_DOUBLE_EQ(node2.position_.x(), 1.0);
  EXPECT_DOUBLE_EQ(node2.position_.y(), 2.0);
  EXPECT_DOUBLE_EQ(node2.position_.z(), 3.0);
  EXPECT_EQ(node2.type_, NodeType::LOCAL_VIA_POINT); // 从Node(position)构造函数调用Node()，所以type_也是LOCAL_VIA_POINT

  // Test constructor with point and type
  geometry_msgs::Point point;
  point.x = 4.0;
  point.y = 5.0;
  point.z = 6.0;
  Node node3(point, NodeType::GLOBAL_VIEWPOINT);
  EXPECT_DOUBLE_EQ(node3.position_.x(), 4.0);
  EXPECT_DOUBLE_EQ(node3.position_.y(), 5.0);
  EXPECT_DOUBLE_EQ(node3.position_.z(), 6.0);
  EXPECT_EQ(node3.type_, NodeType::GLOBAL_VIEWPOINT);
}

TEST(NodeTest, IsLocalTest)
{
  Node node1(Eigen::Vector3d(0, 0, 0));
  node1.type_ = NodeType::LOCAL_VIEWPOINT;
  EXPECT_TRUE(node1.IsLocal());

  node1.type_ = NodeType::GLOBAL_VIEWPOINT;
  EXPECT_FALSE(node1.IsLocal());

  node1.type_ = NodeType::LOCAL_PATH_START;
  EXPECT_TRUE(node1.IsLocal());
}

TEST(NodeTest, ComparisonTest)
{
  Eigen::Vector3d pos1(1.0, 2.0, 3.0);
  Eigen::Vector3d pos2(1.0, 2.0, 3.0);
  Eigen::Vector3d pos3(4.0, 5.0, 6.0);

  Node node1(pos1);
  Node node2(pos2);
  Node node3(pos3);

  EXPECT_TRUE(node1 == node2);
  EXPECT_FALSE(node1 != node2);
  EXPECT_FALSE(node1 == node3);
  EXPECT_TRUE(node1 != node3);
}

TEST(ExplorationPathTest, BasicOperationsTest)
{
  ExplorationPath path;

  // Test initial state
  EXPECT_EQ(path.GetNodeNum(), 0);
  EXPECT_DOUBLE_EQ(path.GetLength(), 0.0);

  // Test Append node
  Eigen::Vector3d pos1(0.0, 0.0, 0.0);
  Node node1(pos1);
  path.Append(node1);
  EXPECT_EQ(path.GetNodeNum(), 1);

  Eigen::Vector3d pos2(1.0, 0.0, 0.0);
  Node node2(pos2);
  path.Append(node2);
  EXPECT_EQ(path.GetNodeNum(), 2);

  // Test path length
  EXPECT_DOUBLE_EQ(path.GetLength(), 1.0);

  // Test Reset
  path.Reset();
  EXPECT_EQ(path.GetNodeNum(), 0);
  EXPECT_DOUBLE_EQ(path.GetLength(), 0.0);
}

TEST(ExplorationPathTest, AppendPathTest)
{
  // Create first path
  ExplorationPath path1;
  Node node1(Eigen::Vector3d(0.0, 0.0, 0.0));
  Node node2(Eigen::Vector3d(1.0, 0.0, 0.0));
  path1.Append(node1);
  path1.Append(node2);

  // Create second path
  ExplorationPath path2;
  Node node3(Eigen::Vector3d(2.0, 0.0, 0.0));
  Node node4(Eigen::Vector3d(3.0, 0.0, 0.0));
  path2.Append(node3);
  path2.Append(node4);

  // Append path2 to path1
  path1.Append(path2);
  EXPECT_EQ(path1.GetNodeNum(), 4);

  // Check path length
  EXPECT_DOUBLE_EQ(path1.GetLength(), 3.0);
}

TEST(ExplorationPathTest, ReverseTest)
{
  ExplorationPath path;
  Node node1(Eigen::Vector3d(0.0, 0.0, 0.0));
  Node node2(Eigen::Vector3d(1.0, 0.0, 0.0));
  Node node3(Eigen::Vector3d(2.0, 0.0, 0.0));

  path.Append(node1);
  path.Append(node2);
  path.Append(node3);

  // Check initial order
  EXPECT_DOUBLE_EQ(path.nodes_[0].position_.x(), 0.0);
  EXPECT_DOUBLE_EQ(path.nodes_[2].position_.x(), 2.0);

  // Reverse path
  path.Reverse();

  // Check reversed order
  EXPECT_DOUBLE_EQ(path.nodes_[0].position_.x(), 2.0);
  EXPECT_DOUBLE_EQ(path.nodes_[2].position_.x(), 0.0);
}

TEST(ExplorationPathTest, GetPathTest)
{
  ExplorationPath path;
  Node node1(Eigen::Vector3d(0.0, 0.0, 0.0));
  Node node2(Eigen::Vector3d(1.0, 1.0, 1.0));

  path.Append(node1);
  path.Append(node2);

  nav_msgs::Path ros_path = path.GetPath();
  EXPECT_EQ(ros_path.poses.size(), 2);

  EXPECT_DOUBLE_EQ(ros_path.poses[0].pose.position.x, 0.0);
  EXPECT_DOUBLE_EQ(ros_path.poses[0].pose.position.y, 0.0);
  EXPECT_DOUBLE_EQ(ros_path.poses[0].pose.position.z, 0.0);

  EXPECT_DOUBLE_EQ(ros_path.poses[1].pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(ros_path.poses[1].pose.position.y, 1.0);
  EXPECT_DOUBLE_EQ(ros_path.poses[1].pose.position.z, 1.0);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
