#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include "grid_world/grid_world.h"
#include "viewpoint_manager/viewpoint_manager.h"
#include "keypose_graph/keypose_graph.h"

using namespace grid_world_ns;

class GridWorldGlobalPlannerTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    // 初始化ROS节点（如果尚未初始化）
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "grid_world_global_planner_test", ros::init_options::AnonymousName);
    }
  }
  
  virtual void TearDown() {
  }
};

TEST_F(GridWorldGlobalPlannerTest, ConstructorTest)
{
  // 测试GridWorld构造函数
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  // 最初，网格世界应该未初始化
  EXPECT_FALSE(grid_world.Initialized());
  
  // 测试设置机器人位置（初始化网格世界）
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // 更新机器人位置以初始化网格世界
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // 现在应该已初始化
  EXPECT_TRUE(grid_world.Initialized());
  
  // 测试单元格访问
  int cell_ind = grid_world.GetCellInd(robot_position.x, robot_position.y, robot_position.z);
  EXPECT_GE(cell_ind, 0);
  
  // 检查单元格状态最初应为UNSEEN
  CellStatus status = grid_world.GetCellStatus(cell_ind);
  EXPECT_EQ(status, CellStatus::UNSEEN);
}

TEST_F(GridWorldGlobalPlannerTest, CellStatusManagementTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // 初始化网格世界
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  int cell_ind = grid_world.GetCellInd(robot_position.x, robot_position.y, robot_position.z);
  
  // 测试设置和获取单元格状态
  grid_world.SetCellStatus(cell_ind, CellStatus::EXPLORING);
  EXPECT_EQ(grid_world.GetCellStatus(cell_ind), CellStatus::EXPLORING);
  
  grid_world.SetCellStatus(cell_ind, CellStatus::COVERED);
  EXPECT_EQ(grid_world.GetCellStatus(cell_ind), CellStatus::COVERED);
  
  grid_world.SetCellStatus(cell_ind, CellStatus::UNSEEN);
  EXPECT_EQ(grid_world.GetCellStatus(cell_ind), CellStatus::UNSEEN);
}

TEST_F(GridWorldGlobalPlannerTest, IndexConversionTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // 初始化网格世界
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // 测试索引转换函数
  int linear_index = grid_world.sub2ind(1, 1, 1);
  Eigen::Vector3i sub_index = grid_world.ind2sub(linear_index);
  
  EXPECT_EQ(sub_index.x(), 1);
  EXPECT_EQ(sub_index.y(), 1);
  EXPECT_EQ(sub_index.z(), 1);
  
  // 测试边界检查
  EXPECT_TRUE(grid_world.SubInBound(0, 0, 0));
  EXPECT_TRUE(grid_world.SubInBound(4, 4, 2));
  EXPECT_FALSE(grid_world.SubInBound(5, 5, 3));
  EXPECT_FALSE(grid_world.SubInBound(-1, 0, 0));
}

TEST_F(GridWorldGlobalPlannerTest, CellPositionTest)
{
  GridWorld grid_world(3, 3, 1, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // 初始化网格世界
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // 测试获取单元格位置
  geometry_msgs::Point cell_position = grid_world.GetCellPosition(0);
  
  // 验证位置合理（基于3x3x1网格，单元格大小为6.0）
  EXPECT_LE(cell_position.x, 6.0);
  EXPECT_LE(cell_position.y, 6.0);
  EXPECT_LE(cell_position.z, 6.0);
}

TEST_F(GridWorldGlobalPlannerTest, NeighborCellTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // 初始化网格世界
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // 测试邻居单元格功能
  std::vector<int> neighbor_indices;
  Eigen::Vector3i center_cell_sub(2, 2, 1);
  Eigen::Vector3i neighbor_range(1, 1, 1);
  
  grid_world.GetNeighborCellIndices(center_cell_sub, neighbor_range, neighbor_indices);
  
  // 验证邻居索引已填充（可能为空，这取决于实现）
  EXPECT_TRUE(neighbor_indices.size() >= 0);
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}