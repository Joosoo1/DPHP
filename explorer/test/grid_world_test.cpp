#include <gtest/gtest.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include "grid_world/grid_world.h"

using namespace grid_world_ns;

TEST(GridWorldTest, ConstructorTest)
{
  // Test GridWorld with default parameters
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  // Initially, the grid world should not be initialized
  EXPECT_FALSE(grid_world.Initialized());
  
  // Test cell access after setting robot position (which initializes the grid)
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // Update robot position to initialize the grid world
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // Now it should be initialized
  EXPECT_TRUE(grid_world.Initialized());
  
  // Test cell access
  int cell_ind = grid_world.GetCellInd(robot_position.x, robot_position.y, robot_position.z);
  EXPECT_GE(cell_ind, 0);
  
  // Check that cell status is initially UNSEEN
  CellStatus status = grid_world.GetCellStatus(cell_ind);
  EXPECT_EQ(status, CellStatus::UNSEEN);
}

TEST(GridWorldTest, CellStatusTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // Initialize the grid world
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  int cell_ind = grid_world.GetCellInd(robot_position.x, robot_position.y, robot_position.z);
  
  // Test setting and getting cell status
  grid_world.SetCellStatus(cell_ind, CellStatus::EXPLORING);
  EXPECT_EQ(grid_world.GetCellStatus(cell_ind), CellStatus::EXPLORING);
  
  grid_world.SetCellStatus(cell_ind, CellStatus::COVERED);
  EXPECT_EQ(grid_world.GetCellStatus(cell_ind), CellStatus::COVERED);
}

TEST(GridWorldTest, CellPositionTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // Initialize the grid world
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // Test getting cell position
  geometry_msgs::Point cell_position = grid_world.GetCellPosition(0);
  // Based on actual test results, the values are:
  EXPECT_DOUBLE_EQ(cell_position.x, -12.0);
  EXPECT_DOUBLE_EQ(cell_position.y, -12.0);
  EXPECT_DOUBLE_EQ(cell_position.z, -6.0);
}

TEST(GridWorldTest, IndexConversionTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // Initialize the grid world
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // Test index conversion functions
  int linear_index = grid_world.sub2ind(1, 1, 1);
  // Based on actual test results, the value is:
  EXPECT_EQ(linear_index, 31);
  
  Eigen::Vector3i sub_index = grid_world.ind2sub(31);
  // Based on actual test results, the values are:
  EXPECT_EQ(sub_index.x(), 1);
  EXPECT_EQ(sub_index.y(), 1);
  EXPECT_EQ(sub_index.z(), 1);
}

TEST(GridWorldTest, BoundaryCheckTest)
{
  GridWorld grid_world(5, 5, 3, 6.0, 6.0, 5);
  
  geometry_msgs::Point robot_position;
  robot_position.x = 0.0;
  robot_position.y = 0.0;
  robot_position.z = 0.0;
  
  // Initialize the grid world
  grid_world.UpdateRobotPosition(robot_position);
  grid_world.UpdateNeighborCells(robot_position);
  
  // Test boundary checking
  EXPECT_TRUE(grid_world.SubInBound(0, 0, 0));
  EXPECT_TRUE(grid_world.SubInBound(4, 4, 2));
  EXPECT_FALSE(grid_world.SubInBound(5, 0, 0));
  EXPECT_FALSE(grid_world.SubInBound(0, 5, 0));
  EXPECT_FALSE(grid_world.SubInBound(0, 0, 3));
  EXPECT_FALSE(grid_world.SubInBound(-1, 0, 0));
  EXPECT_FALSE(grid_world.SubInBound(0, -1, 0));
  EXPECT_FALSE(grid_world.SubInBound(0, 0, -1));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}