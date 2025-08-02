#include <gtest/gtest.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <cmath>
#include <limits>
#include "viewpoint/viewpoint_refactored.h"

using namespace viewpoint_ns;

class ViewPointRefactoredTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Setup code if needed
  }

  void TearDown() override {
    // Teardown code if needed
  }
};

TEST_F(ViewPointRefactoredTest, ConstructorTest) {
  // Test ViewPoint constructor with default parameters
  ViewPointRefactored viewpoint(1.0, 2.0, 3.0);
  
  // Verify position
  geometry_msgs::Point position = viewpoint.GetPosition();
  EXPECT_DOUBLE_EQ(position.x, 1.0);
  EXPECT_DOUBLE_EQ(position.y, 2.0);
  EXPECT_DOUBLE_EQ(position.z, 3.0);
  
  // Test construction with geometry_msgs::Point
  geometry_msgs::Point point;
  point.x = 4.0;
  point.y = 5.0;
  point.z = 6.0;
  
  ViewPointRefactored viewpoint2(point);
  
  // Verify position
  geometry_msgs::Point position2 = viewpoint2.GetPosition();
  EXPECT_DOUBLE_EQ(position2.x, 4.0);
  EXPECT_DOUBLE_EQ(position2.y, 5.0);
  EXPECT_DOUBLE_EQ(position2.z, 6.0);
}

TEST_F(ViewPointRefactoredTest, InvalidConstructorTest) {
  // Test construction with NaN values
  testing::internal::CaptureStderr();
  ViewPointRefactored viewpoint_nan(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should produce warning for NaN
  
  // Test construction with infinite values
  testing::internal::CaptureStderr();
  ViewPointRefactored viewpoint_inf(std::numeric_limits<double>::infinity(), 0.0, 0.0);
  output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should produce warning for infinity
}

TEST_F(ViewPointRefactoredTest, PropertyTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test setting and getting collision status
  viewpoint.SetInCollision(true);
  EXPECT_TRUE(viewpoint.InCollision());
  
  viewpoint.SetInCollision(false);
  EXPECT_FALSE(viewpoint.InCollision());
  
  // Test setting and getting visited status
  viewpoint.SetVisited(true);
  EXPECT_TRUE(viewpoint.Visited());
  
  viewpoint.SetVisited(false);
  EXPECT_FALSE(viewpoint.Visited());
  
  // Test setting and getting selected status
  viewpoint.SetSelected(true);
  EXPECT_TRUE(viewpoint.Selected());
  
  viewpoint.SetSelected(false);
  EXPECT_FALSE(viewpoint.Selected());
  
  // Test setting and getting candidate status
  viewpoint.SetCandidate(true);
  EXPECT_TRUE(viewpoint.IsCandidate());
  
  viewpoint.SetCandidate(false);
  EXPECT_FALSE(viewpoint.IsCandidate());
  
  // Test setting and getting connected status
  viewpoint.SetConnected(true);
  EXPECT_TRUE(viewpoint.Connected());
  
  viewpoint.SetConnected(false);
  EXPECT_FALSE(viewpoint.Connected());
  
  // Test setting and getting line of sight status
  viewpoint.SetInLineOfSight(true);
  EXPECT_TRUE(viewpoint.InLineOfSight());
  
  viewpoint.SetInLineOfSight(false);
  EXPECT_FALSE(viewpoint.InLineOfSight());
  
  // Test setting and getting current frame line of sight status
  viewpoint.SetInCurrentFrameLineOfSight(true);
  EXPECT_TRUE(viewpoint.InCurrentFrameLineOfSight());
  
  viewpoint.SetInCurrentFrameLineOfSight(false);
  EXPECT_FALSE(viewpoint.InCurrentFrameLineOfSight());
}

TEST_F(ViewPointRefactoredTest, TerrainPropertyTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test setting and getting terrain height status
  viewpoint.SetHasTerrainHeight(true);
  EXPECT_TRUE(viewpoint.HasTerrainHeight());
  
  viewpoint.SetHasTerrainHeight(false);
  EXPECT_FALSE(viewpoint.HasTerrainHeight());
  
  // Test setting and getting terrain height value
  viewpoint.SetTerrainHeight(10.5);
  EXPECT_DOUBLE_EQ(viewpoint.GetTerrainHeight(), 10.5);
  
  // Test setting and getting terrain neighbor status
  viewpoint.SetHasTerrainNeighbor(true);
  EXPECT_TRUE(viewpoint.HasTerrainNeighbor());
  
  viewpoint.SetHasTerrainNeighbor(false);
  EXPECT_FALSE(viewpoint.HasTerrainNeighbor());
}

TEST_F(ViewPointRefactoredTest, AssociatedCellTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test setting and getting associated cell index
  viewpoint.SetCellInd(42);
  EXPECT_EQ(viewpoint.GetCellInd(), 42);
  EXPECT_EQ(viewpoint.GetCellIndex(), 42);
  
  // Test setting negative cell index (should produce warning)
  testing::internal::CaptureStderr();
  viewpoint.SetCellInd(-5);
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should produce warning for negative index
  EXPECT_EQ(viewpoint.GetCellInd(), -5);
}

TEST_F(ViewPointRefactoredTest, PositionTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test setting and getting position components
  EXPECT_DOUBLE_EQ(viewpoint.GetX(), 0.0);
  EXPECT_DOUBLE_EQ(viewpoint.GetY(), 0.0);
  EXPECT_DOUBLE_EQ(viewpoint.GetHeight(), 0.0);
  
  // Test setting height
  viewpoint.SetHeight(5.0);
  EXPECT_DOUBLE_EQ(viewpoint.GetHeight(), 5.0);
  
  // Test setting invalid height (should produce error)
  testing::internal::CaptureStderr();
  viewpoint.SetHeight(std::numeric_limits<double>::quiet_NaN());
  std::string output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should produce error for NaN
  
  // Test setting position
  geometry_msgs::Point new_position;
  new_position.x = 10.0;
  new_position.y = 20.0;
  new_position.z = 30.0;
  
  viewpoint.SetPosition(new_position);
  EXPECT_DOUBLE_EQ(viewpoint.GetX(), 10.0);
  EXPECT_DOUBLE_EQ(viewpoint.GetY(), 20.0);
  EXPECT_DOUBLE_EQ(viewpoint.GetHeight(), 30.0);
  
  geometry_msgs::Point retrieved_position = viewpoint.GetPosition();
  EXPECT_DOUBLE_EQ(retrieved_position.x, 10.0);
  EXPECT_DOUBLE_EQ(retrieved_position.y, 20.0);
  EXPECT_DOUBLE_EQ(retrieved_position.z, 30.0);
  
  // Test setting invalid position (should produce error)
  testing::internal::CaptureStderr();
  geometry_msgs::Point invalid_position;
  invalid_position.x = std::numeric_limits<double>::quiet_NaN();
  invalid_position.y = 0.0;
  invalid_position.z = 0.0;
  viewpoint.SetPosition(invalid_position);
  output = testing::internal::GetCapturedStderr();
  EXPECT_FALSE(output.empty()); // Should produce error for NaN
}

TEST_F(ViewPointRefactoredTest, CoverageTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test initial covered point count is zero
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
  EXPECT_TRUE(viewpoint.GetCoveredPointList().empty());
  
  // Test initial covered frontier point count is zero
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 0);
  EXPECT_TRUE(viewpoint.GetCoveredFrontierPointList().empty());
  
  // Test adding covered points
  viewpoint.AddCoveredPoint(1);
  viewpoint.AddCoveredPoint(2);
  viewpoint.AddCoveredPoint(3);
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 3);
  EXPECT_EQ(viewpoint.GetCoveredPointList().size(), 3);
  
  // Test adding covered frontier points
  viewpoint.AddCoveredFrontierPoint(10);
  viewpoint.AddCoveredFrontierPoint(20);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 2);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointList().size(), 2);
  
  // Test resetting coverage
  viewpoint.ResetCoverage();
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 0);
  
  // Test resetting individual lists
  viewpoint.AddCoveredPoint(1);
  viewpoint.AddCoveredPoint(2);
  viewpoint.AddCoveredFrontierPoint(10);
  viewpoint.AddCoveredFrontierPoint(20);
  
  viewpoint.ResetCoveredPointList();
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 2);
  
  viewpoint.ResetCoveredFrontierPointList();
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 0);
}

TEST_F(ViewPointRefactoredTest, CollisionFrameTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test initial collision frame count
  EXPECT_EQ(viewpoint.GetCollisionFrameCount(), 0);
  
  // Test adding collision frames
  viewpoint.AddCollisionFrame();
  EXPECT_EQ(viewpoint.GetCollisionFrameCount(), 1);
  
  viewpoint.AddCollisionFrame();
  viewpoint.AddCollisionFrame();
  EXPECT_EQ(viewpoint.GetCollisionFrameCount(), 3);
  
  // Test resetting collision frame count
  viewpoint.ResetCollisionFrameCount();
  EXPECT_EQ(viewpoint.GetCollisionFrameCount(), 0);
}

TEST_F(ViewPointRefactoredTest, ExploringCellTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Test setting and getting exploring cell status
  viewpoint.SetInExploringCell(true);
  EXPECT_TRUE(viewpoint.InExploringCell());
  
  viewpoint.SetInExploringCell(false);
  EXPECT_FALSE(viewpoint.InExploringCell());
}

TEST_F(ViewPointRefactoredTest, ResetTest) {
  ViewPointRefactored viewpoint(0.0, 0.0, 0.0);
  
  // Set various properties
  viewpoint.SetInCollision(true);
  viewpoint.SetVisited(true);
  viewpoint.SetSelected(true);
  viewpoint.SetCandidate(true);
  viewpoint.SetConnected(true);
  viewpoint.SetInLineOfSight(true);
  viewpoint.SetInCurrentFrameLineOfSight(true);
  viewpoint.SetHasTerrainHeight(true);
  viewpoint.SetHasTerrainNeighbor(true);
  viewpoint.SetInExploringCell(true);
  viewpoint.SetCellInd(10);
  viewpoint.SetTerrainHeight(5.0);
  viewpoint.AddCoveredPoint(1);
  viewpoint.AddCoveredFrontierPoint(2);
  viewpoint.AddCollisionFrame();
  
  // Verify properties are set
  EXPECT_TRUE(viewpoint.InCollision());
  EXPECT_TRUE(viewpoint.Visited());
  EXPECT_TRUE(viewpoint.Selected());
  EXPECT_TRUE(viewpoint.IsCandidate());
  EXPECT_TRUE(viewpoint.Connected());
  EXPECT_TRUE(viewpoint.InLineOfSight());
  EXPECT_TRUE(viewpoint.InCurrentFrameLineOfSight());
  EXPECT_TRUE(viewpoint.HasTerrainHeight());
  EXPECT_TRUE(viewpoint.HasTerrainNeighbor());
  EXPECT_TRUE(viewpoint.InExploringCell());
  EXPECT_EQ(viewpoint.GetCellInd(), 10);
  EXPECT_DOUBLE_EQ(viewpoint.GetTerrainHeight(), 5.0);
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 1);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 1);
  EXPECT_EQ(viewpoint.GetCollisionFrameCount(), 1);
  
  // Reset viewpoint
  viewpoint.Reset();
  
  // Verify properties are reset
  EXPECT_FALSE(viewpoint.InCollision());
  EXPECT_FALSE(viewpoint.Visited());
  EXPECT_FALSE(viewpoint.Selected());
  EXPECT_FALSE(viewpoint.IsCandidate());
  EXPECT_FALSE(viewpoint.Connected());
  EXPECT_FALSE(viewpoint.InLineOfSight());
  EXPECT_FALSE(viewpoint.InCurrentFrameLineOfSight());
  EXPECT_FALSE(viewpoint.HasTerrainHeight());
  EXPECT_FALSE(viewpoint.HasTerrainNeighbor());
  EXPECT_FALSE(viewpoint.InExploringCell());
  EXPECT_EQ(viewpoint.GetCellInd(), -1);
  EXPECT_DOUBLE_EQ(viewpoint.GetTerrainHeight(), 0.0);
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
  EXPECT_EQ(viewpoint.GetCoveredFrontierPointNum(), 0);
  EXPECT_EQ(viewpoint.GetCollisionFrameCount(), 0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}