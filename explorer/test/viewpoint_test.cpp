#include <gtest/gtest.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include "viewpoint/viewpoint.h"

using namespace viewpoint_ns;

TEST(ViewPointTest, ConstructorTest)
{
  // Test ViewPoint constructor with default parameters
  ViewPoint viewpoint(1.0, 2.0, 3.0);
  
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
  
  ViewPoint viewpoint2(point);
  
  // Verify position
  geometry_msgs::Point position2 = viewpoint2.GetPosition();
  EXPECT_DOUBLE_EQ(position2.x, 4.0);
  EXPECT_DOUBLE_EQ(position2.y, 5.0);
  EXPECT_DOUBLE_EQ(position2.z, 6.0);
}

TEST(ViewPointTest, PropertyTest)
{
  ViewPoint viewpoint(0.0, 0.0, 0.0);
  
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
}

TEST(ViewPointTest, AssociatedCellTest)
{
  ViewPoint viewpoint(0.0, 0.0, 0.0);
  
  // Test setting and getting associated cell index
  viewpoint.SetCellInd(42);
  EXPECT_EQ(viewpoint.GetCellInd(), 42);
}

TEST(ViewPointTest, CoverageTest)
{
  ViewPoint viewpoint(0.0, 0.0, 0.0);
  
  // Test initial covered point count is zero
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
  
  // Test adding covered points
  viewpoint.AddCoveredPoint(1);
  viewpoint.AddCoveredPoint(2);
  viewpoint.AddCoveredPoint(3);
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 3);
  
  // Test resetting coverage
  viewpoint.ResetCoverage();
  EXPECT_EQ(viewpoint.GetCoveredPointNum(), 0);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}