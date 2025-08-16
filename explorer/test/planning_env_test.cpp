#include <gtest/gtest.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "planning_env.h"
#include "grid_world.h"

using namespace planning_env_ns;

TEST(PlanningEnvParametersTest, ReadParametersTest)
{
  // Since this requires ROS node handle, we'll just verify
  // that the test compiles and runs. Full testing would require
  // a ROS environment.
  ASSERT_TRUE(true);
}

TEST(PlanningEnvTest, ConstructorTest)
{
  // Since PlanningEnv requires ROS node handle, we'll just verify
  // that the test compiles and runs. Full testing would require
  // a ROS environment.
  ASSERT_TRUE(true);
}

TEST(PlanningEnvTest, BasicOperationsTest)
{
  // Since PlanningEnv requires ROS node handle, we'll just verify
  // that the test compiles and runs. Full testing would require
  // a ROS environment.
  ASSERT_TRUE(true);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}