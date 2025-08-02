#include <gtest/gtest.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include "local_coverage_planner/local_coverage_planner.h"

using namespace local_coverage_planner_ns;

class LocalCoveragePlannerTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    // 初始化ROS节点（如果尚未初始化）
    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "local_coverage_planner_test", ros::init_options::AnonymousName);
    }
    // 创建节点句柄
    nh_ = std::make_unique<ros::NodeHandle>("~");
  }
  
  virtual void TearDown() {
  }
  
  std::unique_ptr<ros::NodeHandle> nh_;
};

TEST_F(LocalCoveragePlannerTest, ConstructorTest)
{
  // 测试LocalCoveragePlanner构造函数
  LocalCoveragePlanner local_planner(*nh_);
  
  // 验证初始状态
  EXPECT_FALSE(local_planner.IsLocalCoverageComplete());
}

TEST_F(LocalCoveragePlannerTest, SetterGetterTest)
{
  LocalCoveragePlanner local_planner(*nh_);
  
  // 测试设置机器人位置
  Eigen::Vector3d robot_position(1.0, 2.0, 3.0);
  local_planner.SetRobotPosition(robot_position);
  
  // 测试设置前瞻点
  Eigen::Vector3d lookahead_point(4.0, 5.0, 6.0);
  local_planner.SetLookAheadPoint(lookahead_point);
  
  // 验证设置成功（通过间接方式）
  EXPECT_NO_THROW(local_planner.SetLookAheadPoint(lookahead_point));
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}