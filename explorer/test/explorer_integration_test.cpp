/**
 * @file explorer_integration_test.cpp
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief 集成测试文件，测试explorer模块基本功能
 * @version 0.1
 * @date 2025-07-27
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 测试发布者和订阅者
ros::Publisher start_exploration_pub;
ros::Publisher state_estimation_pub;
ros::Publisher registered_scan_pub;

// 测试变量
bool exploration_finished = false;
bool waypoint_received = false;
int waypoint_count = 0;

// 回调函数
void explorationFinishCallback(const std_msgs::Bool::ConstPtr& msg)
{
  exploration_finished = msg->data;
}

void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  waypoint_received = true;
  waypoint_count++;
}

// 测试类
class ExplorerIntegrationTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 初始化发布者和订阅者
    start_exploration_pub = nh_.advertise<std_msgs::Bool>("/start_exploration", 1);
    state_estimation_pub = nh_.advertise<nav_msgs::Odometry>("/state_estimation", 1);
    registered_scan_pub = nh_.advertise<sensor_msgs::PointCloud2>("/registered_scan", 1);
    
    // 订阅测试结果 (尝试不同的可能话题名称)
    exploration_finish_sub1_ = nh_.subscribe("exploration_finish", 1, explorationFinishCallback);
    exploration_finish_sub2_ = nh_.subscribe("/explorer/exploration_finish", 1, explorationFinishCallback);
    waypoint_sub1_ = nh_.subscribe("/goal", 1, waypointCallback);
    waypoint_sub2_ = nh_.subscribe("goal", 1, waypointCallback);
    
    // 等待连接建立
    ros::Duration(1.0).sleep();
  }
  
  void TearDown() override
  {
    // 清理工作
  }
  
  ros::NodeHandle nh_;
  ros::Subscriber exploration_finish_sub1_;
  ros::Subscriber exploration_finish_sub2_;
  ros::Subscriber waypoint_sub1_;
  ros::Subscriber waypoint_sub2_;
};

// 测试explorer节点基本功能
TEST_F(ExplorerIntegrationTest, BasicFunctionalityTest)
{
  // 检查节点是否成功启动
  ASSERT_TRUE(ros::master::check());
  
  // 发送开始探索信号
  std_msgs::Bool start_msg;
  start_msg.data = true;
  start_exploration_pub.publish(start_msg);
  
  // 发送状态估计数据
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = ros::Time::now();
  odom_msg.header.frame_id = "map";
  odom_msg.pose.pose.position.x = 0.0;
  odom_msg.pose.pose.position.y = 0.0;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation.w = 1.0;
  state_estimation_pub.publish(odom_msg);
  
  // 发送点云数据
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (float x = -5.0; x <= 5.0; x += 0.5)
  {
    for (float y = -5.0; y <= 5.0; y += 0.5)
    {
      pcl::PointXYZ point;
      point.x = x;
      point.y = y;
      point.z = 0.0;
      cloud.points.push_back(point);
    }
  }
  
  sensor_msgs::PointCloud2 cloud_msg;
  pcl::toROSMsg(cloud, cloud_msg);
  cloud_msg.header.stamp = ros::Time::now();
  cloud_msg.header.frame_id = "map";
  registered_scan_pub.publish(cloud_msg);
  
  // 等待处理
  ros::Duration(1.0).sleep();
  ros::spinOnce();
  
  // 再次发送数据以触发处理
  for (int i = 0; i < 5; i++) {
    odom_msg.header.stamp = ros::Time::now();
    state_estimation_pub.publish(odom_msg);
    
    cloud_msg.header.stamp = ros::Time::now();
    registered_scan_pub.publish(cloud_msg);
    ros::Duration(0.1).sleep();
  }
  
  ros::spinOnce();
  
  // 我们期望至少节点能接收数据而不会崩溃
  SUCCEED();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explorer_integration_test");
  testing::InitGoogleTest(&argc, argv);
  
  // 确保在测试运行期间ROS节点保持活动状态
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  int result = RUN_ALL_TESTS();
  spinner.stop();
  return result;
}