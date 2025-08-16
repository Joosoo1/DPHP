/**
 * @file sensor_coverage_planner_ground.cpp
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int32.h>
#include "explorer/sensor_coverage_planner_ground.h"
#include "explorer/misc_utils.h"
#include "explorer/graph.h"
// 从配置文件中读取参数
namespace sensor_coverage_planner_3d_ns
{
    bool PlannerParameters::ReadParameters(ros::NodeHandle& nh)
    {
        sub_start_exploration_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "sub_start_exploration_topic_", "/exploration_start");
        sub_state_estimation_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "sub_state_estimation_topic_", "/state_estimation_at_scan");
        sub_registered_scan_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "sub_registered_scan_topic_", "/registered_scan");
        sub_coverage_boundary_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "sub_coverage_boundary_topic_", "/coverage_boundary");
        sub_viewpoint_boundary_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "sub_viewpoint_boundary_topic_", "/navigation_boundary");
        sub_nogo_boundary_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "sub_nogo_boundary_topic_", "/nogo_boundary");
        pub_exploration_finish_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "pub_exploration_finish_topic_", "exploration_finish");
        pub_runtime_breakdown_topic_ =
            misc_utils_ns::getParam<std::string>(nh, "pub_runtime_breakdown_topic_", "runtime_breakdown");

        pub_runtime_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_runtime_topic_", "/runtime");
        pub_waypoint_topic_ = misc_utils_ns::getParam<std::string>(nh, "pub_waypoint_topic_", "/way_point");

        pub_momentum_activation_count_topic_ = misc_utils_ns::getParam<std::string>(
            nh, "pub_momentum_activation_count_topic_", "momentum_activation_count");

        // Bool
        kAutoStart = misc_utils_ns::getParam<bool>(nh, "kAutoStart", false);
        kRushHome = misc_utils_ns::getParam<bool>(nh, "kRushHome", false);
        kUseTerrainHeight = misc_utils_ns::getParam<bool>(nh, "kUseTerrainHeight", false);
        kCheckTerrainCollision = misc_utils_ns::getParam<bool>(nh, "kCheckTerrainCollision", false);
        kExtendWayPoint = misc_utils_ns::getParam<bool>(nh, "kExtendWayPoint", true);
        kUseLineOfSightLookAheadPoint = misc_utils_ns::getParam<bool>(nh, "kUseLineOfSightLookAheadPoint", true);
        kNoExplorationReturnHome = misc_utils_ns::getParam<bool>(nh, "kNoExplorationReturnHome", true);
        kUseMomentum = misc_utils_ns::getParam<bool>(nh, "kUseMomentum", false);

        // Double
        kKeyposeCloudDwzFilterLeafSize = misc_utils_ns::getParam<double>(nh, "kKeyposeCloudDwzFilterLeafSize", 0.2);
        kRushHomeDist = misc_utils_ns::getParam<double>(nh, "kRushHomeDist", 10.0);
        kAtHomeDistThreshold = misc_utils_ns::getParam<double>(nh, "kAtHomeDistThreshold", 0.5);
        kTerrainCollisionThreshold = misc_utils_ns::getParam<double>(nh, "kTerrainCollisionThreshold", 0.5);
        kLookAheadDistance = misc_utils_ns::getParam<double>(nh, "kLookAheadDistance", 5.0);
        kExtendWayPointDistanceBig = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceBig", 8.0);
        kExtendWayPointDistanceSmall = misc_utils_ns::getParam<double>(nh, "kExtendWayPointDistanceSmall", 3.0);

        // Int
        kDirectionChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionChangeCounterThr", 4);
        kDirectionNoChangeCounterThr = misc_utils_ns::getParam<int>(nh, "kDirectionNoChangeCounterThr", 5);

        return true;
    }

    void PlannerData::Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
    {
        // 实例化各类点云
        keypose_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "keypose_cloud", kWorldFrameID);
        registered_scan_stack_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>>(nh, "registered_scan_stack", kWorldFrameID);
        registered_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "registered_cloud", kWorldFrameID);
        large_terrain_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud_large", kWorldFrameID);
        terrain_collision_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "terrain_collision_cloud", kWorldFrameID);
        terrain_ext_collision_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "terrain_ext_collision_cloud", kWorldFrameID);
        viewpoint_vis_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "viewpoint_vis_cloud", kWorldFrameID);
        grid_world_vis_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "grid_world_vis_cloud", kWorldFrameID);
        exploration_path_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "bspline_path_cloud", kWorldFrameID);

        selected_viewpoint_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "selected_viewpoint_vis_cloud", kWorldFrameID);
        exploring_cell_vis_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "exploring_cell_vis_cloud", kWorldFrameID);
        collision_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "collision_cloud", kWorldFrameID);
        lookahead_point_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "lookahead_point_cloud", kWorldFrameID);
        keypose_graph_vis_cloud_ =
            std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "keypose_graph_cloud", kWorldFrameID);
        viewpoint_in_collision_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "viewpoint_in_collision_cloud_", kWorldFrameID);
        point_cloud_manager_neighbor_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "pointcloud_manager_cloud", kWorldFrameID);
        reordered_global_subspace_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
            nh, "reordered_global_subspace_cloud", kWorldFrameID);

        /*local planner*/
        planning_env_ = std::make_unique<planning_env_ns::PlanningEnv>(nh, nh_p);
        viewpoint_manager_ = std::make_shared<viewpoint_manager_ns::ViewPointManager>(nh_p);
        local_coverage_planner_ = std::make_unique<local_coverage_planner_ns::LocalCoveragePlanner>(nh_p);
        local_coverage_planner_->SetViewPointManager(
            viewpoint_manager_); // local_coverage_planner与viewpoint_manager关联，用于局部规划

        /*global planner*/
        keypose_graph_ = std::make_unique<keypose_graph_ns::KeyposeGraph>(nh_p);
        grid_world_ = std::make_unique<grid_world_ns::GridWorld>(nh_p);
        grid_world_->SetUseKeyposeGraph(true); // GridWorld同keypose_graph关联，用于全局规划

        /*可视化*/
        visualizer_ = std::make_unique<explorer_visualizer_ns::TAREVisualizer>(nh, nh_p);

        // keypose_graph的节点可视化
        keypose_graph_node_marker_ =
            std::make_unique<misc_utils_ns::Marker>(nh, "keypose_graph_node_marker", kWorldFrameID);
        keypose_graph_node_marker_->SetType(visualization_msgs::Marker::POINTS);
        keypose_graph_node_marker_->SetScale(0.4, 0.4, 0.1);
        keypose_graph_node_marker_->SetColorRGBA(1.0, 0.0, 0.0, 1.0);

        // keypose_graph的边可视化
        keypose_graph_edge_marker_ =
            std::make_unique<misc_utils_ns::Marker>(nh, "keypose_graph_edge_marker", kWorldFrameID);
        keypose_graph_edge_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
        keypose_graph_edge_marker_->SetScale(0.05, 0.0, 0.0);
        keypose_graph_edge_marker_->SetColorRGBA(1.0, 1.0, 0.0, 0.9);

        // 禁止边界可视化
        nogo_boundary_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "nogo_boundary_marker", kWorldFrameID);
        nogo_boundary_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
        nogo_boundary_marker_->SetScale(0.05, 0.0, 0.0);
        nogo_boundary_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

        // grid_world立体框可视化
        grid_world_marker_ = std::make_unique<misc_utils_ns::Marker>(nh, "grid_world_marker", kWorldFrameID);
        grid_world_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
        grid_world_marker_->SetScale(1.0, 1.0, 1.0);
        grid_world_marker_->SetColorRGBA(1.0, 0.0, 0.0, 0.8);

        /*初始化*/
        initial_position_.x() = 0.0;
        initial_position_.y() = 0.0;
        initial_position_.z() = 0.0;
        cur_keypose_node_ind_ = 0;
        robot_yaw_ = 0.0;
        lookahead_point_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
        moving_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
        moving_forward_ = true; // 默认为前进

        Eigen::Vector3d viewpoint_resolution = viewpoint_manager_->GetResolution();
        double add_non_keypose_node_min_dist = std::min(viewpoint_resolution.x(), viewpoint_resolution.y()) / 2;
        keypose_graph_->SetAddNonKeyposeNodeMinDist() =
            add_non_keypose_node_min_dist; // 设置非关键节点最小距离，即普通点

        robot_position_.x = 0;
        robot_position_.y = 0;
        robot_position_.z = 0;

        last_robot_position_ = robot_position_;
    }

    SensorCoveragePlanner3D::SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p) :
        keypose_cloud_update_(false), initialized_(false), lookahead_point_update_(false), relocation_(false),
        start_exploration_(false), exploration_finished_(false), near_home_(false), at_home_(false), stopped_(false),
        test_point_update_(false), viewpoint_ind_update_(false), step_(false), use_momentum_(false),
        lookahead_point_in_line_of_sight_(true), registered_cloud_count_(0), keypose_count_(0),
        direction_change_count_(0), direction_no_change_count_(0), momentum_activation_count_(0)
    {
        // 初始化参数
        initialize(nh, nh_p);
        PrintExplorationStatus("Exploration Started", false);
    }

    bool SensorCoveragePlanner3D::initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p)
    {
        // 从ros参数服务器读取参数
        if (!pp_.ReadParameters(nh_p))
        {
            ROS_ERROR("Read parameters failed");
            return false;
        }

        // 初始化探索规划器数据管理对象，包括local planner和global planner部分
        pd_.Initialize(nh, nh_p);

        // 因为是无人车探索，所以不需要考虑垂直方向，所以禁用
        pd_.keypose_graph_->SetAllowVerticalEdge(true);

        // 根据planning_env的surface点云下采样大小确定LiDARModel点云的下采样分辨率
        lidar_model_ns::LiDARModel::setCloudDWZResol(pd_.planning_env_->GetPlannerCloudResolution());

        // 以1hz的频率执行探索
        execution_timer_ = nh.createTimer(ros::Duration(0.1), &SensorCoveragePlanner3D::execute, this);

        // 是否开始探索的话题
        exploration_start_sub_ =
            nh.subscribe(pp_.sub_start_exploration_topic_, 5, &SensorCoveragePlanner3D::ExplorationStartCallback, this);
        // 订阅配准之后的点云->世界坐标系下的点云
        registered_scan_sub_ =
            nh.subscribe(pp_.sub_registered_scan_topic_, 5, &SensorCoveragePlanner3D::RegisteredScanCallback, this);
        // 订阅里程计的状态估计信息
        state_estimation_sub_ =
            nh.subscribe(pp_.sub_state_estimation_topic_, 5, &SensorCoveragePlanner3D::StateEstimationCallback, this);
        // 订阅探索边界
        viewpoint_boundary_sub_ = nh.subscribe(pp_.sub_viewpoint_boundary_topic_, 1,
                                               &SensorCoveragePlanner3D::ViewPointBoundaryCallback, this);

        // 探索过程可视化，用于发布rviz可视化信息和性能评估
        global_path_full_publisher_ = nh.advertise<nav_msgs::Path>("global_path_full", 1);
        global_path_publisher_ = nh.advertise<nav_msgs::Path>("global_path", 1);
        old_global_path_publisher_ = nh.advertise<nav_msgs::Path>("old_global_path", 1);
        to_nearest_global_subspace_path_publisher_ = nh.advertise<nav_msgs::Path>("to_nearest_global_subspace_path", 1);
        local_tsp_path_publisher_ = nh.advertise<nav_msgs::Path>("local_path", 1);
        exploration_path_publisher_ = nh.advertise<nav_msgs::Path>("exploration_path", 1);
        waypoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>(pp_.pub_waypoint_topic_, 2);
        exploration_finish_pub_ = nh.advertise<std_msgs::Bool>(pp_.pub_exploration_finish_topic_, 2);
        runtime_breakdown_pub_ = nh.advertise<std_msgs::Int32MultiArray>(pp_.pub_runtime_breakdown_topic_, 2);
        runtime_pub_ = nh.advertise<std_msgs::Float32>(pp_.pub_runtime_topic_, 2);
        momentum_activation_count_pub_ = nh.advertise<std_msgs::Int32>(pp_.pub_momentum_activation_count_topic_, 2);
        // Debug
        pointcloud_manager_neighbor_cells_origin_pub_ =
            nh.advertise<geometry_msgs::PointStamped>("pointcloud_manager_neighbor_cells_origin", 1);
        return true;
    }

    void SensorCoveragePlanner3D::ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg)
    {
        // 接收到启动探索的消息
        if (start_msg->data)
        {
            start_exploration_ = true;
        }
    }

    void SensorCoveragePlanner3D::ViewPointBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg)
    {
        pd_.viewpoint_manager_->UpdateViewPointBoundary((*polygon_msg).polygon);
    }

    void SensorCoveragePlanner3D::StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg)
    {
        pd_.robot_position_ = state_estimation_msg->pose.pose.position; // 更新机器人的位置
        // Todo: use a boolean
        if (std::abs(pd_.initial_position_.x()) < 0.01 && std::abs(pd_.initial_position_.y()) < 0.01 &&
            std::abs(pd_.initial_position_.z()) < 0.01)
        {
            pd_.initial_position_.x() = pd_.robot_position_.x;
            pd_.initial_position_.y() = pd_.robot_position_.y;
            pd_.initial_position_.z() = pd_.robot_position_.z;
        } // 初始化机器人的位置

        double roll, pitch, yaw;
        //
        geometry_msgs::Quaternion geo_quat = state_estimation_msg->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geo_quat.x, geo_quat.y, geo_quat.z, geo_quat.w)).getRPY(roll, pitch, yaw);
        // 更新无人机朝向
        pd_.robot_yaw_ = yaw;

        // 坐标系为flu，通过判断x轴的速度来判断机器人是否向前移动
        if (state_estimation_msg->twist.twist.linear.x > 0.4)
        {
            pd_.moving_forward_ = true;
        }
        else if (state_estimation_msg->twist.twist.linear.x < -0.4)
        {
            pd_.moving_forward_ = false;
        }
        // 接收到第一帧机器人的里程计消息则完成探索规划器初始化
        initialized_ = true;
    }

    void SensorCoveragePlanner3D::RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_scan_msg)
    {
        // 等待接受里程计信息，完成初始化
        if (!initialized_)
        {
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr registered_scan_tmp(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*registered_scan_msg, *registered_scan_tmp);
        // 如果接收到点云为空，则直接返回
        if (registered_scan_tmp->points.empty())
        {
            return;
        }

        *(pd_.registered_scan_stack_->cloud_) += *(registered_scan_tmp); // 将点云加入点云堆栈中
        // 对点云进行下采样
        pointcloud_downsizer_.Downsize(registered_scan_tmp, pp_.kKeyposeCloudDwzFilterLeafSize,
                                       pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);
        // 清空扫描点云
        pd_.registered_cloud_->cloud_->clear();
        // 将下采样后的配准点云赋值给配准点云管理器
        pcl::copyPointCloud(*registered_scan_tmp, *(pd_.registered_cloud_->cloud_));

        // 更新planning_env_的输入数据（机器人和当前的世界坐标系下的点云）
        pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_);
        pd_.planning_env_->UpdateRegisteredCloud<pcl::PointXYZI>(pd_.registered_cloud_->cloud_);

        registered_cloud_count_ = (registered_cloud_count_ + 1) % 5;
        if (registered_cloud_count_ == 0) // 每过5帧标记一帧为关键帧
        {
            // initialized_ = true;
            pd_.keypose_.pose.pose.position = pd_.robot_position_;
            pd_.keypose_.pose.covariance[0] = keypose_count_++;
            // 在keypose_graph_中添加关键帧节点，并返回当前关键帧节点的索引
            pd_.cur_keypose_node_ind_ = pd_.keypose_graph_->AddKeyposeNode(pd_.keypose_, *(pd_.planning_env_));

            // 对配准后的点云堆栈进行下采样
            pointcloud_downsizer_.Downsize(pd_.registered_scan_stack_->cloud_, pp_.kKeyposeCloudDwzFilterLeafSize,
                                           pp_.kKeyposeCloudDwzFilterLeafSize, pp_.kKeyposeCloudDwzFilterLeafSize);

            pd_.keypose_cloud_->cloud_->clear(); // 清空当前关键帧点云
            pcl::copyPointCloud(*(pd_.registered_scan_stack_->cloud_),
                                *(pd_.keypose_cloud_->cloud_)); // 将点云堆栈中的点云赋值给当前关键帧点云
            // pd_.keypose_cloud_->Publish();
            pd_.registered_scan_stack_->cloud_->clear(); // 每隔5帧清空一次配准点云堆栈
            keypose_cloud_update_ = true;
        }
    }

    void SensorCoveragePlanner3D::TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg)
    {
        if (pp_.kCheckTerrainCollision)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
            pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_msg, *terrain_map_tmp);
            // 记录了地形图中存在碰撞的点云
            pd_.terrain_collision_cloud_->cloud_->clear();
            for (auto& point : terrain_map_tmp->points)
            {
                if (point.intensity > pp_.kTerrainCollisionThreshold)
                {
                    pd_.terrain_collision_cloud_->cloud_->points.push_back(point); // 提取地形图中的障碍物点云
                }
            }
        }
    }

    void SensorCoveragePlanner3D::TerrainMapExtCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_ext_msg)
    {
        //
        if (pp_.kUseTerrainHeight)
        {
            pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg, *(pd_.large_terrain_cloud_->cloud_));
        }

        if (pp_.kCheckTerrainCollision)
        {
            pcl::fromROSMsg<pcl::PointXYZI>(*terrain_map_ext_msg,
                                            *(pd_.large_terrain_cloud_->cloud_)); // 获取扩展地形图
            pd_.terrain_ext_collision_cloud_->cloud_->clear();

            for (auto& point : pd_.large_terrain_cloud_->cloud_->points)
            {
                if (point.intensity > pp_.kTerrainCollisionThreshold)
                {
                    pd_.terrain_ext_collision_cloud_->cloud_->points.push_back(point); // 提取扩展地形图的障碍物点云
                }
            }
        }
    }

    // 发布一个初始航点，以促使机器人开始探索
    void SensorCoveragePlanner3D::SendInitialWaypoint()
    {
        // send waypoint ahead
        double lx = 12.0;
        double ly = 0.0;
        double dx = cos(pd_.robot_yaw_) * lx - sin(pd_.robot_yaw_) * ly;
        double dy = sin(pd_.robot_yaw_) * lx + cos(pd_.robot_yaw_) * ly;

        geometry_msgs::PoseStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.header.stamp = ros::Time::now();

        waypoint.pose.position.x = pd_.robot_position_.x + dx;
        waypoint.pose.position.y = pd_.robot_position_.y + dy;
        waypoint.pose.position.z = pd_.robot_position_.z + 0.75;

        double yaw = atan2(dy, dx);

        Eigen::Quaterniond q;
        q = GetQuaternionFromYaw(yaw);
        waypoint.pose.orientation.w = q.w();
        waypoint.pose.orientation.x = q.x();
        waypoint.pose.orientation.y = q.y();
        waypoint.pose.orientation.z = q.z();

        std::cout << "init waypoint is: " << waypoint.pose.position.x << ", " << waypoint.pose.position.y << ", "
                  << waypoint.pose.position.z << std::endl;
        waypoint_pub_.publish(waypoint);
    }

    // 更新keypose_graph
    void SensorCoveragePlanner3D::UpdateKeyposeGraph()
    {
        misc_utils_ns::Timer update_keypose_graph_timer("update keypose graph");
        update_keypose_graph_timer.Start();

        pd_.keypose_graph_->GetMarker(pd_.keypose_graph_node_marker_->marker_, pd_.keypose_graph_edge_marker_->marker_);
        // pd_.keypose_graph_node_marker_->Publish();
        pd_.keypose_graph_edge_marker_->Publish();
        pd_.keypose_graph_vis_cloud_->cloud_->clear();
        //
        pd_.keypose_graph_->CheckLocalCollision(pd_.robot_position_, pd_.viewpoint_manager_);
        pd_.keypose_graph_->CheckConnectivity(pd_.robot_position_);
        pd_.keypose_graph_->GetVisualizationCloud(pd_.keypose_graph_vis_cloud_->cloud_);
        pd_.keypose_graph_vis_cloud_->Publish();

        update_keypose_graph_timer.Stop(false);
    }

    // 更新Viewpoint，返回当前候选视点的数量
    int SensorCoveragePlanner3D::UpdateViewPoints()
    {
        misc_utils_ns::Timer collision_cloud_timer("update collision cloud");
        collision_cloud_timer.Start();
        pd_.collision_cloud_->cloud_ = pd_.planning_env_->GetCollisionCloud(); // 获取障碍物点云
        collision_cloud_timer.Stop(false);

        // 更新ViewpointManager
        misc_utils_ns::Timer viewpoint_manager_update_timer("update viewpoint manager");
        viewpoint_manager_update_timer.Start();

        if (pp_.kUseTerrainHeight)
        {
            pd_.viewpoint_manager_->SetViewPointHeightWithTerrain(
                pd_.large_terrain_cloud_->cloud_); // 根据扩展地形图点云获取视点的高度
        }

        if (pp_.kCheckTerrainCollision)
        {
            *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_collision_cloud_->cloud_);
            *(pd_.collision_cloud_->cloud_) += *(pd_.terrain_ext_collision_cloud_->cloud_);
        } // 汇总障碍物点云

        pd_.viewpoint_manager_->CheckViewPointCollision(
            pd_.collision_cloud_->cloud_); // 输入障碍物点云，检查视点是否与障碍物碰撞
        pd_.viewpoint_manager_->CheckViewPointLineOfSight(); // 检查视点是否在视线范围内
        pd_.viewpoint_manager_->CheckViewPointConnectivity(); // 检测视点与其它视点之间的连通性

        int viewpoint_candidate_count = pd_.viewpoint_manager_->GetViewPointCandidate(); // 获取候选视点的数量

        UpdateVisitedPositions(); // 更新无人车已经访问过的位置
        pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.visited_positions_); // 更新视点是否被访问过
        pd_.viewpoint_manager_->UpdateViewPointVisited(pd_.grid_world_); // 更新栅格地图中的cell是否被访问过

        // For visualization
        pd_.collision_cloud_->Publish(); // 发布障碍物点云
        // pd_.collision_grid_cloud_->Publish();
        pd_.viewpoint_manager_->GetCollisionViewPointVisCloud(
            pd_.viewpoint_in_collision_cloud_->cloud_); // 获取存在碰撞的视点点云
        pd_.viewpoint_in_collision_cloud_->Publish(); // 发布存在碰撞的视点点云

        viewpoint_manager_update_timer.Stop(false);
        return viewpoint_candidate_count; // 返回当前候选视点的数量
    }

    // 更新视点的覆盖率
    void SensorCoveragePlanner3D::UpdateViewPointCoverage()
    {
        // Update viewpoint coverage
        misc_utils_ns::Timer update_coverage_timer("update viewpoint coverage");
        update_coverage_timer.Start();
        pd_.viewpoint_manager_->UpdateViewPointCoverage<PlannerCloudPointType>(
            pd_.planning_env_->GetDiffCloud()); // 输入新点云，更新视点的覆盖率
        pd_.viewpoint_manager_->UpdateRolledOverViewPointCoverage<PlannerCloudPointType>(
            pd_.planning_env_->GetStackedCloud()); //
        // Update robot coverage
        pd_.robot_viewpoint_.ResetCoverage(); // 重置机器人视点的覆盖率
        geometry_msgs::Pose robot_pose;
        robot_pose.position = pd_.robot_position_;
        pd_.robot_viewpoint_.setPose(robot_pose);
        UpdateRobotViewPointCoverage(); // 更新机器人视点的覆盖率
        update_coverage_timer.Stop(false);
    }

    // 更新机器人视点的覆盖点云
    void SensorCoveragePlanner3D::UpdateRobotViewPointCoverage()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pd_.planning_env_->GetCollisionCloud();
        for (const auto& point : cloud->points)
        {
            // 在视角和距离范围内
            if (pd_.viewpoint_manager_->InFOVAndRange(
                    Eigen::Vector3d(point.x, point.y, point.z),
                    Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z)))
            {
                pd_.robot_viewpoint_.UpdateCoverage<pcl::PointXYZI>(point);
            }
        }
    }

    // 更新已覆盖区域，返回未覆盖区域点云的数量和边界点云的数量
    void SensorCoveragePlanner3D::UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num)
    {
        // Update covered area
        misc_utils_ns::Timer update_coverage_area_timer("update covered area");
        update_coverage_area_timer.Start();
        pd_.planning_env_->UpdateCoveredArea(pd_.robot_viewpoint_, pd_.viewpoint_manager_); // 更新已覆盖区域
        update_coverage_area_timer.Stop(false);
        // 获取未覆盖区域
        misc_utils_ns::Timer get_uncovered_area_timer("get uncovered area");
        get_uncovered_area_timer.Start();
        pd_.planning_env_->GetUncoveredArea(pd_.viewpoint_manager_, uncovered_point_num, uncovered_frontier_point_num);
        get_uncovered_area_timer.Stop(false);
        // 发布未覆盖区域点云
        pd_.planning_env_->PublishUncoveredCloud();
        // 发布未覆盖边界点云
        pd_.planning_env_->PublishUncoveredFrontierCloud();
    }

    // 更新已访问过的位置
    void SensorCoveragePlanner3D::UpdateVisitedPositions()
    {
        Eigen::Vector3d robot_current_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
        bool existing = false;
        for (int i = 0; i < pd_.visited_positions_.size(); i++)
        {
            // TODO: parameterize this
            if ((robot_current_position - pd_.visited_positions_[i]).norm() < 1)
            {
                existing = true;
                break;
            }
        }
        if (!existing)
        {
            pd_.visited_positions_.push_back(robot_current_position);
        }
    }

    // 更新全局地图
    void SensorCoveragePlanner3D::UpdateGlobalRepresentation()
    {
        pd_.local_coverage_planner_->SetRobotPosition(Eigen::Vector3d(
            pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z)); // 在局部规划器中，更新机器人位置

        bool viewpoint_rollover = pd_.viewpoint_manager_->UpdateRobotPosition(
            Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y,
                            pd_.robot_position_.z)); // 更新机器人位置，并判断是否发生了回绕？

        if (!pd_.grid_world_->Initialized() || viewpoint_rollover)
        {
            pd_.grid_world_->UpdateNeighborCells(pd_.robot_position_); // 更新邻近cell
        }

        pd_.planning_env_->UpdateRobotPosition(pd_.robot_position_); // planning_env_中更新机器人位置
        // 没有......
        pd_.planning_env_->GetVisualizationPointCloud(
            pd_.point_cloud_manager_neighbor_cloud_
                ->cloud_); // 获取点云管理器中邻近cell的点云存储到point_cloud_manager_neighbor_cloud_
        pd_.point_cloud_manager_neighbor_cloud_->Publish(); // 再发布邻近点云

        // DEBUG
        Eigen::Vector3d pointcloud_manager_neighbor_cells_origin =
            pd_.planning_env_->GetPointCloudManagerNeighborCellsOrigin();
        geometry_msgs::PointStamped pointcloud_manager_neighbor_cells_origin_point;
        pointcloud_manager_neighbor_cells_origin_point.header.frame_id = "map";
        pointcloud_manager_neighbor_cells_origin_point.header.stamp = ros::Time::now();
        pointcloud_manager_neighbor_cells_origin_point.point.x = pointcloud_manager_neighbor_cells_origin.x();
        pointcloud_manager_neighbor_cells_origin_point.point.y = pointcloud_manager_neighbor_cells_origin.y();
        pointcloud_manager_neighbor_cells_origin_point.point.z = pointcloud_manager_neighbor_cells_origin.z();
        pointcloud_manager_neighbor_cells_origin_pub_.publish(pointcloud_manager_neighbor_cells_origin_point);

        if (exploration_finished_ && pp_.kNoExplorationReturnHome)
        {
            pd_.planning_env_->SetUseFrontier(false);
        }
        pd_.planning_env_->UpdateKeyposeCloud<PlannerCloudPointType>(pd_.keypose_cloud_->cloud_);

        int closest_node_ind =
            pd_.keypose_graph_->GetClosestNodeInd(pd_.robot_position_); // 获取当前机器人位置最近的keypose节点索引
        geometry_msgs::Point closest_node_position = pd_.keypose_graph_->GetClosestNodePosition(
            pd_.robot_position_); // 获取当前机器人位置最近的keypose节点的位置
        pd_.grid_world_->SetCurKeyposeGraphNodeInd(closest_node_ind); // grid_world_设置当前keypose节点索引和位置
        pd_.grid_world_->SetCurKeyposeGraphNodePosition(closest_node_position); // grid_world_设置当前keypose节点位置

        pd_.grid_world_->UpdateRobotPosition(pd_.robot_position_); // 更新机器人位置在grid_world_
        if (!pd_.grid_world_->HomeSet())
        {
            pd_.grid_world_->SetHomePosition(pd_.initial_position_);
        } // 如果未设置home点，则初始位置为home点
    }

    // 根据cell_order，求解全局引导路径
    void SensorCoveragePlanner3D::GlobalPlanning(std::vector<int>& global_cell_tsp_order,
                                                 exploration_path_ns::ExplorationPath& global_path)
    {
        misc_utils_ns::Timer global_tsp_timer("Global planning");
        global_tsp_timer.Start();

        pd_.grid_world_->UpdateCellStatus(pd_.viewpoint_manager_); // 更新cell状态
        pd_.grid_world_->UpdateCellKeyposeGraphNodes(pd_.keypose_graph_); // 更新keypose_graph
        pd_.grid_world_->AddPathsInBetweenCells(pd_.viewpoint_manager_, pd_.keypose_graph_); // 搜寻两两cell之间的路径

        pd_.viewpoint_manager_->UpdateCandidateViewPointCellStatus(pd_.grid_world_); // 根据候选视点更新cell状态

        // 边框Viewpoint
        global_path = pd_.grid_world_->SolveGlobalTSP(pd_.viewpoint_manager_, global_cell_tsp_order,
                                                      pd_.keypose_graph_); // 由视点、keypose_graph求解全局路径

        global_tsp_timer.Stop(false);
        global_planning_runtime_ = global_tsp_timer.GetDuration("ms");
    }

    // 发布全局探索规划路径
    void
    SensorCoveragePlanner3D::PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath& global_path,
                                                                const exploration_path_ns::ExplorationPath& local_path)
    {
        // 发布全局路径
        nav_msgs::Path global_path_full = global_path.GetPath();
        global_path_full.header.frame_id = "map";
        global_path_full.header.stamp = ros::Time::now();
        global_path_full_publisher_.publish(global_path_full);
        // Get the part that connects with the local path

        int start_index = 0;
        for (int i = 0; i < global_path.nodes_.size(); i++)
        {
            if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
                global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
                !pd_.viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
            {
                break;
            }
            start_index = i;
        }

        int end_index = global_path.nodes_.size() - 1;

        for (int i = global_path.nodes_.size() - 1; i >= 0; i--)
        {
            if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
                global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
                !pd_.viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
            {
                break;
            }
            end_index = i;
        }

        // 修剪全局路径
        nav_msgs::Path global_path_trim;
        if (local_path.nodes_.size() >= 2)
        {
            geometry_msgs::PoseStamped first_pose;
            first_pose.pose.position.x = local_path.nodes_.front().position_.x();
            first_pose.pose.position.y = local_path.nodes_.front().position_.y();
            first_pose.pose.position.z = local_path.nodes_.front().position_.z();
            global_path_trim.poses.push_back(first_pose);
        }

        for (int i = start_index; i <= end_index; i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = global_path.nodes_[i].position_.x();
            pose.pose.position.y = global_path.nodes_[i].position_.y();
            pose.pose.position.z = global_path.nodes_[i].position_.z();
            global_path_trim.poses.push_back(pose);
        }

        if (local_path.nodes_.size() >= 2)
        {
            geometry_msgs::PoseStamped last_pose;
            last_pose.pose.position.x = local_path.nodes_.back().position_.x();
            last_pose.pose.position.y = local_path.nodes_.back().position_.y();
            last_pose.pose.position.z = local_path.nodes_.back().position_.z();
            global_path_trim.poses.push_back(last_pose);
        }

        global_path_trim.header.frame_id = "map";
        global_path_trim.header.stamp = ros::Time::now();
        global_path_publisher_.publish(global_path_trim);

        pd_.grid_world_->GetVisualizationCloud(pd_.grid_world_vis_cloud_->cloud_);
        pd_.grid_world_vis_cloud_->Publish();
        pd_.grid_world_->GetMarker(pd_.grid_world_marker_->marker_);
        pd_.grid_world_marker_->Publish();
        nav_msgs::Path full_path = pd_.exploration_path_.GetPath();
        full_path.header.frame_id = "map";
        full_path.header.stamp = ros::Time::now();
        // exploration_path_publisher_.publish(full_path);
        pd_.exploration_path_.GetVisualizationCloud(pd_.exploration_path_cloud_->cloud_);
        pd_.exploration_path_cloud_->Publish();
        // pd_.planning_env_->PublishStackedCloud();
    }

    // 局部探索路径规划
    void SensorCoveragePlanner3D::LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                                                const exploration_path_ns::ExplorationPath& global_path,
                                                exploration_path_ns::ExplorationPath& local_path)
    {
        misc_utils_ns::Timer local_tsp_timer("Local planning");
        local_tsp_timer.Start();
        // lookahead_point是否已经更新，如果已经更新则设置新的前向点
        if (lookahead_point_update_)
        {
            //
            pd_.local_coverage_planner_->SetLookAheadPoint(pd_.lookahead_point_);
        }
        // 根据全局路径和uncovered点更新局部探索路径
        local_path = pd_.local_coverage_planner_->SolveLocalCoverageProblem(global_path, uncovered_point_num,
                                                                            uncovered_frontier_point_num);
        local_tsp_timer.Stop(false);
    }

    void
    SensorCoveragePlanner3D::PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path)
    {
        pd_.viewpoint_manager_->GetVisualizationCloud(pd_.viewpoint_vis_cloud_->cloud_);
        pd_.viewpoint_vis_cloud_->Publish(); // 视点可视化点云
        pd_.lookahead_point_cloud_->Publish(); // 前向点
        nav_msgs::Path local_tsp_path = local_path.GetPath();
        local_tsp_path.header.frame_id = "map";
        local_tsp_path.header.stamp = ros::Time::now();
        local_tsp_path_publisher_.publish(local_tsp_path); // 局部路径
        pd_.local_coverage_planner_->GetSelectedViewPointVisCloud(pd_.selected_viewpoint_vis_cloud_->cloud_);
        pd_.selected_viewpoint_vis_cloud_->Publish(); // 被选中的视点点云
    }

    // 连接全局路径和局部路径
    exploration_path_ns::ExplorationPath
    SensorCoveragePlanner3D::ConcatenateGlobalLocalPath(const exploration_path_ns::ExplorationPath& global_path,
                                                        const exploration_path_ns::ExplorationPath& local_path)
    {
        exploration_path_ns::ExplorationPath full_path;
        // 如果机器人正在向home点移动，则将当前点和home点添加到全局路径中
        if (exploration_finished_ && near_home_ && pp_.kRushHome)
        {
            exploration_path_ns::Node node;
            node.position_.x() = pd_.robot_position_.x;
            node.position_.y() = pd_.robot_position_.y;
            node.position_.z() = pd_.robot_position_.z;
            node.type_ = exploration_path_ns::NodeType::ROBOT;
            full_path.nodes_.push_back(node);
            //
            node.position_ = pd_.initial_position_;
            node.type_ = exploration_path_ns::NodeType::HOME;
            full_path.nodes_.push_back(node);
            return full_path;
        }

        // 获取当前全局和局部路径的长度
        double global_path_length = global_path.GetLength();
        double local_path_length = local_path.GetLength();
        // 若全局路径和局部路径长度过小，则直接返回full_path-->返回空路径
        if (global_path_length < 3 && local_path_length < 5)
        {
            return full_path;
        }
        else
        {
            full_path = local_path; // 初始化为full_path为loacl_path
            if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
                local_path.nodes_.back().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START)
            {
                full_path.Reverse(); // 如果local_path第一个点位end最后一个点为start，则翻转路径
            }
            else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_START &&
                     local_path.nodes_.back() == local_path.nodes_.front())
            {
                full_path.nodes_.back().type_ = exploration_path_ns::NodeType::LOCAL_PATH_END; //
            }
            else if (local_path.nodes_.front().type_ == exploration_path_ns::NodeType::LOCAL_PATH_END &&
                     local_path.nodes_.back() == local_path.nodes_.front())
            {
                full_path.nodes_.front().type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
            }
        } // 最后得到顺序正确，包含起点和终点的full_path

        return full_path; //
    }

    // 考虑了多种情况的获取lookahead_point，以引导机器人探索
    bool SensorCoveragePlanner3D::GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                                                    const exploration_path_ns::ExplorationPath& global_path,
                                                    Eigen::Vector3d& lookahead_point)
    {
        Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
        // Determine which direction to follow on the global path
        // 判断前进方向
        double dist_from_start = 0.0;
        for (int i = 1; i < global_path.nodes_.size(); i++)
        {
            // 计算GLOBAL_VIEWPOINT到起点在全局路径中的距离
            dist_from_start += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
            if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
            {
                break;
            }
        }

        double dist_from_end = 0.0;
        for (int i = global_path.nodes_.size() - 2; i > 0; i--)
        {
            // 同上，计算终点到GLOBAL_VIEWPOINT在全局路径中的距离
            dist_from_end += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
            if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT)
            {
                break;
            }
        }

        bool local_path_too_short = true;
        for (int i = 0; i < local_path.nodes_.size(); i++)
        {
            //
            double dist_to_robot = (robot_position - local_path.nodes_[i].position_).norm();
            if (dist_to_robot > pp_.kLookAheadDistance / 5)
            {
                local_path_too_short = false;
                break;
            }
        } // 如果机器人到局部路径中某点的距离大于***，则不认为局部路径过短

        /*如果局部路径节点数小于1或者局部路径过短，且GLOBAL_VIEWPOINT到起点的距离小于GLOBAL_VIEWPOINT到终点的距离，
          则在全局路径中，选取距离机器人位置大于1/2*kLookAheadDistance作为lookahead_point，引导机器人
        */
        if (local_path.GetNodeNum() < 1 || local_path_too_short)
        {
            if (dist_from_start < dist_from_end)
            {
                double dist_from_robot = 0.0;
                for (int i = 1; i < global_path.nodes_.size(); i++)
                {
                    dist_from_robot += (global_path.nodes_[i - 1].position_ - global_path.nodes_[i].position_).norm();
                    if (dist_from_robot > pp_.kLookAheadDistance / 2)
                    {
                        lookahead_point = global_path.nodes_[i].position_;
                        break;
                    }
                }
            }
            else // 如果GLOBAL_VIEWPOINT到起点的距离大于等于GLOBAL_VIEWPOINT到终点的距离，则为反方向的全局路径上的lookahead_point
            {
                double dist_from_robot = 0.0;
                for (int i = global_path.nodes_.size() - 2; i > 0; i--)
                {
                    dist_from_robot += (global_path.nodes_[i + 1].position_ - global_path.nodes_[i].position_).norm();
                    if (dist_from_robot > pp_.kLookAheadDistance / 2)
                    {
                        lookahead_point = global_path.nodes_[i].position_;
                        break;
                    }
                }
            }
            return false; // 无论是否得到都返回false
        }

        bool has_lookahead = false;
        bool dir = true;
        int robot_i = 0; // 机器人在局部路径中的索引
        int lookahead_i = 0; // lookahead_point在局部路径中的索引

        for (int i = 0; i < local_path.nodes_.size(); i++)
        {
            if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::ROBOT)
            {
                robot_i = i;
            }
            if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOOKAHEAD_POINT)
            {
                has_lookahead = true;
                lookahead_i = i;
            }
        }

        int forward_viewpoint_count = 0; // 前向Viewpoint计数
        int backward_viewpoint_count = 0; // 后向Viewpoint计数

        bool local_loop = false;
        // 如果局部路径起点和终点一致，且节点属性为ROBOT，则局部路径为回环路径
        if (local_path.nodes_.front() == local_path.nodes_.back() &&
            local_path.nodes_.front().type_ == exploration_path_ns::NodeType::ROBOT)
        {
            local_loop = true;
        } // 如果局部路径为循环路径，则robot_i置为0
        if (local_loop)
        {
            robot_i = 0;
        }

        // 以机器人在局部路径中的索引，向前遍历，计算前向Viewpoint的数量
        for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
        {
            if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
            {
                forward_viewpoint_count++;
            }
        }

        // 如果为回环路径，则robot_i置为局部路径最后一个节点的索引
        if (local_loop)
        {
            robot_i = local_path.nodes_.size() - 1;
        }
        // 以机器人在局部路径中的索引，向后遍历，计算后向Viewpoint的数量
        for (int i = robot_i - 1; i >= 0; i--)
        {
            if (local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT)
            {
                backward_viewpoint_count++;
            }
        }

        // 初始化前向和后向引导点为机器人位置
        Eigen::Vector3d forward_lookahead_point = robot_position;
        Eigen::Vector3d backward_lookahead_point = robot_position;

        bool has_forward = false;
        bool has_backward = false;

        if (local_loop)
        {
            robot_i = 0;
        }
        bool forward_lookahead_point_in_los = true;
        bool backward_lookahead_point_in_los = true;
        double length_from_robot = 0.0;

        for (int i = robot_i + 1; i < local_path.GetNodeNum(); i++)
        {
            length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i - 1].position_).norm();
            double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
            bool in_line_of_sight = true;
            if (i < local_path.GetNodeNum() - 1)
            {
                in_line_of_sight = pd_.viewpoint_manager_->InCurrentFrameLineOfSight(
                    local_path.nodes_[i + 1].position_); // 判断是否在当前帧的视场内
            }
            // 如果该点距离超过kLookAheadDistance，或者不在当前帧的视场内，或者为LOCAL_VIEWPOINT，LOCAL_PATH_START，LOCAL_PATH_END，或者为最后一个点
            if ((length_from_robot > pp_.kLookAheadDistance ||
                 (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
                 local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
                 local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
                 local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END ||
                 i == local_path.GetNodeNum() - 1))

            {
                // 如果该点不在当前帧的视场内，则lookahead_point_in_los置为false，引导点不在当前范围内
                if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
                {
                    forward_lookahead_point_in_los = false;
                }
                // 则前向引导点为该点
                forward_lookahead_point = local_path.nodes_[i].position_;
                has_forward = true;
                break;
            }
        }
        // 如果为回环路径，则robot_i置为局部路径最后一个节点的索引
        if (local_loop)
        {
            robot_i = local_path.nodes_.size() - 1;
        }
        // 同上，计算后向引导点
        length_from_robot = 0.0;
        for (int i = robot_i - 1; i >= 0; i--)
        {
            length_from_robot += (local_path.nodes_[i].position_ - local_path.nodes_[i + 1].position_).norm();
            double dist_to_robot = (local_path.nodes_[i].position_ - robot_position).norm();
            bool in_line_of_sight = true;
            if (i > 0)
            {
                in_line_of_sight =
                    pd_.viewpoint_manager_->InCurrentFrameLineOfSight(local_path.nodes_[i - 1].position_);
            }
            if ((length_from_robot > pp_.kLookAheadDistance ||
                 (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight) ||
                 local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_VIEWPOINT ||
                 local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_START ||
                 local_path.nodes_[i].type_ == exploration_path_ns::NodeType::LOCAL_PATH_END || i == 0))

            {
                if (pp_.kUseLineOfSightLookAheadPoint && !in_line_of_sight)
                {
                    backward_lookahead_point_in_los = false;
                }
                backward_lookahead_point = local_path.nodes_[i].position_;
                has_backward = true;
                break;
            }
        }

        if (forward_viewpoint_count > 0 && !has_forward)
        {
            std::cout << "forward viewpoint count > 0 but does not have forward lookahead point" << std::endl;
            exit(1);
        }
        if (backward_viewpoint_count > 0 && !has_backward)
        {
            std::cout << "backward viewpoint count > 0 but does not have backward lookahead point" << std::endl;
            exit(1);
        }

        double dx = pd_.lookahead_point_direction_.x(); //
        double dy = pd_.lookahead_point_direction_.y(); // 获取当前lookahead_point_direction_

        double forward_angle_score = -2;
        double backward_angle_score = -2;
        double lookahead_angle_score = -2;

        double dist_robot_to_lookahead = 0.0;
        if (has_forward)
        {
            Eigen::Vector3d forward_diff = forward_lookahead_point - robot_position;
            forward_diff.z() = 0.0;
            forward_diff = forward_diff.normalized(); // 归一化，模长为1
            forward_angle_score = dx * forward_diff.x() + dy * forward_diff.y(); // 与前向lookahead点方向的夹角
        }
        if (has_backward)
        {
            Eigen::Vector3d backward_diff = backward_lookahead_point - robot_position;
            backward_diff.z() = 0.0;
            backward_diff = backward_diff.normalized();
            backward_angle_score = dx * backward_diff.x() + dy * backward_diff.y(); // 与前向lookahead点方向的夹角
        }
        if (has_lookahead)
        {
            Eigen::Vector3d prev_lookahead_point = local_path.nodes_[lookahead_i].position_;
            dist_robot_to_lookahead = (robot_position - prev_lookahead_point).norm();
            Eigen::Vector3d diff = prev_lookahead_point - robot_position;
            diff.z() = 0.0;
            diff = diff.normalized();
            lookahead_angle_score = dx * diff.x() + dy * diff.y(); // 与前向lookahead点方向的夹角
        } // 计算前向、后向、lookahead点的分数，即与当前lookahead_point_direction_的夹角

        pd_.lookahead_point_cloud_->cloud_->clear(); // 清空前向点云

        if (forward_viewpoint_count == 0 && backward_viewpoint_count == 0)
        {
            relocation_ = true;
        }
        else
        {
            relocation_ = false;
        }

        if (relocation_) // 该区域无Viewpoint时
        {
            if (use_momentum_ &&
                pp_.kUseMomentum) // 如果使用惯性动量，避免频繁切换，则根据前向、后向lookahead点分数选择lookahead_point
            {
                if (forward_angle_score > backward_angle_score)
                {
                    lookahead_point = forward_lookahead_point;
                }
                else
                {
                    lookahead_point = backward_lookahead_point;
                }
            }
            else // 如果不使用惯性动量，则根据距离，选取短距离的lookahead_point
            {
                // follow the shorter distance one
                if (dist_from_start < dist_from_end &&
                    local_path.nodes_.front().type_ != exploration_path_ns::NodeType::ROBOT)
                {
                    lookahead_point = backward_lookahead_point;
                }
                else if (dist_from_end < dist_from_start &&
                         local_path.nodes_.back().type_ != exploration_path_ns::NodeType::ROBOT)
                {
                    lookahead_point = forward_lookahead_point;
                }
                else
                {
                    lookahead_point =
                        forward_angle_score > backward_angle_score ? forward_lookahead_point : backward_lookahead_point;
                }
            }
        } //
        else if (has_lookahead && lookahead_angle_score > 0 && dist_robot_to_lookahead > pp_.kLookAheadDistance / 2 &&
                 pd_.viewpoint_manager_->InLocalPlanningHorizon(local_path.nodes_[lookahead_i].position_))
        {
            lookahead_point = local_path.nodes_[lookahead_i].position_; // 按照规划的lookahead点
        }
        else // 其它
        {
            if (forward_angle_score > backward_angle_score)
            {
                if (forward_viewpoint_count > 0)
                {
                    lookahead_point = forward_lookahead_point;
                }
                else
                {
                    lookahead_point = backward_lookahead_point;
                }
            }
            else
            {
                if (backward_viewpoint_count > 0)
                {
                    lookahead_point = backward_lookahead_point;
                }
                else
                {
                    lookahead_point = forward_lookahead_point;
                }
            }
        }

        // forward_lookahead_point_in_los--->forward_lookahead_point_in_line_of_sight_
        // 如果前向引导点为选定的引导点但不在前向视野范围内，则判定lookahead_point_in_line_of_sight_=false
        if ((lookahead_point == forward_lookahead_point && !forward_lookahead_point_in_los) ||
            (lookahead_point == backward_lookahead_point && !backward_lookahead_point_in_los))
        {
            lookahead_point_in_line_of_sight_ = false;
        }
        else
        {
            lookahead_point_in_line_of_sight_ = true;
        }

        // 更新前向方向
        pd_.lookahead_point_direction_ = lookahead_point - robot_position;
        pd_.lookahead_point_direction_.z() = 0.0;
        pd_.lookahead_point_direction_.normalize();

        // 发布lookahead点
        pcl::PointXYZI point;
        point.x = lookahead_point.x();
        point.y = lookahead_point.y();
        point.z = lookahead_point.z();
        point.intensity = 1.0;
        pd_.lookahead_point_cloud_->cloud_->points.push_back(point);

        // 如果在loaclpath中有lookahead点，则把该点加入到lookahead点云中，并发布出来
        if (has_lookahead)
        {
            point.x = local_path.nodes_[lookahead_i].position_.x();
            point.y = local_path.nodes_[lookahead_i].position_.y();
            point.z = local_path.nodes_[lookahead_i].position_.z();
            point.intensity = 0;
            pd_.lookahead_point_cloud_->cloud_->points.push_back(point);
        }
        return true;
    }

    // 使用lookahead_point作为waypoint，并在此方向上扩展一定距离
    void SensorCoveragePlanner3D::PublishWaypoint()
    {
        geometry_msgs::PoseStamped waypoint;
        // 如果机器人探索完毕，且机器人接近home点，且允许回home点，则waypoint设置为初始位置
        if (exploration_finished_ && near_home_ && pp_.kRushHome)
        {
            waypoint.pose.position.x = pd_.initial_position_.x();
            waypoint.pose.position.y = pd_.initial_position_.y();
            waypoint.pose.position.z = pd_.initial_position_.z();
        }
        else
        {
            double dx = pd_.lookahead_point_.x() - pd_.robot_position_.x;
            double dy = pd_.lookahead_point_.y() - pd_.robot_position_.y;
            double r = sqrt(dx * dx + dy * dy); // 计算lookahead点与机器人位置的欧式距离
            // 如果lookahead点在视线范围内，则航点的扩展距离为极大值，否则扩展距离为极小值
            double extend_dist =
                lookahead_point_in_line_of_sight_ ? pp_.kExtendWayPointDistanceBig : pp_.kExtendWayPointDistanceSmall;

            // 如果lookahead点与机器人距离小于扩展距离，且允许扩展waypoint
            if (r < extend_dist && pp_.kExtendWayPoint)
            {
                dx = dx / r * extend_dist;
                dy = dy / r * extend_dist; // 在此方向上扩展extend_dist
            }
            // 得到扩展后的waypoint
            waypoint.pose.position.x = dx + pd_.robot_position_.x;
            waypoint.pose.position.y = dy + pd_.robot_position_.y;
            waypoint.pose.position.z = pd_.lookahead_point_.z();
        }

        const double x_diff = waypoint.pose.position.x - pd_.robot_position_.x;
        const double y_diff = waypoint.pose.position.y - pd_.robot_position_.y;

        const double yaw = atan2(y_diff, x_diff);

        Eigen::Quaterniond q = GetQuaternionFromYaw(yaw);

        waypoint.pose.orientation.w = q.w();
        waypoint.pose.orientation.x = q.x();
        waypoint.pose.orientation.y = q.y();
        waypoint.pose.orientation.z = q.z();

        std::cout << "waypoint: " << waypoint.pose.position.x << ", " << waypoint.pose.position.y << ", "
                  << waypoint.pose.position.z << std::endl;
        // waypoint.point.z = 0.75;
        // 发布waypoint
        misc_utils_ns::Publish<geometry_msgs::PoseStamped>(waypoint_pub_, waypoint, kWorldFrameID);
    }

    Eigen::Quaterniond SensorCoveragePlanner3D::GetQuaternionFromYaw(const double yaw)
    {
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
        return q;
    }

    // 发布runtime，用于验证探索效率
    void SensorCoveragePlanner3D::PublishRuntime()
    {
        local_viewpoint_sampling_runtime_ = pd_.local_coverage_planner_->GetViewPointSamplingRuntime() / 1000;
        local_path_finding_runtime_ =
            (pd_.local_coverage_planner_->GetFindPathRuntime() + pd_.local_coverage_planner_->GetTSPRuntime()) / 1000;

        std_msgs::Int32MultiArray runtime_breakdown_msg;
        runtime_breakdown_msg.data.clear();
        runtime_breakdown_msg.data.push_back(update_representation_runtime_);
        runtime_breakdown_msg.data.push_back(local_viewpoint_sampling_runtime_);
        runtime_breakdown_msg.data.push_back(local_path_finding_runtime_);
        runtime_breakdown_msg.data.push_back(global_planning_runtime_);
        runtime_breakdown_msg.data.push_back(trajectory_optimization_runtime_);
        runtime_breakdown_msg.data.push_back(overall_runtime_);
        runtime_breakdown_pub_.publish(runtime_breakdown_msg);

        float runtime = 0;
        if (!exploration_finished_ && pp_.kNoExplorationReturnHome)
        {
            for (int i = 0; i < runtime_breakdown_msg.data.size() - 1; i++)
            {
                runtime += runtime_breakdown_msg.data[i];
            }
        }

        std_msgs::Float32 runtime_msg;
        runtime_msg.data = runtime / 1000.0;
        runtime_pub_.publish(runtime_msg);
    }

    // 获取机器人与home点的距离-->欧式距离
    double SensorCoveragePlanner3D::GetRobotToHomeDistance() const
    {
        const Eigen::Vector3d robot_position(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z);
        return (robot_position - pd_.initial_position_).norm();
    }

    // 发布探索状态
    void SensorCoveragePlanner3D::PublishExplorationState() const
    {
        std_msgs::Bool exploration_finished_msg;
        exploration_finished_msg.data = exploration_finished_;
        exploration_finish_pub_.publish(exploration_finished_msg);
    }

    //
    void SensorCoveragePlanner3D::PrintExplorationStatus(const std::string& status, const bool clear_last_line)
    {
        if (clear_last_line)
        {
            printf(cursup);
            printf(cursclean);
            printf(cursup);
            printf(cursclean);
        }
        // 换行输出
        std::cout << std::endl << "\033[1;32m" << status << "\033[0m" << std::endl;
    }

    // 记录转向次数
    void SensorCoveragePlanner3D::CountDirectionChange()
    {
        // 确定当前的移动方向
        Eigen::Vector3d current_moving_direction_ =
            Eigen::Vector3d(pd_.robot_position_.x, pd_.robot_position_.y, pd_.robot_position_.z) -
            Eigen::Vector3d(pd_.last_robot_position_.x, pd_.last_robot_position_.y, pd_.last_robot_position_.z);

        //
        if (current_moving_direction_.norm() > 0.5)
        {
            // 点积：大于零表示向量方向相同（锐角），小于零表示向量方向相反（钝角），等于零（正交）
            if (pd_.moving_direction_.dot(current_moving_direction_) < 0) // 如果当前移动方向发生较大变化
            {
                direction_change_count_++; // 转向次数+1
                direction_no_change_count_ = 0; // 重置连续不转向次数为零
                if (direction_change_count_ > pp_.kDirectionChangeCounterThr) // 如果连续转向次数超过阈值
                {
                    if (!use_momentum_) // 且不使用动量
                    {
                        momentum_activation_count_++; // 动量激活次数+1
                    }
                    use_momentum_ = true; // 使用动量
                }
            }
            else // 如果连续转向次数未超过阈值
            {
                direction_no_change_count_++;
                if (direction_no_change_count_ > pp_.kDirectionNoChangeCounterThr) // 如果连续不转向次数超过阈值
                {
                    direction_change_count_ = 0; // 重置连续转向次数为零
                    use_momentum_ = false; // 不使用动量
                }
            }
            pd_.moving_direction_ = current_moving_direction_; // 更新当前移动方向
        }
        pd_.last_robot_position_ = pd_.robot_position_; // 更新上一次机器人位置

        std_msgs::Int32 momentum_activation_count_msg;
        momentum_activation_count_msg.data = momentum_activation_count_;
        momentum_activation_count_pub_.publish(momentum_activation_count_msg);
    }

    // 主函数
    void SensorCoveragePlanner3D::execute(const ros::TimerEvent&)
    {
        if (!pp_.kAutoStart && !start_exploration_)
        {
            ROS_INFO("Waiting for start signal");
            return;
        }

        Timer overall_processing_timer("overall processing");
        update_representation_runtime_ = 0;
        local_viewpoint_sampling_runtime_ = 0;
        local_path_finding_runtime_ = 0;
        global_planning_runtime_ = 0;
        trajectory_optimization_runtime_ = 0;
        overall_runtime_ = 0; // 探索总运行时间

        if (!initialized_)
        {
            SendInitialWaypoint();
            start_time_ = ros::Time::now();
            global_direction_switch_time_ = ros::Time::now();
            return;
        } // 发布初始waypoint，并开始探索

        overall_processing_timer.Start();
        if (keypose_cloud_update_)
        {
            keypose_cloud_update_ = false;

            CountDirectionChange();

            misc_utils_ns::Timer update_representation_timer("update representation");
            update_representation_timer.Start();

            // Update grid world
            UpdateGlobalRepresentation(); // for global planning

            int viewpoint_candidate_count = UpdateViewPoints(); //

            if (viewpoint_candidate_count == 0)
            {
                ROS_WARN("Cannot get candidate viewpoints, skipping this round");
                return;
            }

            UpdateKeyposeGraph(); // 更新roadmap

            int uncovered_point_num = 0;
            int uncovered_frontier_point_num = 0;
            if (!exploration_finished_ || !pp_.kNoExplorationReturnHome)
            {
                UpdateViewPointCoverage();
                UpdateCoveredAreas(uncovered_point_num, uncovered_frontier_point_num);
            }
            else
            {
                pd_.viewpoint_manager_->ResetViewPointCoverage();
            }

            update_representation_timer.Stop(false);
            update_representation_runtime_ += update_representation_timer.GetDuration("ms");

            // Global TSP
            std::vector<int> global_cell_tsp_order;
            exploration_path_ns::ExplorationPath global_path;
            GlobalPlanning(global_cell_tsp_order, global_path); // 求解全局路径

            // Local TSP
            exploration_path_ns::ExplorationPath local_path;
            LocalPlanning(uncovered_point_num, uncovered_frontier_point_num, global_path, local_path);

            near_home_ = GetRobotToHomeDistance() < pp_.kRushHomeDist;
            at_home_ = GetRobotToHomeDistance() < pp_.kAtHomeDistThreshold;

            // 判断是否探索完毕的判断条件，从grid和local_planner中判断
            if (pd_.grid_world_->IsReturningHome() && pd_.local_coverage_planner_->IsLocalCoverageComplete() &&
                (ros::Time::now() - start_time_).toSec() > 5)
            {
                if (!exploration_finished_)
                {
                    PrintExplorationStatus("Exploration completed, returning home", false);
                }
                exploration_finished_ = true;
            }

            if (exploration_finished_ && at_home_ && !stopped_)
            {
                PrintExplorationStatus("Return home completed", false);
                stopped_ = true;
            }

            // 返回有起点终点的局部探索路径
            pd_.exploration_path_ = ConcatenateGlobalLocalPath(global_path, local_path);

            // 发布探索状态
            PublishExplorationState();

            // 获取引导点lookahead
            lookahead_point_update_ = GetLookAheadPoint(pd_.exploration_path_, global_path, pd_.lookahead_point_);
            PublishWaypoint();

            overall_processing_timer.Stop(false);
            overall_runtime_ = overall_processing_timer.GetDuration("ms");

            pd_.visualizer_->GetGlobalSubspaceMarker(pd_.grid_world_, global_cell_tsp_order);
            Eigen::Vector3d viewpoint_origin = pd_.viewpoint_manager_->GetOrigin();
            pd_.visualizer_->GetLocalPlanningHorizonMarker(viewpoint_origin.x(), viewpoint_origin.y(),
                                                           pd_.robot_position_.z);
            pd_.visualizer_->PublishMarkers();

            PublishLocalPlanningVisualization(local_path);
            PublishGlobalPlanningVisualization(global_path, local_path);
            PublishRuntime();
        }
    }
} // namespace sensor_coverage_planner_3d_ns
