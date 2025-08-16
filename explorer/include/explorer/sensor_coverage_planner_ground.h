/**
 * @file sensor_coverage_planner_ground.h
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include <Eigen/Core>
// ROS
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
// PCL
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
// Third parties
#include "explorer/misc_utils.h"
#include "explorer/pointcloud_utils.h"
// Components
#include "explorer/exploration_path.h"
#include "explorer/explorer_visualizer.h"
#include "explorer/grid_world.h"
#include "explorer/keypose_graph.h" // 用于全局引导路径规划---->在探索过程中不断建立Node&Edge
#include "explorer/local_coverage_planner.h"
#include "explorer/planning_env.h" // 占据栅格地图
#include "explorer/viewpoint_manager.h" // 对Viewpoint进行操作，基于lidar模型和周围环境以及grid_world

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

namespace sensor_coverage_planner_3d_ns
{
    const std::string kWorldFrameID = "map";
    typedef pcl::PointXYZRGBNormal PlannerCloudPointType; //
    typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
    typedef misc_utils_ns::Timer Timer; // 计时器

    // 探索规划器的参数对象
    struct PlannerParameters
    {
        // String
        std::string sub_start_exploration_topic_; //
        std::string sub_state_estimation_topic_; // 订阅状态估计
        std::string sub_registered_scan_topic_; // 订阅配准后的点云
        std::string sub_coverage_boundary_topic_;
        std::string sub_viewpoint_boundary_topic_;
        std::string sub_nogo_boundary_topic_;

        std::string pub_exploration_finish_topic_;
        std::string pub_runtime_breakdown_topic_;
        std::string pub_runtime_topic_;
        std::string pub_waypoint_topic_;
        std::string pub_momentum_activation_count_topic_;

        // Bool
        bool kAutoStart; // 是否自动开始探索
        bool kRushHome; // 是否返回
        bool kUseTerrainHeight; // 是否使用地形高度
        bool kCheckTerrainCollision; // 是否检测地形碰撞
        bool kExtendWayPoint; // 是否扩展航点
        bool kUseLineOfSightLookAheadPoint; // 是否使用视距航点
        bool kNoExplorationReturnHome; // 无探索时返回home点
        bool kUseMomentum; // 是否使用惯性

        // Double
        double kKeyposeCloudDwzFilterLeafSize; // keypose点云降采样大小
        double kRushHomeDist; // 回家距离
        double kAtHomeDistThreshold; // 回家距离阈值
        double kTerrainCollisionThreshold; // 地形碰撞检测阈值
        double kLookAheadDistance; // 视距航点距离
        double kExtendWayPointDistanceBig; // 航点最大扩展距离
        double kExtendWayPointDistanceSmall; // 航点最小扩展距离

        // Int
        int kDirectionChangeCounterThr; // 方向改变计数阈值
        int kDirectionNoChangeCounterThr; // 方向不改变计数阈值

        cv::Mat kTerrainCollisionMapSize; // 地形碰撞检测地图大小

        bool ReadParameters(ros::NodeHandle& nh);
    };

    // 探索过程中的数据管理器
    struct PlannerData
    {
        // PCL clouds TODO: keypose cloud does not need to be PlannerCloudPointType
        // 大多数点云用于可视化
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>
            keypose_cloud_; // keypose使用PointXYZRGBNormal
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>> registered_scan_stack_; // 配准后的扫描点云堆栈
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> registered_cloud_; // 配准后的点云

        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> large_terrain_cloud_; // 大地形图点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_collision_cloud_; // 地形碰撞检测点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
            terrain_ext_collision_cloud_; // 扩展地形碰撞检测点云

        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_vis_cloud_; // 视点可视化点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> grid_world_vis_cloud_; // 栅格世界可视化点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
            selected_viewpoint_vis_cloud_; // 被选择的视点可视化点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
            exploring_cell_vis_cloud_; // 正在探索的单元格可视化点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploration_path_cloud_; // 探索路径点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> collision_cloud_; // 碰撞检测点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> lookahead_point_cloud_; // 视距航点点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> keypose_graph_vis_cloud_; // keypose图可视化点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
            viewpoint_in_collision_cloud_; // 存在碰撞的视点点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> point_cloud_manager_neighbor_cloud_; // 邻近点云
        std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>
            reordered_global_subspace_cloud_; // 重新排序的子空间点云

        nav_msgs::Odometry keypose_; //
        geometry_msgs::Point robot_position_; // 机器人位置
        geometry_msgs::Point last_robot_position_; // 上一时刻机器人位置
        lidar_model_ns::LiDARModel robot_viewpoint_; // 机器人视点
        exploration_path_ns::ExplorationPath exploration_path_; // 探索路径
        Eigen::Vector3d lookahead_point_; // 前向点
        Eigen::Vector3d lookahead_point_direction_; // 前向点方向
        Eigen::Vector3d moving_direction_; // 机器人运动方向
        double robot_yaw_; // 机器人航向角
        bool moving_forward_; // 机器人是否向前运动
        std::vector<Eigen::Vector3d> visited_positions_; // 访问过的位置
        int cur_keypose_node_ind_; // 当前关键点节点索引
        Eigen::Vector3d initial_position_; // 初始位置

        std::unique_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_; // 关键位姿图
        std::unique_ptr<planning_env_ns::PlanningEnv> planning_env_; // 环境映射
        std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_; // 视点管理对象
        std::unique_ptr<local_coverage_planner_ns::LocalCoveragePlanner> local_coverage_planner_; // 局部覆盖规划器
        std::unique_ptr<grid_world_ns::GridWorld> grid_world_; // 网格世界
        std::unique_ptr<explorer_visualizer_ns::TAREVisualizer> visualizer_; // 可视化对象

        std::unique_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_; //
        std::unique_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_; //
        std::unique_ptr<misc_utils_ns::Marker> nogo_boundary_marker_; //
        std::unique_ptr<misc_utils_ns::Marker> grid_world_marker_; //
        void Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
    };

    class SensorCoveragePlanner3D
    {
    public:
        // explicit 关键字->创建对象时,必须显式调用构造函数
        explicit SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
        bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
        void execute(const ros::TimerEvent&);
        ~SensorCoveragePlanner3D() = default;

    private:
        bool keypose_cloud_update_;
        bool initialized_;
        bool lookahead_point_update_;
        bool relocation_;
        bool start_exploration_;
        bool exploration_finished_;
        bool near_home_;
        bool at_home_;
        bool stopped_;
        bool test_point_update_;
        bool viewpoint_ind_update_;
        bool step_;
        bool use_momentum_;
        bool lookahead_point_in_line_of_sight_;
        PlannerParameters pp_; // planner参数
        PlannerData pd_; // planner数据
        pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_; // 点云降采样

        int update_representation_runtime_; //
        int local_viewpoint_sampling_runtime_;
        int local_path_finding_runtime_;
        int global_planning_runtime_;
        int trajectory_optimization_runtime_;
        int overall_runtime_;
        int registered_cloud_count_;
        int keypose_count_;
        int direction_change_count_;
        int direction_no_change_count_;
        int momentum_activation_count_;

        ros::Time start_time_;
        ros::Time global_direction_switch_time_;

        ros::Timer execution_timer_;

        // ROS subscribers
        ros::Subscriber exploration_start_sub_;
        ros::Subscriber state_estimation_sub_;
        ros::Subscriber registered_scan_sub_;
        ros::Subscriber terrain_map_sub_;
        ros::Subscriber terrain_map_ext_sub_;
        ros::Subscriber coverage_boundary_sub_;
        ros::Subscriber viewpoint_boundary_sub_;
        ros::Subscriber nogo_boundary_sub_;

        // ROS publishers
        ros::Publisher global_path_full_publisher_;
        ros::Publisher global_path_publisher_;
        ros::Publisher old_global_path_publisher_;
        ros::Publisher to_nearest_global_subspace_path_publisher_;
        ros::Publisher local_tsp_path_publisher_;
        ros::Publisher exploration_path_publisher_;
        ros::Publisher waypoint_pub_;
        ros::Publisher exploration_finish_pub_;
        ros::Publisher runtime_breakdown_pub_;
        ros::Publisher runtime_pub_;
        ros::Publisher momentum_activation_count_pub_;
        // Debug
        ros::Publisher pointcloud_manager_neighbor_cells_origin_pub_;

        // Callback functions
        void ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg);
        void StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg);
        void RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_cloud_msg);
        void TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg);
        void TerrainMapExtCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_cloud_large_msg);
        void CoverageBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
        void ViewPointBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
        void NogoBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);

        void SendInitialWaypoint();
        void UpdateKeyposeGraph();
        int UpdateViewPoints();
        void UpdateViewPointCoverage();
        void UpdateRobotViewPointCoverage();
        void UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num);
        void UpdateVisitedPositions();
        void UpdateGlobalRepresentation();
        void GlobalPlanning(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path);
        void PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath& global_path,
                                                const exploration_path_ns::ExplorationPath& local_path);
        void LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                           const exploration_path_ns::ExplorationPath& global_path,
                           exploration_path_ns::ExplorationPath& local_path);
        void PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path);
        exploration_path_ns::ExplorationPath
        ConcatenateGlobalLocalPath(const exploration_path_ns::ExplorationPath& global_path,
                                   const exploration_path_ns::ExplorationPath& local_path);

        void PublishRuntime();
        double GetRobotToHomeDistance() const;
        void PublishExplorationState() const;
        static void PrintExplorationStatus(const std::string& status, bool clear_last_line);
        void PublishWaypoint();
        bool GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                               const exploration_path_ns::ExplorationPath& global_path,
                               Eigen::Vector3d& lookahead_point);

        void CountDirectionChange();
        static Eigen::Quaterniond GetQuaternionFromYaw(double yaw);
    };
} // namespace sensor_coverage_planner_3d_ns
