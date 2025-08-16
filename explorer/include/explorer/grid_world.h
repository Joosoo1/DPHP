/**
 * @file grid_world.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <vector>

#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "explorer/exploration_path.h"
#include "explorer/grid.h"
#include "explorer/keypose_graph.h"

namespace viewpoint_manager_ns
{
    class ViewPointManager;
}

namespace grid_world_ns
{
    // 每个cell都只有一种状态
    enum class CellStatus
    {
        UNSEEN = 0,
        EXPLORING = 1,
        COVERED = 2,
        COVERED_BY_OTHERS = 3,
        NOGO = 4,
        SEMI_EXPLORED = 5
    };

    // 子空间cell
    class Cell
    {
    public:
        explicit Cell(double x = 0.0, double y = 0.0, double z = 0.0);
        explicit Cell(const geometry_msgs::Point& center);
        // explicit会阻止隐式构造对象，比如Cell cell(1,2,3);会导致编译错误，只能显式构造：Cell cell = Cell(1,2,3) or
        // 使用默认参数；
        ~Cell() = default;
        bool IsCellConnected(int cell_ind);

        void AddViewPoint(const int viewpoint_ind) { viewpoint_indices_.push_back(viewpoint_ind); }

        void AddGraphNode(const int node_ind) { keypose_graph_node_indices_.push_back(node_ind); }

        void AddConnectedCell(const int cell_ind)
        {
            connected_cell_indices_.push_back(cell_ind);
            misc_utils_ns::UniquifyIntVector(connected_cell_indices_);
        }

        void ClearViewPointIndices() { viewpoint_indices_.clear(); }

        void ClearGraphNodeIndices() { keypose_graph_node_indices_.clear(); }

        void ClearConnectedCellIndices() { connected_cell_indices_.clear(); }

        CellStatus GetStatus() const { return status_; }

        void SetStatus(const CellStatus status) { status_ = status; }

        std::vector<int> GetViewPointIndices() { return viewpoint_indices_; }

        std::vector<int> GetConnectedCellIndices() { return connected_cell_indices_; }

        std::vector<int> GetGraphNodeIndices() { return keypose_graph_node_indices_; }

        geometry_msgs::Point GetPosition() const { return center_; }

        void SetPosition(const geometry_msgs::Point& position) { center_ = position; }

        void SetRobotPosition(const geometry_msgs::Point& robot_position)
        {
            robot_position_ = robot_position;
            robot_position_set_ = true;
        }

        void SetKeyposeID(int keypose_id)
        {
            keypose_id_ = keypose_id;
            robot_position_set_ = true;
        }

        bool IsRobotPositionSet() const { return robot_position_set_; }

        geometry_msgs::Point GetRobotPosition() const { return robot_position_; }

        void AddVisitCount() { visit_count_++; }

        int GetVisitCount() const { return visit_count_; }

        void Reset();

        int GetKeyposeID() const { return keypose_id_; }

        void SetViewPointPosition(const Eigen::Vector3d& position) { viewpoint_position_ = position; }

        Eigen::Vector3d GetViewPointPosition() { return viewpoint_position_; }

        void SetRoadmapConnectionPoint(const Eigen::Vector3d& roadmap_connection_point)
        {
            roadmap_connection_point_ = roadmap_connection_point;
        }

        Eigen::Vector3d GetRoadmapConnectionPoint() { return roadmap_connection_point_; }

        nav_msgs::Path GetPathToKeyposeGraph() { return path_to_keypose_graph_; }

        void SetPathToKeyposeGraph(const nav_msgs::Path& path) { path_to_keypose_graph_ = path; }

        bool IsPathAddedToKeyposeGraph() const { return path_added_to_keypose_graph_; }

        void SetPathAddedToKeyposeGraph(bool add_path) { path_added_to_keypose_graph_ = add_path; }

        //
        bool IsRoadmapConnectionPointSet() const { return roadmap_connection_point_set_; }

        void SetRoadmapConnectionPointSet(bool set) { roadmap_connection_point_set_ = set; }

    private:
        // 子空间状态
        CellStatus status_;
        // The center location of this cell.该cell的中心位置
        geometry_msgs::Point center_;
        // Position of the robot where this cell is first observed and turned EXPLORING
        // 机器人首次观察到这个cell并将其状态转为EXPLORING的位置
        geometry_msgs::Point robot_position_;
        // Whether the robot position has been set for this cell 是否已为这个cell设置了机器人的位置
        bool robot_position_set_;
        // Number of times the cell is visited by the robot机器人访问这个cell的次数
        int visit_count_;
        // Indices of the viewpoints within this cell.此cell内视点的索引
        std::vector<int> viewpoint_indices_;
        // Indices of other cells that are connected by a path. 由路径连接的其他cell的索引
        std::vector<int> connected_cell_indices_;
        // Indices of connected keypose graph nodes 连接的keypose graph的节点索引
        std::vector<int> keypose_graph_node_indices_;
        // Whether this cell is in the planning horizon, which consists of nine cells around the robot.
        // 这个cell是否在规划视野内，规划视野由机器人周围的九个单元格组成，机器人所在位置平面内最近的9个cell
        bool in_horizon_;
        // ID of the keypose where viewpoints in this cell can be observed 在该cell中可以观察到视点的keypose的ID
        int keypose_id_;
        // Position of the highest score viewpoint 该cell内得分最高的视点的位置
        Eigen::Vector3d viewpoint_position_;
        // Position for connecting the cell to the global roadmap 将该cell连接到全局roadmap的位置
        Eigen::Vector3d roadmap_connection_point_;
        // Path to the nearest keypose on the keypose graph 通往keypose graph上最近keypose的路径
        nav_msgs::Path path_to_keypose_graph_;
        // If the path has been added to the keypose graph 该路径是否已添加到keypose graph
        bool path_added_to_keypose_graph_;
        // If the roadmap connection point has been added to the cell 是否已将roadmap连接点添加到该cell
        bool roadmap_connection_point_set_;
    };

    // cell组成grid world
    class GridWorld
    {
    public:
        explicit GridWorld(ros::NodeHandle& nh);
        explicit GridWorld(int row_num = 1, int col_num = 1, int level_num = 1, double cell_size = 6.0,
                           double cell_height = 6.0, int nearby_grid_num = 5); // 两种构造方式->explicit拒绝隐式构造
        ~GridWorld() = default;
        // 从ros参数服务器中读取参数
        void ReadParameters(ros::NodeHandle& nh);
        // 更新robot的临近cells
        void UpdateNeighborCells(const geometry_msgs::Point& robot_position);
        // 根据robot所在的位置更新robot_cell_ind
        void UpdateRobotPosition(const geometry_msgs::Point& robot_position);
        // 更新状态为EXPLORING的cell的keypose_graph节点
        void UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph) const;

        int GetMinAddPointNum() const
        {
            return kMinAddPointNumSmall; // covered point最小阈值
        }

        int GetMinAddFrontierPointNum() const
        {
            return kMinAddFrontierPointNum; // frontier point最小阈值
        }

        // 获取全局栅格空间的原点
        geometry_msgs::Point GetOrigin() const
        {
            // return origin_;
            Eigen::Vector3d origin = subspaces_->GetOrigin(); // 子空间集合原点即为全局栅格空间原点
            geometry_msgs::Point geo_origin;
            geo_origin.x = origin.x();
            geo_origin.y = origin.y();
            geo_origin.z = origin.z();
            return geo_origin;
        }

        // 由栅格空间索引获取全局索引
        int sub2ind(const Eigen::Vector3i& sub) const { return subspaces_->Sub2Ind(sub); }

        // 由栅格空间索引获取全局索引
        int sub2ind(const int row_idx, const int col_idx, int level_idx) const
        {
            return subspaces_->Sub2Ind(row_idx, col_idx, level_idx);
        }

        // 由全局索引获取栅格空间索引
        Eigen::Vector3i ind2sub(const int ind) const { return subspaces_->Ind2Sub(ind); }

        // 由全局索引获取栅格空间索引(行、列、层)
        void ind2sub(const int ind, int& row_idx, int& col_idx, int& level_idx) const
        {
            Eigen::Vector3i sub = subspaces_->Ind2Sub(ind);
            row_idx = sub.x();
            col_idx = sub.y();
            level_idx = sub.z();
        }

        // 判断栅格空间索引是否在栅格空间内
        bool SubInBound(const Eigen::Vector3i& sub) const { return subspaces_->InRange(sub); }

        //  判断栅格空间索引是否在栅格空间内
        bool SubInBound(const int row_idx, const int col_idx, const int level_idx) const
        {
            return subspaces_->InRange(Eigen::Vector3i(row_idx, col_idx, level_idx));
        }

        //  判断全局索引是否在栅格空间内
        bool IndInBound(const int ind) const { return subspaces_->InRange(ind); }

        // Get the cell index where the robot is currently in.robot所处的的cell的全局索引
        bool AreNeighbors(int cell_ind1, int cell_ind2) const;
        // 根据坐标获取栅格空间全局索引
        int GetCellInd(double qx, double qy, double qz) const;
        // 根据坐标获取栅格空间三维索引
        void GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz) const;
        // 根据坐标获取栅格空间三维索引
        Eigen::Vector3i GetCellSub(const Eigen::Vector3d& point) const;
        // Get the visualization markers for Rviz display.
        void GetMarker(visualization_msgs::Marker& marker) const;
        // Get the visualization pointcloud for debugging purpose
        void GetVisualizationCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const;

        // 是否已初始化
        bool Initialized() const { return initialized_; }

        void SetUseKeyposeGraph(bool use_keypose_graph) { use_keypose_graph_ = use_keypose_graph; }

        bool UseKeyposeGraph() const { return use_keypose_graph_; }

        // 添加视点到cell，视点索引添加到cell的视点列表中
        void AddViewPointToCell(int cell_ind, int viewpoint_ind) const;
        // 添加keypose graph节点到cell，将keypose graph节点索引添加到对应cell的keypose graph节点列表中
        void AddGraphNodeToCell(int cell_ind, int node_ind) const;
        // 清除该cell的视点列表
        void ClearCellViewPointIndices(int cell_ind) const;
        // 获取该cell的视点列表
        std::vector<int> GetCellViewPointIndices(int cell_ind) const;
        // ？？
        std::vector<int> GetNeighborCellIndices() { return neighbor_cell_indices_; };
        // 获取该cell的临近cell列表
        void GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range);
        // 获取该位置的临近cell列表
        void GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range);
        // 获取状态为Exploring的cell列表
        void GetExploringCellIndices(std::vector<int>& exploring_cell_indices) const;
        // 获取该cell的状态
        CellStatus GetCellStatus(int cell_ind) const;
        // 设置该cell的状态
        void SetCellStatus(int cell_ind, CellStatus status) const;
        // 获取该cell的位置
        geometry_msgs::Point GetCellPosition(int cell_ind) const;
        // 设置该cell中的机器人位置
        void SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position) const;
        // 获取该cell中的机器人位置
        geometry_msgs::Point GetCellRobotPosition(int cell_ind) const;
        // 增加该cell的访问次数
        void CellAddVisitCount(int cell_ind) const;
        // 获取该cell的访问次数
        int GetCellVisitCount(int cell_ind) const;
        // 获取该cell是否已经设置机器人位置
        bool IsRobotPositionSet(int cell_ind) const;
        // 重置所有cell的状态、访问次数、机器人位置、视点列表、keypose graph节点列表、临近cell列表
        void Reset() const;
        // 获取处于该状态的cell数量
        int GetCellStatusCount(grid_world_ns::CellStatus status) const;
        // 根据视点及covered的point和frontier更新cell的状态
        void UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
        // 获取全局TSP路径
        exploration_path_ns::ExplorationPath
        SolveGlobalTSP(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                       std::vector<int>& ordered_cell_indices,
                       const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph = nullptr);

        // 设置当前keypose graph节点索引
        inline void SetCurKeyposeGraphNodeInd(int node_ind) { cur_keypose_graph_node_ind_ = node_ind; }

        // 设置当前keypose graph节点位置
        inline void SetCurKeyposeGraphNodePosition(geometry_msgs::Point node_position)
        {
            cur_keypose_graph_node_position_ = node_position;
        }

        // 设置当前keypose id
        inline void SetCurKeyposeID(int keypose_id) { cur_keypose_id_ = keypose_id; }

        // 设置当前keypose
        inline void SetCurKeypose(const Eigen::Vector3d& cur_keypose) { cur_keypose_ = cur_keypose; }

        // 获取当前cell的keypose节点索引
        int GetCellKeyposeID(int cell_ind) const;
        // 设置home位置
        void SetHomePosition(const Eigen::Vector3d& home_position)
        {
            home_position_ = home_position;
            set_home_ = true;
        }

        // home点是否已设置
        bool HomeSet() const { return set_home_; }

        // 是否正在返回home
        bool IsReturningHome() const { return return_home_; }

        // 获取不在临近且状态为EXPLORING的cell的的视点位置
        void GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions);
        // 添加cell之间的路径
        void AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                    const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph) const;

        // 检验路径是否有效
        bool PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind) const;

        // 在keypose_graph中能否搜索到从start_position到goal_position之间的路径，路径长度不超过max_path_length(两倍cell边长)
        bool HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                             const Eigen::Vector3d& start_position,
                                             const Eigen::Vector3d& goal_position) const;

        // 获取semi-explored的cell状态转换概率
        static double GetSemiCellTransitionProbability(const int& cell_ind);
        void getSemiExploredCellIndices(const std::vector<Eigen::Vector3d>& semi_dynamic_frontier_positions);

    private:
        // 3维栅格空间行列层数
        int kRowNum{};
        int kColNum{};
        int kLevelNum{};

        // cell的尺寸，长和宽为局部规划区域的1/5，高为自定义
        double kCellSize{};
        double kCellHeight{};

        // 邻近的cell范围，单位单元格个数
        int KNearbyGridNum{};
        // 视点最小covered的point数量小阈值
        int kMinAddPointNumSmall{};
        // 视点最小covered的point数量大阈值
        int kMinAddPointNumBig{};
        // 视点最小frontier数量阈值
        int kMinAddFrontierPointNum{};

        // cell的状态转换阈值->指的是cell中包含的有效的viewpoint数量，小于阈值则状态为COVERED
        int kCellExploringToCoveredThr{};
        int kCellCoveredToExploringThr{};
        int kCellExploringToAlmostCoveredThr{};
        int kCellAlmostCoveredToExploringThr{};
        int kCellUnknownToExploringThr{};
        // 这个是根据概率函数推算出来的概率值
        double kCellSemiExploredToExploringThr{};

        // std::vector<Cell> cells_;// gridworld单元格集合
        std::unique_ptr<grid_ns::Grid<Cell>> subspaces_; // 全局子空间栅格对象，子空间集合->在构造函数中初始化
        // 是否完成初始化
        bool initialized_;
        // 是否使用keypose graph
        bool use_keypose_graph_;
        // 当前keypose id
        int cur_keypose_id_{};
        // 机器人位置
        geometry_msgs::Point robot_position_;
        // 栅格世界原点
        geometry_msgs::Point origin_;
        // 临近cell列表
        std::vector<int> neighbor_cell_indices_;
        // 几乎covered的cell列表
        std::vector<int> almost_covered_cell_indices_;
        // semi-explored的cell列表
        std::vector<int> semi_explored_cell_indices_;
        std::vector<std::pair<int, int>> to_connect_cell_indices_;
        std::vector<nav_msgs::Path> to_connect_cell_paths_;
        // home点
        Eigen::Vector3d home_position_;
        // keypose
        Eigen::Vector3d cur_keypose_;
        // home是否已设置
        bool set_home_;
        // 是否正在返回home
        bool return_home_;
        // 当前keypose graph节点位置
        geometry_msgs::Point cur_keypose_graph_node_position_;
        // 当前keypose graph节点索引
        int cur_keypose_graph_node_ind_{};
        // 当前机器人所在cell索引
        int cur_robot_cell_ind_;
        // 之前机器人所在cell索引
        int prev_robot_cell_ind_;
    };
} // namespace grid_world_ns
