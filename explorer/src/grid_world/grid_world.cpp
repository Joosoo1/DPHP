/**
 * @file grid_world.cpp
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "../../include/grid_world/grid_world.h"
#include <algorithm>
#include <map>
#include <utils/misc_utils.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace grid_world_ns
{
    Cell::Cell(const double x, const double y, const double z) :
        robot_position_set_(false), visit_count_(0), in_horizon_(false), keypose_id_(0),
        viewpoint_position_(Eigen::Vector3d(x, y, z)), roadmap_connection_point_(Eigen::Vector3d(x, y, z)),
        path_added_to_keypose_graph_(false), roadmap_connection_point_set_(false)
    {
        center_.x = x;
        center_.y = y;
        center_.z = z;

        robot_position_.x = 0;
        robot_position_.y = 0;
        robot_position_.z = 0;
        status_ = CellStatus::UNSEEN;
    }

    Cell::Cell(const geometry_msgs::Point& center) : Cell(center.x, center.y, center.z) {}

    void Cell::Reset()
    {
        status_ = CellStatus::UNSEEN;
        robot_position_.x = 0;
        robot_position_.y = 0;
        robot_position_.z = 0;
        visit_count_ = 0;
        viewpoint_indices_.clear();
        connected_cell_indices_.clear();
        keypose_graph_node_indices_.clear();
    }

    bool Cell::IsCellConnected(const int cell_ind)
    {
        if (std::find(connected_cell_indices_.begin(), connected_cell_indices_.end(), cell_ind) !=
            connected_cell_indices_.end())
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    GridWorld::GridWorld(ros::NodeHandle& nh) : initialized_(false), use_keypose_graph_(false)
    {
        ReadParameters(nh);
        robot_position_.x = 0.0;
        robot_position_.y = 0.0;
        robot_position_.z = 0.0;

        origin_.x = 0.0;
        origin_.y = 0.0;
        origin_.z = 0.0;

        Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
        Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
        Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
        Cell cell_tmp;
        subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin,
                                                           grid_resolution); // 实例化子空间集合
        for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
        {
            subspaces_->GetCell(i) = Cell();
        }

        home_position_.x() = 0.0;
        home_position_.y() = 0.0;
        home_position_.z() = 0.0;

        cur_keypose_graph_node_position_.x = 0.0;
        cur_keypose_graph_node_position_.y = 0.0;
        cur_keypose_graph_node_position_.z = 0.0;

        set_home_ = false;
        return_home_ = false;

        cur_robot_cell_ind_ = -1;
        prev_robot_cell_ind_ = -1;
    }

    GridWorld::GridWorld(const int row_num, const int col_num, const int level_num, const double cell_size,
                         const double cell_height, const int nearby_grid_num) :
        kRowNum(row_num), kColNum(col_num), kLevelNum(level_num), kCellSize(cell_size), kCellHeight(cell_height),
        KNearbyGridNum(nearby_grid_num), kMinAddPointNumSmall(60), kMinAddPointNumBig(100), kMinAddFrontierPointNum(30),
        kCellExploringToCoveredThr(1), kCellCoveredToExploringThr(10), kCellExploringToAlmostCoveredThr(10),
        kCellAlmostCoveredToExploringThr(20), kCellUnknownToExploringThr(1), kCellSemiExploredToExploringThr(0.99),
        initialized_(false), use_keypose_graph_(false), cur_keypose_id_(0), cur_keypose_(0, 0, 0),
        cur_keypose_graph_node_ind_(0), cur_robot_cell_ind_(-1), prev_robot_cell_ind_(-1)
    {
        robot_position_.x = 0.0;
        robot_position_.y = 0.0;
        robot_position_.z = 0.0;

        origin_.x = 0.0;
        origin_.y = 0.0;
        origin_.z = 0.0;

        Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum); // 每个维度元素个数
        Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
        Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
        Cell cell_tmp;
        subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
        for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
        {
            subspaces_->GetCell(i) = grid_world_ns::Cell();
        }

        home_position_.x() = 0.0;
        home_position_.y() = 0.0;
        home_position_.z() = 0.0;

        cur_keypose_graph_node_position_.x = 0.0;
        cur_keypose_graph_node_position_.y = 0.0;
        cur_keypose_graph_node_position_.z = 0.0;

        set_home_ = false;
        return_home_ = false;
    }

    void GridWorld::ReadParameters(ros::NodeHandle& nh)
    {
        kRowNum = misc_utils_ns::getParam<int>(nh, "kGridWorldXNum", 121);
        kColNum = misc_utils_ns::getParam<int>(nh, "kGridWorldYNum", 121);
        kLevelNum = misc_utils_ns::getParam<int>(nh, "kGridWorldZNum",
                                                 121); // 划分的每个维度元素个数（121^3=1771561），即1771561个子空间

        const int viewpoint_number = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/number_x", 40);
        // 视点采样数量 number_x * number_y * number_z(40*40*1)
        const double viewpoint_resolution = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.0);
        // 视点采样间距？

        kCellSize = viewpoint_number * viewpoint_resolution / 5; // 子空间边长为局部规划范围的1/5
        kCellHeight = misc_utils_ns::getParam<double>(nh, "kGridWorldCellHeight", 8.0); // 每个子空间的高度

        KNearbyGridNum =
            misc_utils_ns::getParam<int>(nh, "kGridWorldNearbyGridNum", 5); // x、y方向上的邻居子空间数量，z轴默认为1

        kMinAddPointNumSmall = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
        kMinAddPointNumBig = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumBig", 100);
        kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);

        // 状态转换阈值
        kCellExploringToCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToCoveredThr", 1);
        kCellCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellCoveredToExploringThr", 10);
        kCellExploringToAlmostCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToAlmostCoveredThr", 10);
        kCellAlmostCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellAlmostCoveredToExploringThr", 20);
        kCellUnknownToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellUnknownToExploringThr", 1);
        kCellSemiExploredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellSemiExploredToExploringThr", 0.99);
    }

    void GridWorld::UpdateNeighborCells(const geometry_msgs::Point& robot_position)
    {
        if (!initialized_)
        {
            // 初始化：设置栅格世界原点的原点位置(在世界坐标下)，设为以机器人起始位置为中心，左下角即为原点
            initialized_ = true;
            //
            origin_.x = robot_position.x - (kCellSize * kRowNum) / 2;
            origin_.y = robot_position.y - (kCellSize * kColNum) / 2;
            origin_.z = robot_position.z - (kCellHeight * kLevelNum) / 2;
            subspaces_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
            // 初始化设置每个cell的位置坐标为其中心点位置--->世界坐标系下
            for (int i = 0; i < kRowNum; i++)
            {
                for (int j = 0; j < kColNum; j++)
                {
                    for (int k = 0; k < kLevelNum; k++)
                    {
                        Eigen::Vector3d subspace_center_position = subspaces_->Sub2Pos(i, j, k);
                        geometry_msgs::Point subspace_center_geo_position;
                        subspace_center_geo_position.x = subspace_center_position.x();
                        subspace_center_geo_position.y = subspace_center_position.y();
                        subspace_center_geo_position.z = subspace_center_position.z();
                        subspaces_->GetCell(i, j, k).SetPosition(subspace_center_geo_position);
                        subspaces_->GetCell(i, j, k).SetRoadmapConnectionPoint(subspace_center_position);
                    }
                }
            }
        }

        // Get neighbor cells
        std::vector<int> prev_neighbor_cell_indices = neighbor_cell_indices_;
        neighbor_cell_indices_.clear();
        int N = KNearbyGridNum / 2; // 5/2=2.5，取整为2，xy平面临近的cell范围
        int M = 1;
        // 获取当前robot所在的cell的临近cell的索引列表-->75
        GetNeighborCellIndices(robot_position, Eigen::Vector3i(N, N, M));

        //
        for (const auto& cell_ind : neighbor_cell_indices_)
        {
            // 遍历新的邻近cell列表，对于每一个新的临近cell，如果它在之前的临近cell列表中不存在，则增加其访问次数
            if (std::find(prev_neighbor_cell_indices.begin(), prev_neighbor_cell_indices.end(), cell_ind) ==
                prev_neighbor_cell_indices.end())
            {
                subspaces_->GetCell(cell_ind).AddVisitCount();
            }
        }
    }

    void GridWorld::UpdateRobotPosition(const geometry_msgs::Point& robot_position)
    {
        robot_position_ = robot_position;
        // 根据robot所在的位置更新robot_cell_ind
        int robot_cell_ind = GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
        if (cur_robot_cell_ind_ != robot_cell_ind)
        {
            prev_robot_cell_ind_ = cur_robot_cell_ind_;
            cur_robot_cell_ind_ = robot_cell_ind;
        }
    }

    // 更新类型为EXPLORING的cell的keypose_graph节点
    void
    GridWorld::UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph) const
    {
        std::vector<int> keypose_graph_connected_node_indices = keypose_graph->GetConnectedGraphNodeIndices();
        // 获取可连接的keypose节点索引

        // 遍历每个cell，如果cell的状态为EXPLORING，则清除该cell的keypose节点索引
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
            {
                subspaces_->GetCell(i).ClearGraphNodeIndices();
            }
        }

        // 遍历keypose_graph的每个节点，如果该节点对应的cell的状态为EXPLORING/SEMI_EXPLORING，则将该节点添加到该cell的keypose节点索引中
        for (const auto& node_ind : keypose_graph_connected_node_indices)
        {
            geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind); // 获取节点位置
            int cell_ind =
                GetCellInd(node_position.x, node_position.y, node_position.z); // 获取该节点位置所对应的cell索引
            if (subspaces_->InRange(cell_ind))
            {
                if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
                {
                    subspaces_->GetCell(cell_ind).AddGraphNode(node_ind); //
                }
            }
        }
    }

    // 判断cell1与cell2是否为邻居
    bool GridWorld::AreNeighbors(const int cell_ind1, const int cell_ind2) const
    {
        Eigen::Vector3i cell_sub1 = subspaces_->Ind2Sub(cell_ind1);
        Eigen::Vector3i cell_sub2 = subspaces_->Ind2Sub(cell_ind2);
        Eigen::Vector3i diff = cell_sub1 - cell_sub2;
        if (std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z()) == 1) // ？
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    int GridWorld::GetCellInd(const double qx, const double qy, const double qz) const
    {
        const Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
        if (subspaces_->InRange(sub))
        {
            return subspaces_->Sub2Ind(sub);
        }
        else
        {
            return -1;
        }
    }

    void GridWorld::GetCellSub(int& row_idx, int& col_idx, int& level_idx, const double qx, const double qy,
                               const double qz) const
    {
        Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
        row_idx = (sub.x() >= 0 && sub.x() < kRowNum) ? sub.x() : -1;
        col_idx = (sub.y() >= 0 && sub.y() < kColNum) ? sub.y() : -1;
        level_idx = (sub.z() >= 0 && sub.z() < kLevelNum) ? sub.z() : -1;
    }

    Eigen::Vector3i GridWorld::GetCellSub(const Eigen::Vector3d& point) const { return subspaces_->Pos2Sub(point); }

    void GridWorld::GetMarker(visualization_msgs::Marker& marker) const
    {
        marker.points.clear();
        marker.colors.clear();
        marker.scale.x = kCellSize;
        marker.scale.y = kCellSize;
        marker.scale.z = kCellHeight;

        int exploring_count = 0;
        int covered_count = 0;
        int unseen_count = 0;

        for (int i = 0; i < kRowNum; i++)
        {
            for (int j = 0; j < kColNum; j++)
            {
                for (int k = 0; k < kLevelNum; k++)
                {
                    int cell_ind = subspaces_->Sub2Ind(i, j, k);
                    geometry_msgs::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
                    std_msgs::ColorRGBA color;
                    bool add_marker = false;
                    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
                    {
                        color.r = 0.0;
                        color.g = 0.0;
                        color.b = 1.0;
                        color.a = 0.1; // a表示透明度
                        unseen_count++;
                    }
                    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
                    {
                        color.r = 1.0;
                        color.g = 1.0;
                        color.b = 0.0;
                        color.a = 0.1;
                        covered_count++;
                    }
                    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
                    {
                        color.r = 0.0;
                        color.g = 1.0;
                        color.b = 0.0;
                        color.a = 0.1;
                        exploring_count++;
                        add_marker = true;
                    }
                    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::NOGO)
                    {
                        color.r = 1.0;
                        color.g = 0.0;
                        color.b = 0.0;
                        color.a = 0.1;
                    }
                    else
                    {
                        color.r = 0.8;
                        color.g = 0.8;
                        color.b = 0.8;
                        color.a = 0.1;
                    }
                    if (add_marker)
                    {
                        marker.colors.push_back(color);
                        marker.points.push_back(cell_center);
                    }
                }
            }
        }

        // Color neighbor cells differently
        // for(const auto & ind : neighbor_cell_indices_){
        //     if(cells_[ind].GetStatus() == CellStatus::UNSEEN) continue;
        //     marker.colors[ind].a = 0.8;
        // }
    }

    void GridWorld::GetVisualizationCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const
    {
        vis_cloud->points.clear();
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            CellStatus cell_status = subspaces_->GetCell(i).GetStatus();
            if (!subspaces_->GetCell(i).GetConnectedCellIndices().empty())
            {
                pcl::PointXYZI point;
                Eigen::Vector3d position = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
                point.x = position.x();
                point.y = position.y();
                point.z = position.z();
                point.intensity = i;
                vis_cloud->points.push_back(point);
            }
        }
    }

    void GridWorld::AddViewPointToCell(const int cell_ind, const int viewpoint_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        subspaces_->GetCell(cell_ind).AddViewPoint(viewpoint_ind);
    }

    void GridWorld::AddGraphNodeToCell(const int cell_ind, const int node_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
    }

    void GridWorld::ClearCellViewPointIndices(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        subspaces_->GetCell(cell_ind).ClearViewPointIndices();
    }

    std::vector<int> GridWorld::GetCellViewPointIndices(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).GetViewPointIndices();
    }

    // 该函数用于获取给定中心网格单元周围邻居网格单元的索引，并将这些索引存储在一个向量中
    void GridWorld::GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub,
                                           const Eigen::Vector3i& neighbor_range)
    {
        int row_idx = 0;
        int col_idx = 0;
        int level_idx = 0;
        // 计数
        int count = 0;
        for (int i = -neighbor_range.x(); i <= neighbor_range.x(); i++)
        {
            for (int j = -neighbor_range.y(); j <= neighbor_range.y(); j++)
            {
                row_idx = center_cell_sub.x() + i;
                col_idx = center_cell_sub.y() + j;

                for (int k = -neighbor_range.z(); k <= neighbor_range.z(); k++)
                {
                    level_idx = center_cell_sub.z() + k;
                    Eigen::Vector3i sub(row_idx, col_idx, level_idx);
                    if (subspaces_->InRange(sub))
                    {
                        int ind = subspaces_->Sub2Ind(sub);
                        neighbor_cell_indices_.push_back(ind);
                        count++;
                    }
                }
            }
        }
        // std::cout << "[grid_world]: neighbor cell count: " << count << std::endl;
    }

    void GridWorld::GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range)
    {
        const Eigen::Vector3i center_cell_sub = GetCellSub(Eigen::Vector3d(position.x, position.y, position.z));

        GetNeighborCellIndices(center_cell_sub, neighbor_range);
    }

    // 获取GridWorld中所有处于探索状态（CellStatus::EXPLORING）的网格单元的索引，
    // 并将这些索引存储在一个名为exploring_cell_indices的向量中
    void GridWorld::GetExploringCellIndices(std::vector<int>& exploring_cell_indices) const
    {
        exploring_cell_indices.clear();
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
            {
                exploring_cell_indices.push_back(i);
            }
        }
    }

    CellStatus GridWorld::GetCellStatus(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).GetStatus();
    }

    void GridWorld::SetCellStatus(const int cell_ind, const CellStatus status) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        subspaces_->GetCell(cell_ind).SetStatus(status);
    }

    geometry_msgs::Point GridWorld::GetCellPosition(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).GetPosition();
    }

    void GridWorld::SetCellRobotPosition(const int cell_ind, const geometry_msgs::Point& robot_position) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position);
    }

    geometry_msgs::Point GridWorld::GetCellRobotPosition(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).GetRobotPosition();
    }

    void GridWorld::CellAddVisitCount(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        subspaces_->GetCell(cell_ind).AddVisitCount();
    }

    int GridWorld::GetCellVisitCount(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).GetVisitCount();
    }

    bool GridWorld::IsRobotPositionSet(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).IsRobotPositionSet();
    }

    void GridWorld::Reset() const
    {
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            subspaces_->GetCell(i).Reset();
        }
    }

    int GridWorld::GetCellStatusCount(const CellStatus status) const
    {
        int count = 0;
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            if (subspaces_->GetCell(i).GetStatus() == status)
            {
                count++;
            }
        }
        return count;
    }

    // 获取
    void GridWorld::getSemiExploredCellindices(const std::vector<Eigen::Vector3d>& semi_dynamic_frontier_positions)
    {
        int index = 0;
        for (const auto& position : semi_dynamic_frontier_positions)
        {
            index = subspaces_->Pos2Ind(position);
            if (subspaces_->InRange(index))
            {
                subspaces_->GetCell(index).SetStatus(CellStatus::SEMI_EXPLOERED);
                semi_explored_cell_indices_.push_back(index);
            }
        }
    }

    void GridWorld::UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
    {
        int exploring_count = 0;
        int unseen_count = 0;
        int covered_count = 0;
        // add semi-explored cells
        int semi_explored_count = 0;
        for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
        {
            if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING) // exploring
            {
                exploring_count++;
            }
            else if (subspaces_->GetCell(i).GetStatus() == CellStatus::UNSEEN) // unexplored
            {
                unseen_count++;
            }
            else if (subspaces_->GetCell(i).GetStatus() == CellStatus::COVERED) // explored
            {
                covered_count++;
            }
            else if (subspaces_->GetCell(i).GetStatus() == CellStatus::SEMI_EXPLOERED) // semi-explored
            {
                semi_explored_count++;
            }
        }

        // TODO: joosoo@ add semi-dynamic frontier,用于semi-explored cell改变

        for (const auto& cell_ind : neighbor_cell_indices_)
        {
            subspaces_->GetCell(cell_ind).ClearViewPointIndices(); // 清空邻近cell中的Viewpoint索引
        }

        // 从viewpoint manager中获取Viewpoint添加到对应的cell中
        for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
        {
            // 获取Viewpoint的位置
            geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
            Eigen::Vector3i sub =
                subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
            // 返回该Viewpoint对应子空间3维索引坐标
            if (subspaces_->InRange(sub)) // 判断该子空间是否在规定范围内
            {
                int cell_ind = subspaces_->Sub2Ind(sub); // 获取该子空间的全局索引
                AddViewPointToCell(cell_ind, viewpoint_ind); // 将该Viewpoint添加到该cell中
                viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind); // 设置该Viewpoint对应的cell索引
            }
            else
            {
                ROS_ERROR_STREAM("subspace sub out of bound: " << sub.transpose());
            }
        }

        // 遍历临近的所有cell
        for (const auto& cell_ind : neighbor_cell_indices_)
        {
            if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)
            {
                continue;
            }

            int candidate_count = 0;
            int selected_viewpoint_count = 0;
            int above_big_threshold_count = 0;
            int above_small_threshold_count = 0;
            int above_frontier_threshold_count = 0;

            int highest_score_viewpoint_ind = -1;
            int highest_score = -1;

            for (const auto& viewpoint_ind :
                 subspaces_->GetCell(cell_ind).GetViewPointIndices()) // 遍历邻近cell中的所有Viewpoint
            {
                MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind));
                candidate_count++; // 候选视点数量
                if (viewpoint_manager->ViewPointSelected(viewpoint_ind)) // 如果该Viewpoint被选中
                {
                    selected_viewpoint_count++; // Viewpoint被选中数量+1
                }

                if (viewpoint_manager->ViewPointVisited(viewpoint_ind)) // 如果该Viewpoint被访问过
                {
                    continue; // 直接跳过
                }

                int score = viewpoint_manager->GetViewPointCoveredPointNum(
                    viewpoint_ind); // 获取该Viewpoint覆盖的surface-->reward
                int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(viewpoint_ind);
                // 获取该Viewpoint覆盖的边界点数量-->边界reward

                // 找到当前cell中reward最高的Viewpoint及其reward值
                if (score > highest_score)
                {
                    highest_score = score;
                    highest_score_viewpoint_ind = viewpoint_ind;
                }

                // 找到当前cell中point reward大于小最小阈值的Viewpoint数量
                if (score > kMinAddPointNumSmall)
                {
                    above_small_threshold_count++;
                }

                // 找到当前cell中point reward大于大最小阈值的Viewpoint数量
                if (score > kMinAddPointNumBig)
                {
                    above_big_threshold_count++;
                }

                // 找到当前cell中边界reward大于边界最小阈值的Viewpoint数量
                if (frontier_score > kMinAddFrontierPointNum)
                {
                    above_frontier_threshold_count++;
                }
            }

            /*cell的状态转换条件*/
            // Exploring to Covered
            if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
                above_frontier_threshold_count < kCellExploringToCoveredThr &&
                above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
                candidate_count > 0)
            {
                subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED); //
            }
            // Covered to Exploring
            else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED &&
                     (above_big_threshold_count >= kCellCoveredToExploringThr ||
                      above_frontier_threshold_count >= kCellCoveredToExploringThr))
            {
                subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
                almost_covered_cell_indices_.push_back(cell_ind);
            }
            // Exploring to Almost cover
            else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
                     selected_viewpoint_count == 0 && candidate_count > 0)
            {
                almost_covered_cell_indices_.push_back(cell_ind);
            }
            // Unseen to Exploring
            else if (subspaces_->GetCell(cell_ind).GetStatus() != CellStatus::COVERED && selected_viewpoint_count > 0)
            {
                subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
                almost_covered_cell_indices_.erase(
                    std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
                    almost_covered_cell_indices_.end());
            }
            else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count == 0)
            {
                // First visit
                if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
                    subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
                {
                    subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
                }
                else
                {
                    geometry_msgs::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
                    double xy_dist_to_robot = misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
                        cell_position, robot_position_);
                    // 水平欧式距离
                    double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z); // 垂直欧式距离
                    if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
                    {
                        subspaces_->GetCell(cell_ind).SetStatus(
                            CellStatus::COVERED); // 如果该cell在机器人附近，则设置为COVERED
                    }
                }
            }
            // 当局部区域存在semi-explored子空间时，将其转化为exploring状态
            else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::SEMI_EXPLOERED) // todo
            {
                subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
            }

            // 如果该cell处于Exploring状态，并且有候选视点，则设置该cell的robot position和keypose id
            if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count > 0)
            {
                subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position_);
                subspaces_->GetCell(cell_ind).SetKeyposeID(cur_keypose_id_);
            }
        }

        // 维护一个semi-explored状态的cell列表，按照频次回调更新其状态转换概率？如下：semi->exploring, exploring->semi?
        for (const auto& cell_ind : semi_explored_cell_indices_)
        {
            // todo,获取状态转换概率
            if (GetSemiCellTransitionProbability(cell_ind) > kCellSemiExploredToExploringThr)
            {
                subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
                semi_explored_cell_indices_.erase(
                    std::remove(semi_explored_cell_indices_.begin(), semi_explored_cell_indices_.end(), cell_ind),
                    semi_explored_cell_indices_.end());
            }
        }

        // 对于处于Almost covered状态的cell，如果该cell未在neighbor_cell_indices_中，则设置为Covered
        for (const auto& cell_ind : almost_covered_cell_indices_)
        {
            if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), cell_ind) ==
                neighbor_cell_indices_.end())
            {
                subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
                // 移除状态已经改变的cell
                almost_covered_cell_indices_.erase(std::remove(almost_covered_cell_indices_.begin(),
                                                               almost_covered_cell_indices_.end(),
                                                               cell_ind), // 移到末尾
                                                   almost_covered_cell_indices_.end()); // 抹除最后的元素
            }
        }
    }

    // todo：实现转换函数,求取转换概率
    double GridWorld::GetSemiCellTransitionProbability(const int& cell_ind) { return 0.0; }

    // 求解全局引导路径
    exploration_path_ns::ExplorationPath
    GridWorld::SolveGlobalTSP(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                              std::vector<int>& ordered_cell_indices,
                              const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
    {
        /****** Get the node on keypose graph associated with the robot position *****/
        /*step1: 获取与机器人位置相关联的keypose_graph中的节点*/
        auto min_dist_to_robot = DBL_MAX;
        geometry_msgs::Point global_path_robot_position = robot_position_;
        Eigen::Vector3d eigen_robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
        // Get nearest connected node
        int closest_node_ind = 0;
        auto closest_node_dist = DBL_MAX;
        // 获取keypose_graph中与机器人位置最接近的节点索引和距离
        keypose_graph->GetClosestConnectedNodeIndAndDistance(robot_position_, closest_node_ind, closest_node_dist);
        if (closest_node_dist < kCellSize / 2 && closest_node_ind >= 0 &&
            closest_node_ind < keypose_graph->GetNodeNum())
        {
            global_path_robot_position = keypose_graph->GetNodePosition(closest_node_ind); // 以此节点作为机器人位置
        }
        else if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeNum())
        {
            // ROS_WARN("GridWorld::SolveGlobalTSP: using nearest keypose node for robot position");
            global_path_robot_position = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_); // 同上
        }
        else // 如果未能找到满足要求的最接近的节点
        {
            // ROS_WARN("GridWorld::SolveGlobalTSP: using neighbor cell roadmap connection points for robot position");
            // 遍历临近cell与roadmap的连接点，找到一个最接近机器人位置的连接点，做为全局tsp的机器人位置
            for (int cell_ind : neighbor_cell_indices_)
            {
                if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
                {
                    Eigen::Vector3d roadmap_connection_point =
                        subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
                    if (viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point))
                    {
                        double dist_to_robot = (roadmap_connection_point - eigen_robot_position).norm();
                        if (dist_to_robot < min_dist_to_robot)
                        {
                            min_dist_to_robot = dist_to_robot;
                            global_path_robot_position.x = roadmap_connection_point.x();
                            global_path_robot_position.y = roadmap_connection_point.y();
                            global_path_robot_position.z = roadmap_connection_point.z(); //
                        }
                    }
                }
            }
        }

        /****** Get all the connected exploring cells *****/
        /*step2：获取所有连接的探索ing cells*/
        exploration_path_ns::ExplorationPath global_path;
        std::vector<geometry_msgs::Point> exploring_cell_positions;
        std::vector<int> exploring_cell_indices;
        // 遍历所有子空间的cell
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            // 针对exploring状态的cell
            if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
            {
                // 如果该cell不在neighbor_cell_indices_中，或者该cell的viewpoint_indices为空且visit_count大于1
                if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) ==
                        neighbor_cell_indices_.end() ||
                    (subspaces_->GetCell(i).GetViewPointIndices().empty() &&
                     subspaces_->GetCell(i).GetVisitCount() > 1))
                {
                    if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
                    {
                        // 如果keypose_graph为空，则直接使用直线连接
                        // Use straight line connection
                        exploring_cell_positions.push_back(GetCellPosition(i)); //
                        exploring_cell_indices.push_back(i);
                    }
                    else // 如果keypose_graph不为空
                    {
                        Eigen::Vector3d connection_point =
                            subspaces_->GetCell(i).GetRoadmapConnectionPoint(); // 获取连接点
                        geometry_msgs::Point connection_point_geo;
                        connection_point_geo.x = connection_point.x();
                        connection_point_geo.y = connection_point.y();
                        connection_point_geo.z = connection_point.z(); // 将连接点转成geometry_msgs::Point类型

                        // 检查该连接点的可达性
                        bool reachable = false;
                        if (keypose_graph->IsPositionReachable(connection_point_geo))
                        {
                            reachable = true;
                        }
                        else // 如果不可达，则遍历该cell的keypose_graph节点，找到最接近连接点的节点
                        {
                            // Check all the keypose graph nodes within this cell to see if there are any connected
                            // nodes
                            auto min_dist = DBL_MAX;
                            double min_dist_node_ind = -1;
                            for (const auto& node_ind : subspaces_->GetCell(i).GetGraphNodeIndices())
                            {
                                geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
                                double dist = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                                    node_position, connection_point_geo);
                                if (dist < min_dist)
                                {
                                    min_dist = dist;
                                    min_dist_node_ind = node_ind;
                                }
                            }
                            // 检测节点有效性，若有效则对连接点进行替换
                            if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
                            {
                                reachable = true;
                                connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
                            }
                        }

                        if (reachable)
                        {
                            exploring_cell_positions.push_back(connection_point_geo);
                            exploring_cell_indices.push_back(
                                i); // 将连接点加入exploring_cell_positions和exploring_cell_indices中
                        }
                    }
                }
            }
        }

        /****** Return home ******/
        /*如果exploring的cell已经没有，则进入return home模式*/
        if (exploring_cell_indices.empty())
        {
            return_home_ = true;

            geometry_msgs::Point home_position;

            nav_msgs::Path return_home_path;
            // 如果keypose_graph为空，则直接使用直线连接
            if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
            {
                geometry_msgs::PoseStamped robot_pose;
                robot_pose.pose.position = robot_position_;

                geometry_msgs::PoseStamped home_pose;
                home_pose.pose.position = home_position;

                return_home_path.poses.push_back(robot_pose);
                return_home_path.poses.push_back(home_pose);
            }
            else // 如果keypose_graph不为空
            {
                // 则home_position为keypose_graph的第一个节点的位置
                home_position = keypose_graph->GetFirstKeyposePosition();
                // 由机器人当前位置和home_position计算最短路径
                keypose_graph->GetShortestPath(global_path_robot_position, home_position, true, return_home_path,
                                               false);
                //
                if (return_home_path.poses.size() >= 2)
                {
                    global_path.FromPath(return_home_path);
                    // 将return_home_path(nav_msgs::Path)转换为exploration_path_ns::ExplorationPath类型
                    global_path.nodes_.front().type_ =
                        exploration_path_ns::NodeType::ROBOT; // 将第一个路径点设置为ROBOT类型

                    for (int i = 1; i < global_path.nodes_.size() - 1; i++)
                    {
                        global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_VIA_POINT;
                        // 将中间路径点设置为GLOBAL_VIA_POINT类型
                    }
                    global_path.nodes_.back().type_ =
                        exploration_path_ns::NodeType::HOME; // 将最后一个路径点设置为HOME类型

                    // Make it a loop， 形成双向循环路径？
                    for (int i = global_path.nodes_.size() - 2; i >= 0; i--)
                    {
                        global_path.Append(global_path.nodes_[i]);
                    }
                }
                else
                {
                    // ROS_ERROR("Cannot find path home");
                    // TODO: find a path
                }
            }
            return global_path; // 返回全局路径--->return home模式
        }

        return_home_ = false; // return home模式为false

        // Put the current robot position in the end
        exploring_cell_positions.push_back(
            global_path_robot_position); // 将当前机器人位置加入exploring_cell_positions的末尾
        exploring_cell_indices.push_back(-1); // -1的索引表示当前机器人

        /******* Construct the distance matrix *****/
        std::vector<std::vector<int>> distance_matrix(
            exploring_cell_positions.size(),
            std::vector<int>(exploring_cell_positions.size(), 0)); // 初始化距离矩阵为0矩阵
        for (int i = 0; i < exploring_cell_positions.size(); i++) // 遍历exploring_cell_positions的所有位置点
        {
            for (int j = 0; j < i; j++)
            {
                // 如果未启用keypose_graph，则使用直线连接，表示两点之间的距离
                if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
                {
                    // Use straight line connection直线连接两点-->欧式距离
                    distance_matrix[i][j] =
                        static_cast<int>(10 *
                                         misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                                             exploring_cell_positions[i], exploring_cell_positions[j]));
                }
                else // 若存在keypose_graph，则求取在keypose_graph中两点之间的最短距离
                {
                    // Use keypose graph
                    nav_msgs::Path path_tmp;
                    //
                    distance_matrix[i][j] =
                        static_cast<int>(10 *
                                         keypose_graph->GetShortestPath(exploring_cell_positions[i],
                                                                        exploring_cell_positions[j], false, path_tmp,
                                                                        false)); // 获取最短路径*10
                }
            }
        }
        // 填充距离矩阵的对称部分
        for (int i = 0; i < exploring_cell_positions.size(); i++)
        {
            for (int j = i + 1; j < exploring_cell_positions.size(); j++)
            {
                distance_matrix[i][j] = distance_matrix[j][i];
            }
        }

        /****** Solve the TSP ******/
        tsp_solver_ns::DataModel data_model;
        data_model.distance_matrix = distance_matrix;
        data_model.depot = exploring_cell_positions.size() - 1; // 填充tsp

        tsp_solver_ns::TSPSolver tsp_solver(data_model); // 实例化tsp_solver
        tsp_solver.Solve(); // 求解tsp
        std::vector<int> node_index;
        tsp_solver.getSolutionNodeIndex(node_index, false); // 获取解，存储到node_index中

        ordered_cell_indices.clear(); // 清空之前的ordered_cell_indices

        // Add the first node in the end to make it a loop
        if (!node_index.empty())
        {
            node_index.push_back(node_index[0]); // 将第一个节点加入node_index的末尾，loop ？
        }

        if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
        {
            // 直接直线连接
            for (int cell_ind : node_index)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position = exploring_cell_positions[cell_ind];
                exploration_path_ns::Node node(exploring_cell_positions[cell_ind],
                                               exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);
                node.global_subspace_index_ = exploring_cell_indices[cell_ind];
                global_path.Append(node); // 全局路径
                ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]); // 每个cell的访问顺序
            }
        }
        else
        {
            geometry_msgs::Point cur_position;
            geometry_msgs::Point next_position;
            int cur_keypose_id;
            int next_keypose_id;
            int cur_ind;
            int next_ind;

            // 填充全局路径
            for (int i = 0; i < node_index.size() - 1; i++)
            {
                cur_ind = node_index[i];
                next_ind = node_index[i + 1];
                cur_position = exploring_cell_positions[cur_ind];
                next_position = exploring_cell_positions[next_ind];

                nav_msgs::Path keypose_path;
                keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, false); // 返回分段路径

                exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));

                if (i == 0)
                {
                    node.type_ = exploration_path_ns::NodeType::ROBOT;
                }
                else
                {
                    node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
                }

                node.global_subspace_index_ = exploring_cell_indices[cur_ind]; // 该node的子空间索引
                global_path.Append(node); // 将此个节点加入全局路径

                ordered_cell_indices.push_back(exploring_cell_indices[cur_ind]); // 每个cell的访问顺序

                // Fill in the path in between
                if (keypose_path.poses.size() >= 2)
                {
                    for (int j = 1; j < keypose_path.poses.size() - 1; j++)
                    {
                        geometry_msgs::Point node_position;
                        node_position = keypose_path.poses[j].pose.position;
                        exploration_path_ns::Node keypose_node(node_position,
                                                               exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
                        keypose_node.keypose_graph_node_ind_ =
                            static_cast<int>(keypose_path.poses[i].pose.orientation.x);
                        global_path.Append(keypose_node); // 将两个GLOBAL_VIEWPOINT之间的路径添加到全局路径中
                    }
                }
            }

            // Append the robot node to the end，loop
            if (!global_path.nodes_.empty())
            {
                global_path.Append(global_path.nodes_[0]);
            }
        }

        return global_path; // 返回全局路径
    }

    int GridWorld::GetCellKeyposeID(const int cell_ind) const
    {
        MY_ASSERT(subspaces_->InRange(cell_ind));
        return subspaces_->GetCell(cell_ind).GetKeyposeID();
    }

    // 获取不在邻近且状态为EXPLORING的cell的的视点位置
    void GridWorld::GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions)
    {
        viewpoint_positions.clear();
        for (int i = 0; i < subspaces_->GetCellNumber(); i++)
        {
            if (subspaces_->GetCell(i).GetStatus() != grid_world_ns::CellStatus::EXPLORING)
            {
                continue;
            }
            if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) ==
                neighbor_cell_indices_.end())
            {
                viewpoint_positions.push_back(subspaces_->GetCell(i).GetViewPointPosition());
            }
        }
    }

    void
    GridWorld::AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                      const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph) const
    {
        // Determine the connection point in each cell确定每个cell的连接点
        // 遍历每个邻近cell，去除存在碰撞的连接点，并寻找该cell中离其中心最近的viewpoint,
        // 将该点作为与全局roadmap的连接点
        for (int cell_ind : neighbor_cell_indices_)
        {
            // 获取cell索引
            // 如果cell已经设置与roadmap的连接点，且连接点
            // 不在局部规划区域或者该连接点存在碰撞，则清除该cell连接的其它cell索引
            if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
            {
                if (viewpoint_manager->InLocalPlanningHorizon(
                        subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()) &&
                    !viewpoint_manager->InCollision(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()))
                {
                    continue;
                }
                else
                {
                    subspaces_->GetCell(cell_ind).ClearConnectedCellIndices();
                }
            }
            // 获取该cell内的候选viewpoint索引
            std::vector<int> candidate_viewpoint_indices = subspaces_->GetCell(cell_ind).GetViewPointIndices();
            // 如果该cell内的视点不为空
            if (!candidate_viewpoint_indices.empty())
            {
                auto min_dist = DBL_MAX; //
                double min_dist_viewpoint_ind =
                    candidate_viewpoint_indices.front(); // 初始化离cell中心最短的距离和视点索引

                // 遍历该cell内的候选视点，找到距离cell中心最短的视点和距离
                for (const auto& viewpoint_ind : candidate_viewpoint_indices)
                {
                    geometry_msgs::Point viewpoint_position =
                        viewpoint_manager->GetViewPointPosition(viewpoint_ind); // 获取该视点的位置
                    // 计算该视点与cell中心的距离
                    double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
                        viewpoint_position, subspaces_->GetCell(cell_ind).GetPosition());

                    if (dist_to_cell_center < min_dist)
                    {
                        min_dist = dist_to_cell_center;
                        min_dist_viewpoint_ind = viewpoint_ind;
                    }
                }

                geometry_msgs::Point min_dist_viewpoint_position = viewpoint_manager->GetViewPointPosition(
                    min_dist_viewpoint_ind); // 获取距离该cell中心最近的视点的位置

                subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(Eigen::Vector3d(
                    min_dist_viewpoint_position.x, min_dist_viewpoint_position.y, min_dist_viewpoint_position.z));
                // 设置该视点位置为该cell与roadmap的连接点

                subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true); // 反馈已将该cell与roadmap的连接
            }
        }

        // 再次遍历邻近的每个cell
        for (int from_cell_ind : neighbor_cell_indices_)
        {
            int viewpoint_num =
                subspaces_->GetCell(from_cell_ind).GetViewPointIndices().size(); // 获取该cell内的视点数量
            if (viewpoint_num == 0) // 如果该cell内的视点为空，则跳过该cell
            {
                continue;
            }
            std::vector<int> from_cell_connected_cell_indices =
                subspaces_->GetCell(from_cell_ind).GetConnectedCellIndices();
            // 获取与该cell存在连接的cell索引集合
            Eigen::Vector3d from_cell_roadmap_connection_position =
                subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint(); // 获取该cell与roadmap的连接点位置

            if (!viewpoint_manager->InLocalPlanningHorizon(from_cell_roadmap_connection_position))
            // 如果该连接点位置在局部规划区域外，则跳过该cell
            {
                continue;
            }

            Eigen::Vector3i from_cell_sub =
                subspaces_->Ind2Sub(from_cell_ind); // ind是cell的一维全局索引，sub为cell的三维索引
            std::vector<int> nearby_cell_indices; //

            for (int x = -1; x <= 1; x++)
            {
                for (int y = -1; y <= 1; y++)
                {
                    for (int z = -1; z <= 1; z++)
                    {
                        if (std::abs(x) + std::abs(y) + std::abs(z) ==
                            1) // 栅格空间内与from_cell_ind相邻的cell(6个),上下左右前后，距离为一个单元格cell
                        {
                            Eigen::Vector3i neighbor_sub = from_cell_sub + Eigen::Vector3i(x, y, z);
                            if (subspaces_->InRange(
                                    neighbor_sub)) // 如果该cell在栅格空间内，则添加到nearby_cell_indices中
                            {
                                int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub); // 获取该cell的一维全局索引
                                nearby_cell_indices.push_back(neighbor_ind); // 添加到nearby_cell_indices中
                            }
                        }
                    }
                }
            }

            // 遍历该cell的相邻cell
            for (int to_cell_ind : nearby_cell_indices)
            {
                //
                // For debug
                if (!AreNeighbors(from_cell_ind, to_cell_ind)) // 再判断是否为相邻
                {
                    ROS_ERROR_STREAM("Cell " << from_cell_ind << " and " << to_cell_ind << " are not neighbors");
                }

                if (subspaces_->GetCell(to_cell_ind)
                        .GetViewPointIndices()
                        .empty()) // 如果该相邻的cell内无viewpoint，则跳过该cell
                {
                    continue;
                }

                std::vector<int> to_cell_connected_cell_indices =
                    subspaces_->GetCell(to_cell_ind).GetConnectedCellIndices();
                // 获取与该邻近cell存在连接的cell索引集合
                Eigen::Vector3d to_cell_roadmap_connection_position =
                    subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint(); // 获取该邻近cell与roadmap的连接点位置
                if (!viewpoint_manager->InLocalPlanningHorizon(to_cell_roadmap_connection_position))
                // 如果该连接点位置在局部规划区域外，则跳过该邻近cell
                {
                    continue;
                }

                // TODO: change to: if there is already a direct keypose graph connection then continue
                bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
                    keypose_graph, from_cell_roadmap_connection_position,
                    to_cell_roadmap_connection_position); // 判断该cell与该邻近cell之间在keypose_graph中是否已经存在直接连接

                // bool forward_connected =
                //     std::find(from_cell_connected_cell_indices.begin(), from_cell_connected_cell_indices.end(),
                //     to_cell_ind) != from_cell_connected_cell_indices.end();//

                // bool backward_connected = std::find(to_cell_connected_cell_indices.begin(),
                // to_cell_connected_cell_indices.end(),
                //                                     from_cell_ind) != to_cell_connected_cell_indices.end();

                if (connected_in_keypose_graph) // 如果该cell与该邻近cell之间在keypose_graph中已经存在直接连接，则跳过该cell
                {
                    continue;
                }

                nav_msgs::Path path_in_between = viewpoint_manager->GetViewPointShortestPath(
                    from_cell_roadmap_connection_position,
                    to_cell_roadmap_connection_position); // 获取该cell与该邻近cell之间的最短路径

                if (PathValid(path_in_between, from_cell_ind, to_cell_ind)) // 若该路径有效
                {
                    path_in_between = misc_utils_ns::SimplifyPath(path_in_between); // 将相连路径简化

                    for (auto& pose : path_in_between.poses)
                    {
                        pose.pose.orientation.w = -1; //?
                    }

                    // 将此路径添加到keypose_graph之中
                    keypose_graph->AddPath(path_in_between);
                    // 再验证keypose_graph之中是否已经存在这两个cell之间的连接
                    bool connected = HasDirectKeyposeGraphConnection(
                        keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);
                    if (!connected)
                    {
                        // 如果发现这个两个cell仍未在keypose graph中建立连接，重置这两个cell的连接状态和来连接的cell集合
                        subspaces_->GetCell(from_cell_ind).SetRoadmapConnectionPointSet(false);
                        subspaces_->GetCell(to_cell_ind).SetRoadmapConnectionPointSet(false);
                        subspaces_->GetCell(from_cell_ind).ClearConnectedCellIndices();
                        subspaces_->GetCell(to_cell_ind).ClearConnectedCellIndices();
                        continue;
                    }
                    else // 如果连接成功，则将这两cell添加为互相的已连接的cell
                    {
                        subspaces_->GetCell(from_cell_ind).AddConnectedCell(to_cell_ind);
                        subspaces_->GetCell(to_cell_ind).AddConnectedCell(from_cell_ind);
                    }
                }
            }
        }
    }

    // 判断路径是否有效？
    bool GridWorld::PathValid(const nav_msgs::Path& path, int from_cell_ind, const int to_cell_ind) const
    {
        if (path.poses.size() >= 2)
        {
            for (const auto& pose : path.poses)
            {
                const int cell_ind = GetCellInd(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                // 该路径点应在起点cell或终点cell之中？
                if (cell_ind != from_cell_ind && cell_ind != to_cell_ind)
                {
                    return false;
                }
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    // 在keypose_graph中能否搜索到从start_position到goal_position之间的路径，路径长度不超过max_path_length(两倍cell边长)
    bool
    GridWorld::HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                               const Eigen::Vector3d& start_position,
                                               const Eigen::Vector3d& goal_position) const
    {
        // 查看起始位置和目标点位置是否有节点在keypose_graph中
        if (!keypose_graph->HasNode(start_position) || !keypose_graph->HasNode(goal_position))
        {
            return false;
        }

        // Search a path connecting start_position and goal_position with a max path length constraint
        // 搜索从start_position到goal_position之间的路径，路径长度不超过max_path_length
        geometry_msgs::Point geo_start_position;
        geo_start_position.x = start_position.x();
        geo_start_position.y = start_position.y();
        geo_start_position.z = start_position.z();

        geometry_msgs::Point geo_goal_position;
        geo_goal_position.x = goal_position.x();
        geo_goal_position.y = goal_position.y();
        geo_goal_position.z = goal_position.z();

        double max_path_length = kCellSize * 2;
        nav_msgs::Path path;
        // 在keypose_graph中能否搜索到从start_position到goal_position之间的路径，路径长度不超过max_path_length即两倍cell边长
        const bool found_path = keypose_graph->GetShortestPathWithMaxLength(geo_start_position, geo_goal_position,
                                                                            max_path_length, false, path);
        //
        return found_path;
    }
} // namespace grid_world_ns
