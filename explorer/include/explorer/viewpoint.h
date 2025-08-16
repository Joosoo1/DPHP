/**
 * @file viewpoint.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a viewpoint
 * @version 0.1
 * @date 2019-11-04
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <geometry_msgs/Point.h>
#include "explorer/lidar_model.h"

namespace viewpoint_ns
{
    class ViewPoint
    {
    public:
        explicit ViewPoint(double x = 0.0, double y = 0.0, double z = 0.0);
        explicit ViewPoint(const geometry_msgs::Point& position);
        ~ViewPoint() = default;

        template <class PCLPointType>
        void UpdateCoverage(const PCLPointType& point)
        {
            lidar_model_.UpdateCoverage<PCLPointType>(point);
        }
        template <class PCLPointType>
        bool CheckVisibility(const PCLPointType& point, double occlusion_threshold) const
        {
            return lidar_model_.CheckVisibility<PCLPointType>(point, occlusion_threshold);
        }
        void SetPosition(const geometry_msgs::Point& position) { lidar_model_.setPosition(position); }
        double GetX() const { return lidar_model_.getPosition().x; }
        double GetY() const { return lidar_model_.getPosition().y; }
        double GetHeight() const { return lidar_model_.getPosition().z; }

        void SetHeight(double height) { lidar_model_.SetHeight(height); }

        geometry_msgs::Point GetPosition() const { return lidar_model_.getPosition(); }
        void ResetCoverage();
        void Reset();

        void SetInCollision(const bool in_collision) { in_collision_ = in_collision; }
        bool InCollision() const { return in_collision_; }

        void SetInLineOfSight(const bool in_line_of_sight) { in_line_of_sight_ = in_line_of_sight; }
        bool InLineOfSight() const { return in_line_of_sight_; }

        void SetInCurrentFrameLineOfSight(const bool in_current_frame_line_of_sight)
        {
            in_current_frame_line_of_sight_ = in_current_frame_line_of_sight;
        }
        bool InCurrentFrameLineOfSight() const { return in_current_frame_line_of_sight_; }

        void SetConnected(const bool connected) { connected_ = connected; }
        bool Connected() const { return connected_; }

        void SetVisited(const bool visited) { visited_ = visited; }
        bool Visited() const { return visited_; }

        void SetSelected(const bool selected) { selected_ = selected; }
        bool Selected() const { return selected_; }

        void SetCandidate(const bool candidate) { is_candidate_ = candidate; }
        bool IsCandidate() const { return is_candidate_; }

        void SetHasTerrainHeight(const bool has_terrain_height) { has_terrain_height_ = has_terrain_height; }
        bool HasTerrainHeight() const { return has_terrain_height_; }

        void SetTerrainHeight(const double terrain_height) { terrain_height_ = terrain_height; }
        double GetTerrainHeight() const { return terrain_height_; }

        void SetHasTerrainNeighbor(const bool has_terrain_neighbor) { has_terrain_neighbor_ = has_terrain_neighbor; }
        bool HasTerrainNeighbor() const { return has_terrain_neighbor_; }

        void SetInExploringCell(const bool in_exploring_cell) { in_exploring_cell_ = in_exploring_cell; }
        bool InExploringCell() const { return in_exploring_cell_; }

        void SetCellInd(const int cell_ind) { cell_ind_ = cell_ind; }
        int GetCellInd() const { return cell_ind_; }

        int GetCellIndex() const { return cell_ind_; }

        void GetVisualizationCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const
        {
            lidar_model_.GetVisualizationCloud(vis_cloud);
        }
        void ResetCoveredPointList() { covered_point_list_.clear(); }
        void ResetCoveredFrontierPointList() { covered_frontier_point_list_.clear(); }
        const std::vector<int>& GetCoveredPointList() const { return covered_point_list_; }
        const std::vector<int>& GetCoveredFrontierPointList() const { return covered_frontier_point_list_; }
        void AddCoveredPoint(const int point_idx) { covered_point_list_.push_back(point_idx); }
        void AddCoveredFrontierPoint(const int point_idx) { covered_frontier_point_list_.push_back(point_idx); }
        int GetCoveredPointNum() const { return covered_point_list_.size(); }
        int GetCoveredFrontierPointNum() const { return covered_frontier_point_list_.size(); }
        int GetCollisionFrameCount() const { return collision_frame_count_; }
        void AddCollisionFrame() { collision_frame_count_++; }
        void ResetCollisionFrameCount() { collision_frame_count_ = 0; }

    private:
        lidar_model_ns::LiDARModel lidar_model_;

        // Whether this viewpoint is in collision with the environment 这个视点是否与环境发生碰撞
        bool in_collision_;
        // Whether this viewpoint has been in the line of sight of the robot 这个视点是否在机器人的视线范围内
        bool in_line_of_sight_;
        // Whether this viewpoint and the robot’s current location are within the same connected component.
        // It must be true to have a collision-free path planned from the current robot position to this viewpoint.
        // 这个视点和机器人当前位置是否在同一连通部分。为了能够从当前机器人位置规划出一条无碰撞路径到这个视点，这一点必须是true
        bool connected_;
        // Whether this viewpoint has been visited by the robot. If true, its coverage area will not be updated and it
        // will not be selected in the sampling process.
        // 这个视点是否已经被机器人访问过。如果是的话，它的覆盖区域将不会被更新，并且在采样过程中也不会被选中
        bool visited_;
        // Whether this viewpoint is selected to form the path. 这个视点是否被选中来形成路径
        bool selected_;
        // Whether this viewpoint is a candidate to be selected to form the path 这个视点是否是形成路径的候选点
        bool is_candidate_;
        // Whether this viewpoint has a height set from terrain analysis 这个视点的高度设定值是否来自分析模块
        bool has_terrain_height_;
        // Whether this viewpoint is in an EXPLORING cell 这个视点是否在EXPLORING cell内
        bool in_exploring_cell_;
        // The index of the cell this viewpoint is in 这个视点所处的cell索引
        int cell_ind_;
        // The index of this viewpoint among all the candidate viewpoints 这个视点在所有候选视点之中的索引
        int collision_frame_count_;
        // For debug
        double terrain_height_; // 地形高度
        bool has_terrain_neighbor_; //
        // Whether the viewpoint is in line of sight in the current frame
        bool in_current_frame_line_of_sight_; // 该视点是否在当前帧的视线范围内
        // Indices of the covered points
        std::vector<int> covered_point_list_; // 覆盖的点集合列表
        // Indices of the covered frontier points
        std::vector<int> covered_frontier_point_list_; // 覆盖的边界点集合列表
    };
} // namespace viewpoint_ns
