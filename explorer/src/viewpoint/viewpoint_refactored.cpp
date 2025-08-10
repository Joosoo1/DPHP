/**
 * @file viewpoint_refactored.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a viewpoint
 * @version 0.2
 * @date 2025-07-28
 *
 * @copyright Copyright (c) 2021-2025
 *
 */

#include "viewpoint/viewpoint_refactored.h"
#include <ros/ros.h>

namespace viewpoint_ns
{

    ViewPointRefactored::ViewPointRefactored(const double x, const double y, const double z) :
        lidar_model_(x, y, z), in_collision_(false), in_line_of_sight_(false), connected_(false), visited_(false),
        selected_(false), is_candidate_(false), has_terrain_height_(false), in_exploring_cell_(false), cell_ind_(-1),
        collision_frame_count_(0), terrain_height_(0.0), has_terrain_neighbor_(false),
        in_current_frame_line_of_sight_(false)
    {
        // Validate inputs
        if (std::isnan(x) || std::isnan(y) || std::isnan(z))
        {
            ROS_WARN("ViewPointRefactored initialized with NaN coordinates");
        }

        if (std::isinf(x) || std::isinf(y) || std::isinf(z))
        {
            ROS_WARN("ViewPointRefactored initialized with infinite coordinates");
        }
    }

    ViewPointRefactored::ViewPointRefactored(const geometry_msgs::Point& position) :
        ViewPointRefactored(position.x, position.y, position.z)
    {
        // Additional validation for geometry_msgs::Point constructor
        if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z))
        {
            ROS_WARN("ViewPointRefactored initialized with NaN coordinates from geometry_msgs::Point");
        }

        if (std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
        {
            ROS_WARN("ViewPointRefactored initialized with infinite coordinates from geometry_msgs::Point");
        }
    }

    void ViewPointRefactored::SetPosition(const geometry_msgs::Point& position)
    {
        if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z))
        {
            ROS_ERROR("Attempted to set ViewPointRefactored position to NaN values");
            return;
        }

        if (std::isinf(position.x) || std::isinf(position.y) || std::isinf(position.z))
        {
            ROS_ERROR("Attempted to set ViewPointRefactored position to infinite values");
            return;
        }

        lidar_model_.setPosition(position);
    }

    double ViewPointRefactored::GetX() const { return lidar_model_.getPosition().x; }

    double ViewPointRefactored::GetY() const { return lidar_model_.getPosition().y; }

    double ViewPointRefactored::GetHeight() const { return lidar_model_.getPosition().z; }

    void ViewPointRefactored::SetHeight(double height)
    {
        if (std::isnan(height) || std::isinf(height))
        {
            ROS_ERROR("Attempted to set ViewPointRefactored height to invalid value: %f", height);
            return;
        }

        lidar_model_.SetHeight(height);
    }

    geometry_msgs::Point ViewPointRefactored::GetPosition() const { return lidar_model_.getPosition(); }

    void ViewPointRefactored::Reset()
    {
        in_collision_ = false;
        in_line_of_sight_ = false;
        connected_ = false;
        visited_ = false;
        selected_ = false;
        is_candidate_ = false;
        has_terrain_height_ = false;
        has_terrain_neighbor_ = false;
        in_exploring_cell_ = false;
        cell_ind_ = -1;
        lidar_model_.ResetCoverage();
        covered_point_list_.clear();
        covered_frontier_point_list_.clear();
        collision_frame_count_ = 0;
        terrain_height_ = 0.0;
        in_current_frame_line_of_sight_ = false;
    }

    void ViewPointRefactored::ResetCoverage()
    {
        lidar_model_.ResetCoverage();
        covered_point_list_.clear();
        covered_frontier_point_list_.clear();
    }

    void ViewPointRefactored::SetInCollision(const bool in_collision) { in_collision_ = in_collision; }

    bool ViewPointRefactored::InCollision() const { return in_collision_; }

    void ViewPointRefactored::SetInLineOfSight(const bool in_line_of_sight) { in_line_of_sight_ = in_line_of_sight; }

    bool ViewPointRefactored::InLineOfSight() const { return in_line_of_sight_; }

    void ViewPointRefactored::SetInCurrentFrameLineOfSight(const bool in_current_frame_line_of_sight)
    {
        in_current_frame_line_of_sight_ = in_current_frame_line_of_sight;
    }

    bool ViewPointRefactored::InCurrentFrameLineOfSight() const { return in_current_frame_line_of_sight_; }

    void ViewPointRefactored::SetConnected(const bool connected) { connected_ = connected; }

    bool ViewPointRefactored::Connected() const { return connected_; }

    void ViewPointRefactored::SetVisited(const bool visited) { visited_ = visited; }

    bool ViewPointRefactored::Visited() const { return visited_; }

    void ViewPointRefactored::SetSelected(const bool selected) { selected_ = selected; }

    bool ViewPointRefactored::Selected() const { return selected_; }

    void ViewPointRefactored::SetCandidate(const bool candidate) { is_candidate_ = candidate; }

    bool ViewPointRefactored::IsCandidate() const { return is_candidate_; }

    void ViewPointRefactored::SetHasTerrainHeight(const bool has_terrain_height) { has_terrain_height_ = has_terrain_height; }

    bool ViewPointRefactored::HasTerrainHeight() const { return has_terrain_height_; }

    void ViewPointRefactored::SetTerrainHeight(const double terrain_height)
    {
        if (std::isnan(terrain_height) || std::isinf(terrain_height))
        {
            ROS_ERROR("Attempted to set terrain height to invalid value: %f", terrain_height);
            return;
        }

        terrain_height_ = terrain_height;
    }

    double ViewPointRefactored::GetTerrainHeight() const { return terrain_height_; }

    void ViewPointRefactored::SetHasTerrainNeighbor(bool has_terrain_neighbor)
    {
        has_terrain_neighbor_ = has_terrain_neighbor;
    }

    bool ViewPointRefactored::HasTerrainNeighbor() const { return has_terrain_neighbor_; }

    void ViewPointRefactored::SetInExploringCell(const bool in_exploring_cell) { in_exploring_cell_ = in_exploring_cell; }

    bool ViewPointRefactored::InExploringCell() const { return in_exploring_cell_; }

    void ViewPointRefactored::SetCellInd(const int cell_ind)
    {
        if (cell_ind < -1)
        {
            ROS_WARN("Setting cell index to negative value: %d", cell_ind);
        }

        cell_ind_ = cell_ind;
    }

    int ViewPointRefactored::GetCellInd() const { return cell_ind_; }

    int ViewPointRefactored::GetCellIndex() const { return cell_ind_; }

    void ViewPointRefactored::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const
    {
        lidar_model_.GetVisualizationCloud(vis_cloud);
    }

    void ViewPointRefactored::ResetCoveredPointList() { covered_point_list_.clear(); }

    void ViewPointRefactored::ResetCoveredFrontierPointList() { covered_frontier_point_list_.clear(); }

    const std::vector<int>& ViewPointRefactored::GetCoveredPointList() const { return covered_point_list_; }

    const std::vector<int>& ViewPointRefactored::GetCoveredFrontierPointList() const
    {
        return covered_frontier_point_list_;
    }

    void ViewPointRefactored::AddCoveredPoint(const int point_idx)
    {
        if (point_idx < 0)
        {
            ROS_WARN("Adding negative point index to covered point list: %d", point_idx);
        }

        covered_point_list_.push_back(point_idx);
    }

    void ViewPointRefactored::AddCoveredFrontierPoint(const int point_idx)
    {
        if (point_idx < 0)
        {
            ROS_WARN("Adding negative point index to covered frontier point list: %d", point_idx);
        }

        covered_frontier_point_list_.push_back(point_idx);
    }

    int ViewPointRefactored::GetCoveredPointNum() const { return static_cast<int>(covered_point_list_.size()); }

    int ViewPointRefactored::GetCoveredFrontierPointNum() const
    {
        return static_cast<int>(covered_frontier_point_list_.size());
    }

    int ViewPointRefactored::GetCollisionFrameCount() const { return collision_frame_count_; }

    void ViewPointRefactored::AddCollisionFrame() { collision_frame_count_++; }

    void ViewPointRefactored::ResetCollisionFrameCount() { collision_frame_count_ = 0; }

} // namespace viewpoint_ns
