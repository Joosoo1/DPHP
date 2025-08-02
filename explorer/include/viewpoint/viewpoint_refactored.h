/**
 * @file viewpoint_refactored.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a viewpoint
 * @version 0.2
 * @date 2025-07-28
 *
 * @copyright Copyright (c) 2021-2025
 *
 */
#pragma once

#include <vector>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "lidar_model/lidar_model.h"

namespace viewpoint_ns {

/**
 * @brief Class representing a viewpoint in 3D space
 * 
 * A viewpoint represents a potential observation position for a robot.
 * It includes information about visibility, collision status, and coverage.
 */
class ViewPointRefactored {
 public:
  /**
   * @brief Construct a new ViewPoint object
   * 
   * @param x X coordinate
   * @param y Y coordinate
   * @param z Z coordinate
   */
  explicit ViewPointRefactored(double x = 0.0, double y = 0.0, double z = 0.0);

  /**
   * @brief Construct a new ViewPoint object from a geometry_msgs::Point
   * 
   * @param position Position as a geometry_msgs::Point
   */
  explicit ViewPointRefactored(const geometry_msgs::Point& position);

  /**
   * @brief Destroy the ViewPoint object
   */
  ~ViewPointRefactored() = default;

  /**
   * @brief Update coverage based on a point
   * 
   * @tparam PCLPointType Type of the PCL point
   * @param point Point to update coverage with
   */
  template <class PCLPointType>
  void UpdateCoverage(const PCLPointType& point) {
    lidar_model_.UpdateCoverage<PCLPointType>(point);
  }

  /**
   * @brief Check if a point is visible from this viewpoint
   * 
   * @tparam PCLPointType Type of the PCL point
   * @param point Point to check visibility for
   * @param occlusion_threshold Threshold for occlusion checking
   * @return true If the point is visible
   * @return false If the point is not visible
   */
  template <class PCLPointType>
  bool CheckVisibility(const PCLPointType& point, double occlusion_threshold) const {
    return lidar_model_.CheckVisibility<PCLPointType>(point, occlusion_threshold);
  }

  /**
   * @brief Set the position of the viewpoint
   * 
   * @param position New position as geometry_msgs::Point
   */
  void SetPosition(const geometry_msgs::Point& position);

  /**
   * @brief Get the X coordinate
   * 
   * @return double X coordinate
   */
  double GetX() const;

  /**
   * @brief Get the Y coordinate
   * 
   * @return double Y coordinate
   */
  double GetY() const;

  /**
   * @brief Get the height (Z coordinate)
   * 
   * @return double Height
   */
  double GetHeight() const;

  /**
   * @brief Set the height (Z coordinate)
   * 
   * @param height New height value
   */
  void SetHeight(double height);

  /**
   * @brief Get the position as geometry_msgs::Point
   * 
   * @return geometry_msgs::Point Position
   */
  geometry_msgs::Point GetPosition() const;

  /**
   * @brief Reset coverage information
   */
  void ResetCoverage();

  /**
   * @brief Reset all viewpoint properties
   */
  void Reset();

  /**
   * @brief Set collision status
   * 
   * @param in_collision Whether the viewpoint is in collision
   */
  void SetInCollision(bool in_collision);

  /**
   * @brief Check if viewpoint is in collision
   * 
   * @return true If in collision
   * @return false If not in collision
   */
  bool InCollision() const;

  /**
   * @brief Set line of sight status
   * 
   * @param in_line_of_sight Whether the viewpoint is in line of sight
   */
  void SetInLineOfSight(bool in_line_of_sight);

  /**
   * @brief Check if viewpoint is in line of sight
   * 
   * @return true If in line of sight
   * @return false If not in line of sight
   */
  bool InLineOfSight() const;

  /**
   * @brief Set current frame line of sight status
   * 
   * @param in_current_frame_line_of_sight Whether viewpoint is in line of sight in current frame
   */
  void SetInCurrentFrameLineOfSight(bool in_current_frame_line_of_sight);

  /**
   * @brief Check if viewpoint is in line of sight in current frame
   * 
   * @return true If in line of sight in current frame
   * @return false If not in line of sight in current frame
   */
  bool InCurrentFrameLineOfSight() const;

  /**
   * @brief Set connected status
   * 
   * @param connected Whether viewpoint is connected
   */
  void SetConnected(bool connected);

  /**
   * @brief Check if viewpoint is connected
   * 
   * @return true If connected
   * @return false If not connected
   */
  bool Connected() const;

  /**
   * @brief Set visited status
   * 
   * @param visited Whether viewpoint has been visited
   */
  void SetVisited(bool visited);

  /**
   * @brief Check if viewpoint has been visited
   * 
   * @return true If visited
   * @return false If not visited
   */
  bool Visited() const;

  /**
   * @brief Set selected status
   * 
   * @param selected Whether viewpoint is selected
   */
  void SetSelected(bool selected);

  /**
   * @brief Check if viewpoint is selected
   * 
   * @return true If selected
   * @return false If not selected
   */
  bool Selected() const;

  /**
   * @brief Set candidate status
   * 
   * @param candidate Whether viewpoint is a candidate
   */
  void SetCandidate(bool candidate);

  /**
   * @brief Check if viewpoint is a candidate
   * 
   * @return true If candidate
   * @return false If not candidate
   */
  bool IsCandidate() const;

  /**
   * @brief Set terrain height status
   * 
   * @param has_terrain_height Whether viewpoint has terrain height
   */
  void SetHasTerrainHeight(bool has_terrain_height);

  /**
   * @brief Check if viewpoint has terrain height
   * 
   * @return true If has terrain height
   * @return false If doesn't have terrain height
   */
  bool HasTerrainHeight() const;

  /**
   * @brief Set terrain height value
   * 
   * @param terrain_height Terrain height value
   */
  void SetTerrainHeight(double terrain_height);

  /**
   * @brief Get terrain height value
   * 
   * @return double Terrain height
   */
  double GetTerrainHeight() const;

  /**
   * @brief Set terrain neighbor status
   * 
   * @param has_terrain_neighbor Whether viewpoint has terrain neighbor
   */
  void SetHasTerrainNeighbor(bool has_terrain_neighbor);

  /**
   * @brief Check if viewpoint has terrain neighbor
   * 
   * @return true If has terrain neighbor
   * @return false If doesn't have terrain neighbor
   */
  bool HasTerrainNeighbor() const;

  /**
   * @brief Set exploring cell status
   * 
   * @param in_exploring_cell Whether viewpoint is in exploring cell
   */
  void SetInExploringCell(bool in_exploring_cell);

  /**
   * @brief Check if viewpoint is in exploring cell
   * 
   * @return true If in exploring cell
   * @return false If not in exploring cell
   */
  bool InExploringCell() const;

  /**
   * @brief Set cell index
   * 
   * @param cell_ind Cell index
   */
  void SetCellInd(int cell_ind);

  /**
   * @brief Get cell index
   * 
   * @return int Cell index
   */
  int GetCellInd() const;

  /**
   * @brief Get cell index (alias for GetCellInd)
   * 
   * @return int Cell index
   */
  int GetCellIndex() const;

  /**
   * @brief Get visualization cloud
   * 
   * @param vis_cloud Output visualization cloud
   */
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud) const;

  /**
   * @brief Reset covered point list
   */
  void ResetCoveredPointList();

  /**
   * @brief Reset covered frontier point list
   */
  void ResetCoveredFrontierPointList();

  /**
   * @brief Get covered point list
   * 
   * @return const std::vector<int>& Covered point list
   */
  const std::vector<int>& GetCoveredPointList() const;

  /**
   * @brief Get covered frontier point list
   * 
   * @return const std::vector<int>& Covered frontier point list
   */
  const std::vector<int>& GetCoveredFrontierPointList() const;

  /**
   * @brief Add a covered point
   * 
   * @param point_idx Point index to add
   */
  void AddCoveredPoint(int point_idx);

  /**
   * @brief Add a covered frontier point
   * 
   * @param point_idx Frontier point index to add
   */
  void AddCoveredFrontierPoint(int point_idx);

  /**
   * @brief Get number of covered points
   * 
   * @return int Number of covered points
   */
  int GetCoveredPointNum() const;

  /**
   * @brief Get number of covered frontier points
   * 
   * @return int Number of covered frontier points
   */
  int GetCoveredFrontierPointNum() const;

  /**
   * @brief Get collision frame count
   * 
   * @return int Collision frame count
   */
  int GetCollisionFrameCount() const;

  /**
   * @brief Add a collision frame
   */
  void AddCollisionFrame();

  /**
   * @brief Reset collision frame count
   */
  void ResetCollisionFrameCount();

 private:
  /// LiDAR model for visibility and coverage calculations
  lidar_model_ns::LiDARModel lidar_model_;

  /// Whether this viewpoint is in collision with the environment
  bool in_collision_;

  /// Whether this viewpoint has been in the line of sight of the robot
  bool in_line_of_sight_;

  /// Whether this viewpoint and the robot's current location are within the same connected component
  bool connected_;

  /// Whether this viewpoint has been visited by the robot
  bool visited_;

  /// Whether this viewpoint is selected to form the path
  bool selected_;

  /// Whether this viewpoint is a candidate to be selected to form the path
  bool is_candidate_;

  /// Whether this viewpoint has a height set from terrain analysis
  bool has_terrain_height_;

  /// Whether this viewpoint is in an EXPLORING cell
  bool in_exploring_cell_;

  /// The index of the cell this viewpoint is in
  int cell_ind_;

  /// The index of this viewpoint among all the candidate viewpoints
  int collision_frame_count_;

  /// Terrain height for this viewpoint
  double terrain_height_;

  /// Whether this viewpoint has a terrain neighbor
  bool has_terrain_neighbor_;

  /// Whether the viewpoint is in line of sight in the current frame
  bool in_current_frame_line_of_sight_;

  /// Indices of the covered points
  std::vector<int> covered_point_list_;

  /// Indices of the covered frontier points
  std::vector<int> covered_frontier_point_list_;
};

}  // namespace viewpoint_ns