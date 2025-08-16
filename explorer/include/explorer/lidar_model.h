/**
 * @file lidar_model.h
 * @author joosoo (joosoo@buct.edu.cn)
 * @brief Class that implements the sensor model of a LiDAR
 * @version 0.1
 * @date 2019-09-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <string>
// ROS
#include <geometry_msgs/Point.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "explorer/misc_utils.h"

namespace lidar_model_ns
{
    class LiDARModel
    {
    public:
        static double pointcloud_resolution_;
        explicit LiDARModel(double px = 0.0, double py = 0.0, double pz = 0.0, double rw = 1.0, double rx = 0.0,
                            double ry = 0.0, double rz = 0.0);
        explicit LiDARModel(const geometry_msgs::Pose& pose);
        ~LiDARModel() = default;

        /**
         * @brief
         * TODO
         * @tparam PointType
         * @param point
         */
        template <class PointType>
        void UpdateCoverage(const PointType& point)
        {
            const double distance_to_point =
                misc_utils_ns::PointXYZDist<PointType, geometry_msgs::Point>(point, pose_.position);

            if (isZero(distance_to_point))
                return;

            const double dx = point.x - pose_.position.x;
            const double dy = point.y - pose_.position.y;
            const double dz = point.z - pose_.position.z;

            const int horizontal_angle = GetHorizontalAngle(dx, dy);
            const int vertical_angle = GetVerticalAngle(dz, distance_to_point);

            const int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
            const int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

            for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; ++n)
            {
                const int column_index = horizontal_angle + n;
                if (!ColumnIndexInRange(column_index))
                    continue;
                for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; ++m)
                {
                    const int row_index = vertical_angle + m;
                    if (!RowIndexInRange(row_index))
                        continue;
                    const int ind = sub2ind(row_index, column_index);
                    const double previous_distance_to_point = covered_voxel_[ind];
                    if (isZero(previous_distance_to_point) || distance_to_point < previous_distance_to_point ||
                        reset_[ind])
                    {
                        covered_voxel_[ind] = distance_to_point;
                        reset_[ind] = false;
                    }
                }
            }
        }

        /**
         * @brief
         * TODO
         * @tparam PointType
         * @param point
         * @param occlusion_threshold
         * @return true
         * @return false
         */
        template <class PointType>
        bool CheckVisibility(const PointType& point, const double occlusion_threshold) const
        {
            const double distance_to_point =
                misc_utils_ns::PointXYZDist<PointType, geometry_msgs::Point>(point, pose_.position);

            if (isZero(distance_to_point))
                return false;

            const double dx = point.x - pose_.position.x;
            const double dy = point.y - pose_.position.y;
            const double dz = point.z - pose_.position.z;

            const int horizontal_angle = GetHorizontalAngle(dx, dy);
            const int vertical_angle = GetVerticalAngle(dz, distance_to_point);

            const int horizontal_neighbor_num = GetHorizontalNeighborNum(distance_to_point);
            const int vertical_neighbor_num = GetVerticalNeighborNum(distance_to_point);

            for (int n = -horizontal_neighbor_num; n <= horizontal_neighbor_num; ++n)
            {
                const int column_index = horizontal_angle + n;
                if (!ColumnIndexInRange(column_index))
                    continue;
                for (int m = -vertical_neighbor_num; m <= vertical_neighbor_num; ++m)
                {
                    const int row_index = vertical_angle + m;
                    if (!RowIndexInRange(row_index))
                        continue;
                    const int ind = sub2ind(row_index, column_index);
                    const float previous_distance_to_point = covered_voxel_[ind];
                    if ((!isZero(previous_distance_to_point) &&
                         distance_to_point < previous_distance_to_point + occlusion_threshold && !reset_[ind]) ||
                        reset_[ind])
                    {
                        return true;
                    }
                }
            }
            return false;
        }
        /**
         * @brief
         * TODO
         */
        void ResetCoverage();
        /**
         * @brief Get the Visualization Cloud object
         * TODO
         * @param visualization_cloud
         * @param resol
         * @param max_range
         */
        void GetVisualizationCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& visualization_cloud, double resol = 0.2,
                                   double max_range = 25.0) const;

        static void setCloudDWZResol(const double cloud_dwz_resol)
        {
            pointcloud_resolution_ = cloud_dwz_resol * kCloudInflateRatio;
        }

        void setPose(const geometry_msgs::Pose& pose) { pose_ = pose; }
        geometry_msgs::Pose getPose() const { return pose_; }
        void setPosition(const geometry_msgs::Point& position) { pose_.position = position; }
        geometry_msgs::Point getPosition() const { return pose_.position; }
        void SetHeight(const double height) { pose_.position.z = height; }

    private:
        /**
         * @brief convert subscripts to linear indices
         *
         * @param row_index row index
         * @param column_index column index
         * @return int linear index
         */
        static int sub2ind(int row_index, int column_index);
        /**
         * @brief convert linear indices to subscripts
         *
         * @param ind linear index
         * @param row_index row index
         * @param column_index column index
         */
        static void ind2sub(int ind, int& row_index, int& column_index);
        /**
         * @brief whether a number is close to zero
         *
         * @param x input number
         * @return true
         * @return false
         */
        static bool isZero(double x) { return std::abs(x) < kEpsilon; }

        /**
         * @brief Get the Horizontal Angle object
         * TODO
         * @param dx
         * @param dy
         * @return int
         */
        static int GetHorizontalAngle(double dx, double dy)
        {
            const double horizontal_angle =
                (misc_utils_ns::ApproxAtan2(dy, dx) * kToDegreeConst + 180) / kHorizontalResolution;
            return static_cast<int>(round(horizontal_angle));
        }
        /**
         * @brief Get the Vertical Angle object
         * TODO
         * @param dz
         * @param distance_to_point
         * @return int
         */
        static int GetVerticalAngle(const double dz, double distance_to_point)
        {
            const double vertical_angle =
                (acos(dz / distance_to_point) * kToDegreeConst + kVerticalAngleOffset) / kVerticalResolution;
            return static_cast<int>(round(vertical_angle));
        }
        /**
         * @brief Get the Horizontal Neighbor Num object
         * TODO
         * @param distance_to_point
         * @return int
         */
        static int GetHorizontalNeighborNum(const double distance_to_point)
        {
            return static_cast<int>(
                       ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kHorizontalResolution)) /
                2;
        }
        /**
         * @brief Get the Vertical Neighbor Num object
         * TODO
         * @param distance_to_point
         * @return int
         */
        static int GetVerticalNeighborNum(const double distance_to_point)
        {
            return static_cast<int>(
                       ceil(pointcloud_resolution_ / distance_to_point * kToDegreeConst / kVerticalResolution)) /
                2;
        }
        static bool RowIndexInRange(const int row_index) { return row_index >= 0 && row_index < kVerticalVoxelSize; }
        static bool ColumnIndexInRange(const int column_index)
        {
            return column_index >= 0 && column_index < kHorizontalVoxelSize;
        }

        // Constant converting radian to degree
        static const double kToDegreeConst;
        // Constant converting degree to radian
        static const double kToRadianConst;
        // Threshold for checking if a number is close to zero
        static const double kEpsilon;
        // Ratio for inflating the cloud
        static const double kCloudInflateRatio;
        // Horizontal field-of-view in degrees
        static constexpr int kHorizontalFOV = 360;
        // Vertical field-of-view in degrees
        static constexpr int kVerticalFOV = 24;
        // Horizontal resolution
        static constexpr int kHorizontalResolution = 2;
        // Vertical resolution
        static constexpr int kVerticalResolution = 2;
        // Horizontal dimension of the voxel grid
        static constexpr int kHorizontalVoxelSize = kHorizontalFOV / kHorizontalResolution;
        // Vertical dimension of the voxel grid
        static constexpr int kVerticalVoxelSize = kVerticalFOV / kVerticalResolution;
        // Vertical angle offset, eg, angle [75, 105] -> indices [0, 30]
        static constexpr int kVerticalAngleOffset = -(90 - kVerticalFOV / 2);
        // The distance that a ray can reach from the direction determined by the horizontal angle and vertical angle
        std::array<float, kHorizontalVoxelSize * kVerticalVoxelSize> covered_voxel_;
        // Whether a voxel is reset
        std::array<bool, kHorizontalVoxelSize * kVerticalVoxelSize> reset_;
        // Pose of the lidar model
        geometry_msgs::Pose pose_;
    };
} // namespace lidar_model_ns
