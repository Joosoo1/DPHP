/**
 * @file semi_dynamic_detector.h
 * @brief Semi-dynamic object detection and state prediction
 */

#pragma once

#include <vector>
#include <string>
#include <Eigen/Core>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace semi_dynamic_ns {

// 半动态物体类型
enum class ObjectType {
    DOOR,
    CHAIR,
    CABINET,
    TABLE,
    UNKNOWN
};

// 半动态物体状态
struct SemiDynamicObject {
    int id;
    ObjectType type;
    geometry_msgs::Pose pose;
    Eigen::Vector3d size;  // 物体尺寸
    bool is_open;  // 对于门、柜子等
    bool is_movable;  // 是否可移动
    double stability;  // 稳定性评分 (0-1)
    double change_probability;  // 状态改变概率
    ros::Time last_observed;
    std::vector<geometry_msgs::Point> occupied_volume;  // 占据空间
};

class SemiDynamicDetector {
public:
    SemiDynamicDetector(ros::NodeHandle& nh);
    
    // 从点云检测半动态物体
    void DetectFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    // 更新物体状态
    void UpdateObjectStates(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    // 获取检测到的物体
    const std::vector<SemiDynamicObject>& GetDetectedObjects() const { return detected_objects_; }
    
    // 检查位置是否可能变得可通行
    bool CheckPotentialAccessibility(const Eigen::Vector3d& position) const;
    
    // 获取物体类型字符串
    static std::string GetTypeString(ObjectType type);
    
private:
    // 检测特定类型的物体
    void DetectDoors(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void DetectChairs(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void DetectCabinets(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    
    // 物体跟踪和状态更新
    void TrackObjects(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    void UpdateChangeProbabilities();
    
    // 几何分析工具
    bool IsDoorLike(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) const;
    bool IsChairLike(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) const;
    bool IsCabinetLike(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) const;
    
    std::vector<SemiDynamicObject> detected_objects_;
    
    // 检测参数
    double door_detection_threshold_;
    double chair_detection_threshold_;
    double cabinet_detection_threshold_;
    double min_object_size_;
    double max_object_size_;
};

} // namespace semi_dynamic_ns