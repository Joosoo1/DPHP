/**
 * @file semi_dynamic_detector.cpp
 * @brief Implementation of semi-dynamic object detection
 */

#include "explorer/semi_dynamic_detector.h"
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <algorithm>
#include <cmath>

namespace semi_dynamic_ns {

SemiDynamicDetector::SemiDynamicDetector(ros::NodeHandle& nh) {
    // 从参数服务器读取检测参数
    nh.param("semi_dynamic/door_threshold", door_detection_threshold_, 0.7);
    nh.param("semi_dynamic/chair_threshold", chair_detection_threshold_, 0.6);
    nh.param("semi_dynamic/cabinet_threshold", cabinet_detection_threshold_, 0.65);
    nh.param("semi_dynamic/min_object_size", min_object_size_, 0.3);
    nh.param("semi_dynamic/max_object_size", max_object_size_, 2.0);
}

void SemiDynamicDetector::DetectFromPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    if (!cloud || cloud->empty()) {
        return;
    }
    
    // 清空之前的检测结果
    detected_objects_.clear();
    
    // 使用欧几里得聚类分割点云
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(0.05); // 5cm
    ec.setMinClusterSize(100);    // 最小点数
    ec.setMaxClusterSize(25000);  // 最大点数
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // 对每个聚类进行类型识别
    for (const auto& indices : cluster_indices) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::copyPointCloud(*cloud, indices, *cluster);
        
        // 检测门
        DetectDoors(cluster);
        
        // 检测椅子
        DetectChairs(cluster);
        
        // 检测柜子
        DetectCabinets(cluster);
    }
    
    // 更新状态改变概率
    UpdateChangeProbabilities();
}

void SemiDynamicDetector::UpdateObjectStates(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // 跟踪已有物体并更新状态
    TrackObjects(cloud);
    
    // 检测新物体
    DetectFromPointCloud(cloud);
    
    // 更新改变概率
    UpdateChangeProbabilities();
}

bool SemiDynamicDetector::CheckPotentialAccessibility(const Eigen::Vector3d& position) const {
    for (const auto& obj : detected_objects_) {
        Eigen::Vector3d obj_pos(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
        double distance = (position - obj_pos).norm();
        
        // 检查是否在物体影响范围内
        if (distance < 1.5) { // 1.5米范围内
            switch (obj.type) {
                case ObjectType::DOOR:
                    // 门关闭但可能被打开
                    if (!obj.is_open && obj.change_probability > 0.4) {
                        return true;
                    }
                    break;
                
                case ObjectType::CHAIR:
                    // 椅子可能被移动
                    if (obj.is_movable && obj.change_probability > 0.6) {
                        return true;
                    }
                    break;
                
                case ObjectType::CABINET:
                    // 柜门可能被打开
                    if (!obj.is_open && obj.change_probability > 0.5) {
                        return true;
                    }
                    break;
                
                default:
                    break;
            }
        }
    }
    
    return false;
}

std::string SemiDynamicDetector::GetTypeString(ObjectType type) {
    switch (type) {
        case ObjectType::DOOR: return "DOOR";
        case ObjectType::CHAIR: return "CHAIR";
        case ObjectType::CABINET: return "CABINET";
        case ObjectType::TABLE: return "TABLE";
        default: return "UNKNOWN";
    }
}

// 私有方法实现
void SemiDynamicDetector::DetectDoors(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) {
    if (IsDoorLike(cluster)) {
        SemiDynamicObject door;
        door.id = detected_objects_.size();
        door.type = ObjectType::DOOR;
        
        // 计算门的位置和尺寸
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        door.pose.position.x = centroid[0];
        door.pose.position.y = centroid[1];
        door.pose.position.z = centroid[2];
        
        // 简单假设门的状态
        door.is_open = false; // 初始状态为关闭
        door.is_movable = true;
        door.stability = 0.8;
        door.change_probability = 0.3; // 初始改变概率
        door.last_observed = ros::Time::now();
        
        detected_objects_.push_back(door);
    }
}

void SemiDynamicDetector::DetectChairs(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) {
    if (IsChairLike(cluster)) {
        SemiDynamicObject chair;
        chair.id = detected_objects_.size();
        chair.type = ObjectType::CHAIR;
        
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        chair.pose.position.x = centroid[0];
        chair.pose.position.y = centroid[1];
        chair.pose.position.z = centroid[2];
        
        chair.is_open = false; // 不适用
        chair.is_movable = true;
        chair.stability = 0.6;
        chair.change_probability = 0.5; // 椅子更容易被移动
        chair.last_observed = ros::Time::now();
        
        detected_objects_.push_back(chair);
    }
}

void SemiDynamicDetector::DetectCabinets(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) {
    if (IsCabinetLike(cluster)) {
        SemiDynamicObject cabinet;
        cabinet.id = detected_objects_.size();
        cabinet.type = ObjectType::CABINET;
        
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cluster, centroid);
        cabinet.pose.position.x = centroid[0];
        cabinet.pose.position.y = centroid[1];
        cabinet.pose.position.z = centroid[2];
        
        cabinet.is_open = false;
        cabinet.is_movable = false; // 柜子通常不可移动
        cabinet.stability = 0.9;
        cabinet.change_probability = 0.4; // 门可能被打开
        cabinet.last_observed = ros::Time::now();
        
        detected_objects_.push_back(cabinet);
    }
}

void SemiDynamicDetector::TrackObjects(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    // 简单的物体跟踪实现
    // 这里应该实现更复杂的多目标跟踪算法
    
    for (auto& obj : detected_objects_) {
        // 基于位置和特征的简单跟踪
        // 在实际系统中应该使用更先进的跟踪算法
        
        // 更新最后观测时间
        obj.last_observed = ros::Time::now();
    }
}

void SemiDynamicDetector::UpdateChangeProbabilities() {
    ros::Time now = ros::Time::now();
    
    for (auto& obj : detected_objects_) {
        // 基于时间衰减和物体类型的概率更新
        double time_since_last_seen = (now - obj.last_observed).toSec();
        double time_factor = std::exp(-time_since_last_seen / 300.0); // 5分钟半衰期
        
        switch (obj.type) {
            case ObjectType::DOOR:
                obj.change_probability = 0.3 + 0.4 * time_factor;
                break;
            
            case ObjectType::CHAIR:
                obj.change_probability = 0.5 + 0.3 * time_factor;
                break;
            
            case ObjectType::CABINET:
                obj.change_probability = 0.4 + 0.2 * time_factor;
                break;
            
            default:
                obj.change_probability = 0.2;
                break;
        }
        
        // 确保概率在合理范围内
        obj.change_probability = std::max(0.1, std::min(0.9, obj.change_probability));
    }
}

// 简单的几何形状识别函数
bool SemiDynamicDetector::IsDoorLike(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) const {
    // 计算点云边界框
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    double width = max_pt.x - min_pt.x;
    double height = max_pt.z - min_pt.z;
    double depth = max_pt.y - min_pt.y;
    
    // 门的典型尺寸: 高2m, 宽0.8-1m, 厚度小
    return (height > 1.8 && height < 2.2) && 
           (width > 0.7 && width < 1.2) && 
           (depth < 0.2);
}

bool SemiDynamicDetector::IsChairLike(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) const {
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    double height = max_pt.z - min_pt.z;
    // 椅子的典型高度
    return (height > 0.7 && height < 1.2);
}

bool SemiDynamicDetector::IsCabinetLike(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster) const {
    pcl::PointXYZI min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    double width = max_pt.x - min_pt.x;
    double height = max_pt.z - min_pt.z;
    double depth = max_pt.y - min_pt.y;
    
    // 柜子的典型尺寸
    return (height > 1.5 && height < 2.5) && 
           (width > 0.5 && width < 2.0) && 
           (depth > 0.3 && depth < 1.0);
}

} // namespace semi_dynamic_ns