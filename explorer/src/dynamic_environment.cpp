/**
 * @file dynamic_environment.cpp
 * @brief Implementation of dynamic environment management
 */

#include "explorer/dynamic_environment.h"
#include <cmath>
#include <algorithm>

namespace dynamic_env_ns {

DynamicEnvironmentManager::DynamicEnvironmentManager(ros::NodeHandle& nh) {
    // 从参数服务器读取配置
    nh.param("dynamic/prediction_time_horizon", prediction_time_horizon_, 5.0);
    nh.param("dynamic/safety_margin", safety_margin_, 1.5);
}

void DynamicEnvironmentManager::UpdateDynamicObjects(const std::vector<DynamicObject>& objects) {
    dynamic_objects_ = objects;
}

void DynamicEnvironmentManager::UpdateSemiDynamicObjects(const std::vector<SemiDynamicObject>& objects) {
    semi_dynamic_objects_ = objects;
}

CollisionRisk DynamicEnvironmentManager::AssessViewpointRisk(const geometry_msgs::Point& viewpoint_pos, 
                                                           double time_horizon) const {
    CollisionRisk risk;
    risk.risk_score = 0.0;
    risk.min_distance = std::numeric_limits<double>::max();
    risk.is_critical = false;
    risk.risk_time = ros::Time::now();
    
    if (dynamic_objects_.empty()) {
        return risk;
    }
    
    Eigen::Vector3d viewpoint_eigen(viewpoint_pos.x, viewpoint_pos.y, viewpoint_pos.z);
    
    for (const auto& obj : dynamic_objects_) {
        // 预测物体轨迹
        auto predicted_traj = PredictObjectTrajectory(obj, time_horizon);
        
        double min_obj_distance = std::numeric_limits<double>::max();
        ros::Time closest_time;
        
        // 检查预测轨迹中的每个点
        for (const auto& pred_pose : predicted_traj) {
            Eigen::Vector3d obj_pos(pred_pose.position.x, pred_pose.position.y, pred_pose.position.z);
            double distance = (viewpoint_eigen - obj_pos).norm();
            
            if (distance < min_obj_distance) {
                min_obj_distance = distance;
            }
            
            // 计算碰撞概率
            double collision_prob = CalculateCollisionProbability(viewpoint_pos, obj);
            risk.risk_score = std::max(risk.risk_score, collision_prob);
        }
        
        risk.min_distance = std::min(risk.min_distance, min_obj_distance);
        
        // 如果距离小于安全边界，标记为关键风险
        if (min_obj_distance < obj.collision_radius + safety_margin_) {
            risk.is_critical = true;
        }
    }
    
    return risk;
}

bool DynamicEnvironmentManager::CheckSemiDynamicAccessibility(const geometry_msgs::Point& position) const {
    Eigen::Vector3d pos_eigen(position.x, position.y, position.z);
    
    for (const auto& obj : semi_dynamic_objects_) {
        Eigen::Vector3d obj_pos(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
        double distance = (pos_eigen - obj_pos).norm();
        
        // 检查是否在物体影响范围内
        if (distance < 2.0) { // 影响范围阈值
            // 对于门，检查是否关闭但可能被打开
            if (obj.type == "door" && !obj.is_open && obj.change_probability > 0.3) {
                return true; // 可能变得可通行
            }
            // 对于椅子等可移动物体
            if (obj.type == "chair" && obj.is_movable && obj.change_probability > 0.5) {
                return true;
            }
        }
    }
    
    return false;
}

double DynamicEnvironmentManager::GetEnvironmentDynamicness() const {
    double dynamicness = 0.0;
    
    // 基于动态物体数量和速度
    for (const auto& obj : dynamic_objects_) {
        double speed = std::sqrt(obj.current_velocity.linear.x * obj.current_velocity.linear.x +
                               obj.current_velocity.linear.y * obj.current_velocity.linear.y +
                               obj.current_velocity.linear.z * obj.current_velocity.linear.z);
        dynamicness += speed * 0.5; // 速度贡献
    }
    
    // 基于半动态物体的状态改变概率
    for (const auto& obj : semi_dynamic_objects_) {
        dynamicness += obj.change_probability * 0.3;
    }
    
    return std::min(dynamicness, 1.0); // 归一化到0-1
}

std::vector<geometry_msgs::Pose> 
DynamicEnvironmentManager::PredictObjectTrajectory(const DynamicObject& object, double time_horizon) const {
    std::vector<geometry_msgs::Pose> trajectory;
    
    // 简单线性预测模型
    geometry_msgs::Pose current_pose = object.current_pose;
    
    for (double t = 0; t <= time_horizon; t += 0.1) { // 0.1秒时间步长
        geometry_msgs::Pose predicted_pose = current_pose;
        predicted_pose.position.x += object.current_velocity.linear.x * t;
        predicted_pose.position.y += object.current_velocity.linear.y * t;
        predicted_pose.position.z += object.current_velocity.linear.z * t;
        
        trajectory.push_back(predicted_pose);
    }
    
    return trajectory;
}

double DynamicEnvironmentManager::CalculateCollisionProbability(const geometry_msgs::Point& point, 
                                                              const DynamicObject& object) const {
    Eigen::Vector3d point_eigen(point.x, point.y, point.z);
    Eigen::Vector3d obj_pos(object.current_pose.position.x, 
                          object.current_pose.position.y, 
                          object.current_pose.position.z);
    
    double distance = (point_eigen - obj_pos).norm();
    double relative_speed = std::sqrt(object.current_velocity.linear.x * object.current_velocity.linear.x +
                                    object.current_velocity.linear.y * object.current_velocity.linear.y +
                                    object.current_velocity.linear.z * object.current_velocity.linear.z);
    
    // 基于距离和相对速度的碰撞概率模型
    double distance_factor = std::exp(-distance / (object.collision_radius * 2.0));
    double speed_factor = std::min(relative_speed / 2.0, 1.0); // 假设最大速度2m/s
    
    return distance_factor * speed_factor * object.confidence;
}

} // namespace dynamic_env_ns