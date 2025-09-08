/**
 * @file dynamic_environment.h
 * @brief Dynamic object and semi-dynamic object handling for exploration
 */

#pragma once

#include <vector>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

namespace dynamic_env_ns {

// 动态物体预测信息
struct DynamicObject {
    int id;
    geometry_msgs::Pose current_pose;
    geometry_msgs::Twist current_velocity;
    std::vector<geometry_msgs::Pose> predicted_trajectory;  // 未来轨迹预测
    double collision_radius;  // 碰撞半径
    double confidence;  // 预测置信度
    ros::Time last_update_time;
};

// 半动态物体状态
struct SemiDynamicObject {
    int id;
    geometry_msgs::Pose pose;
    std::string type;  // "door", "chair", "cabinet", etc.
    bool is_open;  // 对于门
    bool is_movable;  // 是否可移动
    double change_probability;  // 状态改变概率
    ros::Time last_observed_time;
};

// 碰撞风险评估结果
struct CollisionRisk {
    double risk_score;  // 0-1 风险评分
    double min_distance;  // 最小预测距离
    ros::Time risk_time;  // 风险发生时间
    bool is_critical;  // 是否关键风险
};

class DynamicEnvironmentManager {
public:
    DynamicEnvironmentManager(ros::NodeHandle& nh);
    
    // 动态物体处理
    void UpdateDynamicObjects(const std::vector<DynamicObject>& objects);
    CollisionRisk AssessViewpointRisk(const geometry_msgs::Point& viewpoint_pos, 
                                     double time_horizon = 5.0) const;
    
    // 半动态物体处理
    void UpdateSemiDynamicObjects(const std::vector<SemiDynamicObject>& objects);
    bool CheckSemiDynamicAccessibility(const geometry_msgs::Point& position) const;
    
    // 环境状态查询
    double GetEnvironmentDynamicness() const;  // 环境动态程度评分
    
private:
    std::vector<DynamicObject> dynamic_objects_;
    std::vector<SemiDynamicObject> semi_dynamic_objects_;
    
    // 预测参数
    double prediction_time_horizon_;
    double safety_margin_;
    
    // 预测模型
    std::vector<geometry_msgs::Pose> PredictObjectTrajectory(const DynamicObject& object, 
                                                           double time_horizon) const;
    double CalculateCollisionProbability(const geometry_msgs::Point& point, 
                                       const DynamicObject& object) const;
};

} // namespace dynamic_env_ns