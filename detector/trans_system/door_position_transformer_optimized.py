#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
门位置转换器 - 优化版本
将相机坐标系下的门位置转换到全局坐标系
- 优化机器人运动时的坐标转换
- 添加坐标稳定性验证
- 改进TF变换处理
"""

import rospy
import json
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseArray, Pose, PointStamped
from std_msgs.msg import Header

class DoorPositionTransformer:
    def __init__(self):
        """初始化门位置转换器"""
        rospy.init_node('door_position_transformer', anonymous=True)
        
        # 坐标系名称
        self.camera_frame = 'camera_color_optical_frame'  # 相机坐标系
        self.lidar_frame = 'robot/lidar'  # 激光雷达坐标系
        self.odom_frame = 'odom'  # 里程计坐标系
        self.map_frame = 'robot/odom'  # 地图坐标系
        
        rospy.loginfo(f"相机坐标系: {self.camera_frame}")
        rospy.loginfo(f"激光雷达坐标系: {self.lidar_frame}")
        rospy.loginfo(f"里程计坐标系: {self.odom_frame}")
        rospy.loginfo(f"地图坐标系: {self.map_frame}")
        
        # 加载标定参数
        self.load_calibration_params()
        
        # 初始化TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # TF变换状态
        self.tf_ready = False
        self.tf_check_count = 0
        self.max_tf_checks = 10  # 最多检查10次
        
        # 创建发布器和订阅器
        self.door_positions_sub = rospy.Subscriber('/doors/camera_positions', PoseArray, self.door_detection_callback)
        self.global_door_positions_pub = rospy.Publisher('/doors/global_positions', PoseArray, queue_size=10)
        
        # 坐标转换优化参数
        self.transform_timeout = 0.5  # TF变换超时时间
        self.max_transform_attempts = 3  # 最大变换尝试次数
        self.coordinate_stability_threshold = 0.1  # 坐标稳定性阈值（米）
        self.published_doors_global = set()  # 已发布的全局门位置ID
        
        rospy.loginfo("✅ 门位置转换器初始化完成")
        rospy.loginfo("🚪 门位置转换器启动")
        rospy.loginfo("等待门检测结果...")
        
        # 启动TF检查定时器
        self.tf_check_timer = rospy.Timer(rospy.Duration(1.0), self.check_tf_availability)
    
    def load_calibration_params(self):
        """加载标定参数"""
        try:
            with open('/home/xu/DPHP/src/predict/trans_system/calib.json', 'r') as f:
                calib_data = json.load(f)
            
            # 提取T_lidar_camera参数 (x, y, z, qx, qy, qz, qw)
            T_params = calib_data['results']['T_lidar_camera']
            
            # 分离平移和旋转
            self.t_lidar_camera = np.array(T_params[:3])  # [x, y, z]
            self.q_lidar_camera = np.array(T_params[3:])  # [qx, qy, qz, qw]
            
            # 转换为变换矩阵
            self.T_lidar_camera = self.quaternion_to_matrix(self.t_lidar_camera, self.q_lidar_camera)
            
            rospy.loginfo("✅ 标定参数加载成功")
            rospy.loginfo(f"平移: {self.t_lidar_camera}")
            rospy.loginfo(f"四元数: {self.q_lidar_camera}")
            
        except Exception as e:
            rospy.logerr(f"❌ 加载标定参数失败: {e}")
            # 使用默认参数
            self.t_lidar_camera = np.array([0.0, 0.0, 0.0])
            self.q_lidar_camera = np.array([0.0, 0.0, 0.0, 1.0])
            self.T_lidar_camera = np.eye(4)
            rospy.logwarn("使用默认标定参数")
    
    def quaternion_to_matrix(self, translation, quaternion):
        """将平移向量和四元数转换为4x4变换矩阵"""
        # 四元数 [qx, qy, qz, qw]
        qx, qy, qz, qw = quaternion
        
        # 归一化四元数
        norm = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
        
        # 转换为旋转矩阵
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ])
        
        # 创建4x4变换矩阵
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = translation
        
        return T
    
    def check_tf_availability(self, event):
        """检查TF变换是否可用"""
        if self.tf_ready:
            return
            
        self.tf_check_count += 1
        
        try:
            # 尝试获取变换
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                self.lidar_frame, 
                rospy.Time(0), 
                rospy.Duration(0.1)
            )
            
            self.tf_ready = True
            rospy.loginfo("✅ TF变换已就绪，可以开始坐标转换")
            self.tf_check_timer.shutdown()  # 停止检查定时器
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            if self.tf_check_count <= self.max_tf_checks:
                rospy.loginfo(f"⏳ 等待TF变换就绪... ({self.tf_check_count}/{self.max_tf_checks})")
            else:
                rospy.logwarn("⚠️ TF变换检查超时，将使用默认处理")
                self.tf_check_timer.shutdown()
    
    def transform_point_camera_to_lidar(self, point_camera):
        """将相机坐标系下的点转换到激光雷达坐标系"""
        # 将点转换为齐次坐标
        point_homo = np.array([point_camera[0], point_camera[1], point_camera[2], 1.0])
        
        # 应用变换矩阵
        point_lidar_homo = self.T_lidar_camera @ point_homo
        
        # 返回3D坐标
        return point_lidar_homo[:3]
    
    def transform_point_lidar_to_global(self, point_lidar, target_frame='robot/odom'):
        """将激光雷达坐标系下的点转换到全局坐标系（优化版本）"""
        if not self.tf_ready:
            rospy.logwarn("TF变换尚未就绪，跳过坐标转换")
            return None
            
        # 多次尝试获取稳定的变换
        for attempt in range(self.max_transform_attempts):
            try:
                # 创建PointStamped消息
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.lidar_frame
                point_stamped.header.stamp = rospy.Time.now()
                point_stamped.point.x = point_lidar[0]
                point_stamped.point.y = point_lidar[1]
                point_stamped.point.z = point_lidar[2]
                
                # 等待变换可用
                transform = self.tf_buffer.lookup_transform(
                    target_frame, 
                    self.lidar_frame, 
                    rospy.Time(0), 
                    rospy.Duration(self.transform_timeout)
                )
                
                # 应用变换
                point_transformed = tf2_geometry_msgs.do_transform_point(point_stamped, transform)
                
                # 验证变换结果的合理性
                transformed_point = [point_transformed.point.x, point_transformed.point.y, point_transformed.point.z]
                
                # 检查坐标是否在合理范围内
                if self.is_coordinate_reasonable(transformed_point):
                    return transformed_point
                else:
                    rospy.logwarn(f"变换结果不合理，尝试 {attempt + 1}/{self.max_transform_attempts}")
                    rospy.sleep(0.1)  # 短暂等待后重试
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF变换失败 (尝试 {attempt + 1}/{self.max_transform_attempts}): {e}")
                if attempt < self.max_transform_attempts - 1:
                    rospy.sleep(0.1)  # 短暂等待后重试
        
        rospy.logwarn("所有TF变换尝试均失败")
        return None
    
    def is_coordinate_reasonable(self, point):
        """检查坐标是否在合理范围内"""
        x, y, z = point
        
        # 检查坐标是否在合理范围内（根据实际环境调整）
        if abs(x) > 100 or abs(y) > 100 or abs(z) > 100:
            return False
        
        # 检查是否有NaN或无穷大值
        if np.isnan(x) or np.isnan(y) or np.isnan(z):
            return False
        
        if np.isinf(x) or np.isinf(y) or np.isinf(z):
            return False
        
        return True
    
    def generate_global_door_id(self, global_position):
        """根据全局位置生成门ID"""
        x, y, z = global_position
        # 使用位置网格生成ID（1米精度）
        grid_x = int(round(x))
        grid_y = int(round(y))
        grid_z = int(round(z))
        return f"door_{grid_x}_{grid_y}_{grid_z}"
    
    def door_detection_callback(self, msg):
        """门检测结果回调函数（优化版本）"""
        if not msg.poses:
            return
        
        rospy.loginfo(f"接收到 {len(msg.poses)} 个门的位置")
        
        global_door_positions = []
        published_count = 0
        
        for i, pose in enumerate(msg.poses):
            # 提取相机坐标系下的位置
            camera_position = [pose.position.x, pose.position.y, pose.position.z]
            
            # 转换到激光雷达坐标系
            lidar_position = self.transform_point_camera_to_lidar(camera_position)
            
            # 转换到全局坐标系
            global_position = self.transform_point_lidar_to_global(lidar_position, self.map_frame)
            
            if global_position is not None:
                # 生成全局门ID
                global_door_id = self.generate_global_door_id(global_position)
                
                # 检查是否已经发布过这个门
                if global_door_id in self.published_doors_global:
                    rospy.loginfo(f"门 {i+1} 已发布过，跳过 (ID: {global_door_id})")
                    continue
                
                # 创建全局坐标系下的门位置
                global_pose = Pose()
                global_pose.position.x = global_position[0]
                global_pose.position.y = global_position[1]
                global_pose.position.z = 0.25  # 固定Z轴为0.25m
                global_pose.orientation = pose.orientation  # 保持原始方向
                
                global_door_positions.append(global_pose)
                
                # 标记为已发布
                self.published_doors_global.add(global_door_id)
                published_count += 1
                
                rospy.loginfo(f"门 {i+1} 坐标转换成功:")
                rospy.loginfo(f"  相机坐标系: {camera_position}")
                rospy.loginfo(f"  激光雷达坐标系: {lidar_position}")
                rospy.loginfo(f"  全局坐标系: [{global_position[0]:.3f}, {global_position[1]:.3f}, 0.250] (Z固定)")
                rospy.loginfo(f"  门ID: {global_door_id}")
            else:
                rospy.logwarn(f"门 {i+1} 坐标转换失败，跳过")
        
        # 发布全局坐标系下的门位置
        if global_door_positions:
            global_pose_array = PoseArray()
            global_pose_array.header.frame_id = self.map_frame
            global_pose_array.header.stamp = rospy.Time.now()
            global_pose_array.poses = global_door_positions
            
            self.global_door_positions_pub.publish(global_pose_array)
            rospy.loginfo(f"📡 发布 {len(global_door_positions)} 个门的全局位置到 /doors/global_positions")
            rospy.loginfo(f"已发布门总数: {len(self.published_doors_global)}")

def main():
    """主函数"""
    try:
        transformer = DoorPositionTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"❌ 发生错误: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
