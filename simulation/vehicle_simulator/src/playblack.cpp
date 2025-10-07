#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <algorithm>
#include <fstream>

struct TrajectoryPoint {
    double x, y, z;
    double roll, pitch, yaw;
    double timestamp;
};

class MapVisualization {
private:
    ros::NodeHandle nh;
    ros::Publisher map_pub;
    ros::Publisher trajectory_marker_pub;  // 修改为轨迹标记发布器
    ros::Publisher start_marker_pub;
    ros::Publisher end_marker_pub;

    std::string map_file_path;
    std::string traj_file_path;
    double publish_rate;

    geometry_msgs::Point start_point;
    geometry_msgs::Point end_point;

    sensor_msgs::PointCloud2 map_msg_;
    visualization_msgs::Marker trajectory_marker_;  // 轨迹标记
    visualization_msgs::Marker start_marker_, end_marker_;

public:
    MapVisualization() {
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("map_file", map_file_path, "");
        private_nh.param<std::string>("traj_file", traj_file_path, "");
        private_nh.param<double>("publish_rate", publish_rate, 1.0);

        map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1, true);
        trajectory_marker_pub =
            nh.advertise<visualization_msgs::Marker>("trajectory_marker", 1, true);
        start_marker_pub = nh.advertise<visualization_msgs::Marker>("start_marker", 1, true);
        end_marker_pub = nh.advertise<visualization_msgs::Marker>("end_marker", 1, true);
    }

    bool loadAndPublishMap() {
        if (map_file_path.empty()) {
            ROS_WARN("No map file specified");
            return false;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPLYFile<pcl::PointXYZ>(map_file_path, *cloud) == -1) {
            ROS_ERROR("Failed to load PLY file: %s", map_file_path.c_str());
            return false;
        }

        pcl::toROSMsg(*cloud, map_msg_);
        map_msg_.header.frame_id = "map";
        map_msg_.header.stamp = ros::Time::now();
        ROS_INFO("Published %zu map points", cloud->size());
        return true;
    }

    bool loadAndPublishTrajectory() {
        if (traj_file_path.empty()) {
            ROS_WARN("No trajectory file specified");
            return false;
        }

        std::ifstream file(traj_file_path);
        if (!file.is_open()) {
            ROS_ERROR("Failed to open trajectory file: %s", traj_file_path.c_str());
            return false;
        }

        std::vector<TrajectoryPoint> trajectory_points;
        std::string line;
        while (std::getline(file, line)) {
            TrajectoryPoint pt;
            std::istringstream iss(line);
            if (iss >> pt.x >> pt.y >> pt.z >> pt.roll >> pt.pitch >> pt.yaw >> pt.timestamp) {
                trajectory_points.push_back(pt);
            }
        }

        if (trajectory_points.empty()) {
            ROS_WARN("No trajectory points loaded");
            return false;
        }

        // 计算速度
        std::vector<double> speeds(trajectory_points.size(), 0.0);
        if (trajectory_points.size() > 1) {
            for (size_t i = 1; i < trajectory_points.size(); ++i) {
                const auto& prev = trajectory_points[i - 1];
                const auto& curr = trajectory_points[i];
                double dt = curr.timestamp - prev.timestamp;
                if (dt <= 0) {
                    speeds[i] = speeds[i - 1];
                    continue;
                }
                double dx = curr.x - prev.x;
                double dy = curr.y - prev.y;
                double dz = curr.z - prev.z;
                double distance = sqrt(dx * dx + dy * dy + dz * dz);
                speeds[i] = distance / dt;
            }
            speeds[0] = speeds[1];  // 首点速度用第二个点的速度
        }

        // 计算颜色映射
        auto [min_speed, max_speed] = std::minmax_element(speeds.begin(), speeds.end());
        double speed_range = *max_speed - *min_speed;
        if (speed_range < 1e-6) speed_range = 1e-6;  // 防止除以零

        // 配置轨迹标记
        trajectory_marker_.header.frame_id = "map";
        trajectory_marker_.header.stamp = ros::Time::now();
        trajectory_marker_.ns = "trajectory";
        trajectory_marker_.id = 0;
        trajectory_marker_.type = visualization_msgs::Marker::LINE_STRIP;
        trajectory_marker_.action = visualization_msgs::Marker::ADD;
        trajectory_marker_.scale.x = 0.3;  // 线宽
        trajectory_marker_.pose.orientation.w = 1.0;

        // 填充点和颜色
        trajectory_marker_.points.clear();
        trajectory_marker_.colors.clear();
        for (size_t i = 0; i < trajectory_points.size(); ++i) {
            // 添加点坐标
            geometry_msgs::Point p;
            p.x = trajectory_points[i].x;
            p.y = trajectory_points[i].y;
            p.z = trajectory_points[i].z;
            trajectory_marker_.points.push_back(p);

            // 计算颜色（蓝->绿->红渐变）
            std_msgs::ColorRGBA color;
            double ratio = (speeds[i] - *min_speed) / speed_range;
            color.r = std::min(1.0, ratio * 2.0);  // 红通道：0~1当ratio在0~0.5时
            color.g = std::min(1.0, (1.0 - fabs(ratio - 0.5) * 2.0));  // 绿通道：峰值在中间
            color.b = std::max(0.0, 1.0 - ratio * 2.0);  // 蓝通道：1~0当ratio在0~0.5时
            color.a = 1.0;
            trajectory_marker_.colors.push_back(color);
        }

        // 设置起点终点
        start_point.x = trajectory_points.front().x;
        start_point.y = trajectory_points.front().y;
        start_point.z = trajectory_points.front().z;
        end_point.x = trajectory_points.back().x;
        end_point.y = trajectory_points.back().y;
        end_point.z = trajectory_points.back().z;

        createStartMarker();
        createEndMarker();
        return true;
    }

    void createStartMarker() {
        start_marker_.header.frame_id = "map";
        start_marker_.header.stamp = ros::Time::now();
        start_marker_.ns = "markers";
        start_marker_.id = 1;
        start_marker_.type = visualization_msgs::Marker::SPHERE;
        start_marker_.scale.x = start_marker_.scale.y = start_marker_.scale.z = 1.0;
        start_marker_.color = getColor(0.0);  // 使用速度颜色映射的起点颜色
        start_marker_.pose.position = start_point;
    }

    void createEndMarker() {
        end_marker_.header.frame_id = "map";
        end_marker_.header.stamp = ros::Time::now();
        end_marker_.ns = "markers";
        end_marker_.id = 2;
        end_marker_.type = visualization_msgs::Marker::SPHERE;
        end_marker_.scale.x = end_marker_.scale.y = end_marker_.scale.z = 1.0;
        end_marker_.color = getColor(1.0);  // 使用速度颜色映射的终点颜色
        end_marker_.pose.position = end_point;
    }

    std_msgs::ColorRGBA getColor(double ratio) const {
        std_msgs::ColorRGBA color;
        color.r = std::min(1.0, ratio * 2.0);
        color.g = std::min(1.0, (1.0 - fabs(ratio - 0.5) * 2.0));
        color.b = std::max(0.0, 1.0 - ratio * 2.0);
        color.a = 1.0;
        return color;
    }

    void run() {
        bool map_loaded = loadAndPublishMap();
        bool traj_loaded = loadAndPublishTrajectory();

        ros::Rate rate(publish_rate);
        while (ros::ok()) {
            if (map_loaded) map_pub.publish(map_msg_);
            if (traj_loaded) {
                trajectory_marker_pub.publish(trajectory_marker_);
                start_marker_pub.publish(start_marker_);
                end_marker_pub.publish(end_marker_);
            }
            rate.sleep();
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_visualization_node");
    MapVisualization visualizer;
    visualizer.run();
    return 0;
}