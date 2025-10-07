#include <dynamic_reconfigure/server.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "vehicle_simulator/MapConfig.h"

using PointT = pcl::PointXYZI;
pcl::PointCloud<PointT>::Ptr global_map(new pcl::PointCloud<PointT>);
pcl::VoxelGrid<PointT> voxel_filter;
boost::mutex map_mutex;

class PCLMapGenerator {
public:
    PCLMapGenerator(ros::NodeHandle& nh) : nh_(nh) {
        // 初始化订阅者
        sub_ = nh_.subscribe("/robot/dlio/odom_node/pointcloud/deskewed", 10,
                             &PCLMapGenerator::cloudCallback, this);

        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/global_map", 1);

        // 动态参数服务
        dyn_server_.setCallback(boost::bind(&PCLMapGenerator::configCallback, this, _1, _2));

        // 初始化滤波器
        voxel_filter.setLeafSize(0.1, 0.1, 0.1);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
        pcl::fromROSMsg(*msg, *cloud);

        // 数据累积
        boost::mutex::scoped_lock lock(map_mutex);
        *global_map += *cloud;

        // 实时降采样
        pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>);
        voxel_filter.setInputCloud(global_map);
        voxel_filter.filter(*filtered);
        global_map->swap(*filtered);

        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*global_map, map_msg);
        map_msg.header.stamp = ros::Time::now();  // 设置当前时间戳[1](@ref)
        map_msg.header.frame_id = "map";          // 设置坐标系[1](@ref)
        map_pub_.publish(map_msg);
    }

    void configCallback(vehicle_simulator::MapConfig& config, uint32_t level) {
        // 动态调整体素尺寸 (0.01m~1.0m)
        voxel_filter.setLeafSize(config.voxel_size, config.voxel_size, config.voxel_size);

        // 自动保存地图
        if (config.save_trigger) {
            savePLYMap(config.file_path);
            config.save_trigger = false;  // 重置触发器
        }
    }

    void savePLYMap(const std::string& path) {
        boost::mutex::scoped_lock lock(map_mutex);
        pcl::PLYWriter writer;
        writer.write(path, *global_map, true);
        ROS_INFO_STREAM("Map saved to: " << path);
    }

    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    ros::Publisher map_pub_;
    dynamic_reconfigure::Server<vehicle_simulator::MapConfig> dyn_server_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcl_map_generator");
    ros::NodeHandle nh("~");
    PCLMapGenerator node(nh);
    ros::spin();
    return 0;
}