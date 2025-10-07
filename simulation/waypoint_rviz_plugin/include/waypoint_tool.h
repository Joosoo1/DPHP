#ifndef WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
#define WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H

#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

#include <QObject>
#include <sstream>

#include "rviz/default_plugin/tools/pose_tool.h"
#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

namespace rviz {
    class StringProperty;

    class WaypointTool : public PoseTool {
        Q_OBJECT
    public:
        WaypointTool();
        virtual ~WaypointTool() {}
        virtual void onInitialize();

    protected:
        virtual void odomHandler(const nav_msgs::Odometry::ConstPtr& odom);
        virtual void onPoseSet(double x, double y, double theta);

    private Q_SLOTS:
        void updateTopic();

    private:
        float vehicle_z;

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
        ros::Publisher pub_joy_;

        StringProperty* topic_property_;
    };
}  // namespace rviz

#endif  // WAYPOINT_RVIZ_PLUGIN_WAYPOINT_TOOL_H
