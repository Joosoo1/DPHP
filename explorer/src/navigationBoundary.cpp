#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <math.h>
#include <ros/ros.h>

#include <geometry_msgs/PolygonStamped.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

constexpr double PI = 3.1415926;

string boundary_file_dir;
bool sendBoundary = true;
int sendBoundaryInterval = 2;
int sendBoundaryCount = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr boundary(new pcl::PointCloud<pcl::PointXYZ>());

// reading boundary from file function
void readBoundaryFile()
{
    FILE* boundary_file = fopen(boundary_file_dir.c_str(), "r");
    if (boundary_file == nullptr)
    {
        printf("\nCannot read input files, exit.\n\n");
        exit(1);
    }

    char str[50];
    int pointNum;
    string strCur;
    while (strCur != "end_header")
    {
        int val = fscanf(boundary_file, "%s", str);
        if (val != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        string strLast = strCur;
        strCur = string(str);

        if (strCur == "vertex" && strLast == "element")
        {
            val = fscanf(boundary_file, "%d", &pointNum);
            if (val != 1)
            {
                printf("\nError reading input files, exit.\n\n");
                exit(1);
            }
        }
    }

    boundary->clear();
    pcl::PointXYZ point;
    for (int i = 0; i < pointNum; i++)
    {
        const int val1 = fscanf(boundary_file, "%f", &point.x);
        const int val2 = fscanf(boundary_file, "%f", &point.y);
        const int val3 = fscanf(boundary_file, "%f", &point.z);

        if (val1 != 1 || val2 != 1 || val3 != 1)
        {
            printf("\nError reading input files, exit.\n\n");
            exit(1);
        }

        point.z = 0;
        boundary->push_back(point);
    }

    if (boundary->points[0].x != boundary->points[pointNum - 1].x ||
        boundary->points[0].y != boundary->points[pointNum - 1].y)
    {
        boundary->push_back(boundary->points[0]);
    }

    fclose(boundary_file);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigationBoundary");
    ros::NodeHandle nh;
    auto nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("boundary_file_dir", boundary_file_dir);
    nhPrivate.getParam("sendBoundary", sendBoundary);
    nhPrivate.getParam("sendBoundaryInterval", sendBoundaryInterval);

    ros::Publisher pubBoundary = nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary", 5);
    geometry_msgs::PolygonStamped boundaryMsgs;
    boundaryMsgs.header.frame_id = "map";

    // read boundary from file
    if (sendBoundary)
    {
        readBoundaryFile();

        int boundarySize = boundary->points.size();
        boundaryMsgs.polygon.points.resize(boundarySize);
        for (int i = 0; i < boundarySize; i++)
        {
            boundaryMsgs.polygon.points[i].x = boundary->points[i].x;
            boundaryMsgs.polygon.points[i].y = boundary->points[i].y;
            boundaryMsgs.polygon.points[i].z = boundary->points[i].z;
        }
    }

    ros::Rate rate(100);
    bool status = ros::ok();
    while (status)
    {
        ros::spinOnce();

        // publish boundary messages at certain frame rate
        sendBoundaryCount++;
        if (sendBoundaryCount >= 100 * sendBoundaryInterval && sendBoundary)
        {
            pubBoundary.publish(boundaryMsgs);
            sendBoundaryCount = 0;
        }

        status = ros::ok();
        rate.sleep();
    }

    return 0;
}
