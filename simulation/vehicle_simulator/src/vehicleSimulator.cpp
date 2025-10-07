#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <time.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

const double PI = 3.1415926;

bool use_gazebo_time = false;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
double vehicleHeight = 0.75;
double terrainVoxelSize = 0.05;
double groundHeightThre = 0.1;
bool adjustZ = false;
double terrainRadiusZ = 0.5;
int minTerrainPointNumZ = 10;
double smoothRateZ = 0.2;
bool adjustIncl = false;
double terrainRadiusIncl = 1.5;
int minTerrainPointNumIncl = 500;
double smoothRateIncl = 0.2;
double InclFittingThre = 0.2;
double maxIncl = 30.0;

const int systemDelay = 5;
int systemInitCount = 0;
bool systemInited = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr scanData(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudIncl(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());

std::vector<int> scanInd;

ros::Time odomTime;

double realtimeFactor = 1.0;
double windCoeff = 0.05;
double maxRollPitchRate = 20.0;
double rollPitchSmoothRate = 0.1;
double sensorPitch = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 1.0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;
float vehicleVelXG = 0;
float vehicleVelYG = 0;
float vehicleVelZG = 0;

float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleRollCmd = 0;
float vehiclePitchCmd = 0;
float vehicleYawRate = 0;

float terrainZ = 0;
float terrainRoll = 0;
float terrainPitch = 0;

const int stackNum = 400;
float vehicleXStack[stackNum];
float vehicleYStack[stackNum];
float vehicleZStack[stackNum];
float vehicleRollStack[stackNum];
float vehiclePitchStack[stackNum];
float vehicleYawStack[stackNum];
float terrainRollStack[stackNum];
float terrainPitchStack[stackNum];
double odomTimeStack[stackNum];
int odomSendIDPointer = -1;
int odomRecIDPointer = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;

ros::Publisher* pubScanPointer = NULL;

void scanHandler(const sensor_msgs::PointCloud2::ConstPtr& scanIn) {
    if (!systemInited) {
        systemInitCount++;
        if (systemInitCount > systemDelay) { systemInited = true; }
        return;
    }

    double scanTime = scanIn->header.stamp.toSec();

    if (odomSendIDPointer < 0) { return; }
    while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime
           && odomRecIDPointer != (odomSendIDPointer + 1) % stackNum) {
        odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
    }

    double odomRecTime = odomTime.toSec();
    float vehicleRecX = vehicleX;
    float vehicleRecY = vehicleY;
    float vehicleRecZ = vehicleZ;
    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecYaw = vehicleYaw;
    float terrainRecRoll = terrainRoll;
    float terrainRecPitch = terrainPitch;

    if (use_gazebo_time) {
        odomRecTime = odomTimeStack[odomRecIDPointer];
        vehicleRecX = vehicleXStack[odomRecIDPointer];
        vehicleRecY = vehicleYStack[odomRecIDPointer];
        vehicleRecZ = vehicleZStack[odomRecIDPointer];
        vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
        vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
        vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
        terrainRecRoll = terrainRollStack[odomRecIDPointer];
        terrainRecPitch = terrainPitchStack[odomRecIDPointer];
    }

    float sinTerrainRecRoll = sin(terrainRecRoll);
    float cosTerrainRecRoll = cos(terrainRecRoll);
    float sinTerrainRecPitch = sin(terrainRecPitch);
    float cosTerrainRecPitch = cos(terrainRecPitch);

    scanData->clear();
    pcl::fromROSMsg(*scanIn, *scanData);
    pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

    int scanDataSize = scanData->points.size();
    for (int i = 0; i < scanDataSize; i++) {
        float pointX1 = scanData->points[i].x;
        float pointY1 =
            scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
        float pointZ1 =
            scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

        float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
        float pointY2 = pointY1;
        float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

        float pointX3 = pointX2 + vehicleRecX;
        float pointY3 = pointY2 + vehicleRecY;
        float pointZ3 = pointZ2 + vehicleRecZ;

        scanData->points[i].x = pointX3;
        scanData->points[i].y = pointY3;
        scanData->points[i].z = pointZ3;
    }

    // publish 5Hz registered scan messages
    sensor_msgs::PointCloud2 scanData2;
    pcl::toROSMsg(*scanData, scanData2);
    scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
    scanData2.header.frame_id = "map";
    pubScanPointer->publish(scanData2);
}

void scanHandler1(const sensor_msgs::PointCloud::ConstPtr& scanIn) {
    if (!systemInited) {
        systemInitCount++;
        if (systemInitCount > systemDelay) { systemInited = true; }
        return;
    }

    double scanTime = scanIn->header.stamp.toSec();

    if (odomSendIDPointer < 0) { return; }
    while (odomTimeStack[(odomRecIDPointer + 1) % stackNum] < scanTime
           && odomRecIDPointer != (odomSendIDPointer + 1) % stackNum) {
        odomRecIDPointer = (odomRecIDPointer + 1) % stackNum;
    }

    double odomRecTime = odomTime.toSec();
    float vehicleRecX = vehicleX;
    float vehicleRecY = vehicleY;
    float vehicleRecZ = vehicleZ;
    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;
    float vehicleRecYaw = vehicleYaw;
    float terrainRecRoll = terrainRoll;
    float terrainRecPitch = terrainPitch;

    if (use_gazebo_time) {
        odomRecTime = odomTimeStack[odomRecIDPointer];
        vehicleRecX = vehicleXStack[odomRecIDPointer];
        vehicleRecY = vehicleYStack[odomRecIDPointer];
        vehicleRecZ = vehicleZStack[odomRecIDPointer];
        vehicleRecRoll = vehicleRollStack[odomRecIDPointer];
        vehicleRecPitch = vehiclePitchStack[odomRecIDPointer];
        vehicleRecYaw = vehicleYawStack[odomRecIDPointer];
        terrainRecRoll = terrainRollStack[odomRecIDPointer];
        terrainRecPitch = terrainPitchStack[odomRecIDPointer];
    }

    float sinTerrainRecRoll = sin(terrainRecRoll);
    float cosTerrainRecRoll = cos(terrainRecRoll);
    float sinTerrainRecPitch = sin(terrainRecPitch);
    float cosTerrainRecPitch = cos(terrainRecPitch);

    // Convert PointCloud to PCL PointCloudXYZ
    scanData->clear();
    scanData->points.resize(scanIn->points.size());

    for (size_t i = 0; i < scanIn->points.size(); ++i) {
        scanData->points[i].x = scanIn->points[i].x;
        scanData->points[i].y = scanIn->points[i].y;
        scanData->points[i].z = scanIn->points[i].z;
    }

    pcl::removeNaNFromPointCloud(*scanData, *scanData, scanInd);

    int scanDataSize = scanData->points.size();
    for (int i = 0; i < scanDataSize; i++) {
        float pointX1 = scanData->points[i].x;
        float pointY1 =
            scanData->points[i].y * cosTerrainRecRoll - scanData->points[i].z * sinTerrainRecRoll;
        float pointZ1 =
            scanData->points[i].y * sinTerrainRecRoll + scanData->points[i].z * cosTerrainRecRoll;

        float pointX2 = pointX1 * cosTerrainRecPitch + pointZ1 * sinTerrainRecPitch;
        float pointY2 = pointY1;
        float pointZ2 = -pointX1 * sinTerrainRecPitch + pointZ1 * cosTerrainRecPitch;

        float pointX3 = pointX2 + vehicleRecX;
        float pointY3 = pointY2 + vehicleRecY;
        float pointZ3 = pointZ2 + vehicleRecZ;

        scanData->points[i].x = pointX3;
        scanData->points[i].y = pointY3;
        scanData->points[i].z = pointZ3;
    }

    // publish 5Hz registered scan messages
    sensor_msgs::PointCloud2 scanData2;
    pcl::toROSMsg(*scanData, scanData2);
    scanData2.header.stamp = ros::Time().fromSec(odomRecTime);
    scanData2.header.frame_id = "map";
    pubScanPointer->publish(scanData2);
}

void terrainCloudHandler(const sensor_msgs::PointCloud2ConstPtr& terrainCloud2) {
    if (!adjustZ && !adjustIncl) { return; }

    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudIncl->clear();
    int terrainCloudSize = terrainCloud->points.size();
    double elevMean = 0;
    int elevCount = 0;
    bool terrainValid = true;
    for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];

        float dis = sqrt((point.x - vehicleX) * (point.x - vehicleX)
                         + (point.y - vehicleY) * (point.y - vehicleY));

        if (dis < terrainRadiusZ) {
            if (point.intensity < groundHeightThre) {
                elevMean += point.z;
                elevCount++;
            } else {
                terrainValid = false;
            }
        }

        if (dis < terrainRadiusIncl && point.intensity < groundHeightThre) {
            terrainCloudIncl->push_back(point);
        }
    }

    if (elevCount >= minTerrainPointNumZ)
        elevMean /= elevCount;
    else
        terrainValid = false;

    if (terrainValid && adjustZ) {
        terrainZ = (1.0 - smoothRateZ) * terrainZ + smoothRateZ * elevMean;
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudIncl);
    terrainDwzFilter.filter(*terrainCloudDwz);
    int terrainCloudDwzSize = terrainCloudDwz->points.size();

    if (terrainCloudDwzSize < minTerrainPointNumIncl || !terrainValid) { return; }

    cv::Mat matA(terrainCloudDwzSize, 2, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(2, terrainCloudDwzSize, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(2, 2, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(terrainCloudDwzSize, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(2, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(2, 1, CV_32F, cv::Scalar::all(0));

    int inlierNum = 0;
    matX.at<float>(0, 0) = terrainPitch;
    matX.at<float>(1, 0) = terrainRoll;
    for (int iterCount = 0; iterCount < 5; iterCount++) {
        int outlierCount = 0;
        for (int i = 0; i < terrainCloudDwzSize; i++) {
            point = terrainCloudDwz->points[i];

            matA.at<float>(i, 0) = -point.x + vehicleX;
            matA.at<float>(i, 1) = point.y - vehicleY;
            matB.at<float>(i, 0) = point.z - elevMean;

            if (fabs(matA.at<float>(i, 0) * matX.at<float>(0, 0)
                     + matA.at<float>(i, 1) * matX.at<float>(1, 0) - matB.at<float>(i, 0))
                    > InclFittingThre
                && iterCount > 0) {
                matA.at<float>(i, 0) = 0;
                matA.at<float>(i, 1) = 0;
                matB.at<float>(i, 0) = 0;
                outlierCount++;
            }
        }

        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (inlierNum == terrainCloudDwzSize - outlierCount) break;
        inlierNum = terrainCloudDwzSize - outlierCount;
    }

    if (inlierNum < minTerrainPointNumIncl || fabs(matX.at<float>(0, 0)) > maxIncl * PI / 180.0
        || fabs(matX.at<float>(1, 0)) > maxIncl * PI / 180.0) {
        terrainValid = false;
    }

    if (terrainValid && adjustIncl) {
        terrainPitch =
            (1.0 - smoothRateIncl) * terrainPitch + smoothRateIncl * matX.at<float>(0, 0);
        terrainRoll = (1.0 - smoothRateIncl) * terrainRoll + smoothRateIncl * matX.at<float>(1, 0);
    }
}

void controlHandler(const geometry_msgs::TwistStamped::ConstPtr& controlIn) {
    vehicleRollCmd = controlIn->twist.linear.x;
    vehiclePitchCmd = controlIn->twist.linear.y;
    vehicleYawRate = controlIn->twist.angular.z;
    vehicleVelZG = controlIn->twist.linear.z;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vehicleSimulator");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");

    nhPrivate.getParam("use_gazebo_time", use_gazebo_time);
    nhPrivate.getParam("sensorOffsetX", sensorOffsetX);
    nhPrivate.getParam("sensorOffsetY", sensorOffsetY);
    nhPrivate.getParam("vehicleHeight", vehicleHeight);

    // for uav
    nhPrivate.getParam("windCoeff", windCoeff);
    nhPrivate.getParam("maxRollPitchRate", maxRollPitchRate);
    nhPrivate.getParam("rollPitchSmoothRate", rollPitchSmoothRate);
    nhPrivate.getParam("sensorPitch", sensorPitch);
    nhPrivate.getParam("vehicleX", vehicleX);
    nhPrivate.getParam("vehicleY", vehicleY);
    nhPrivate.getParam("vehicleZ", vehicleZ);
    nhPrivate.getParam("vehicleYaw", vehicleYaw);

    nhPrivate.getParam("terrainZ", terrainZ);
    nhPrivate.getParam("terrainVoxelSize", terrainVoxelSize);
    nhPrivate.getParam("groundHeightThre", groundHeightThre);
    nhPrivate.getParam("adjustZ", adjustZ);
    nhPrivate.getParam("terrainRadiusZ", terrainRadiusZ);
    nhPrivate.getParam("minTerrainPointNumZ", minTerrainPointNumZ);
    nhPrivate.getParam("adjustIncl", adjustIncl);
    nhPrivate.getParam("terrainRadiusIncl", terrainRadiusIncl);
    nhPrivate.getParam("minTerrainPointNumIncl", minTerrainPointNumIncl);
    nhPrivate.getParam("InclFittingThre", InclFittingThre);
    nhPrivate.getParam("maxIncl", maxIncl);

    // ros::Subscriber subScan = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 2, scanHandler);

    ros::Subscriber subScan1 = nh.subscribe<sensor_msgs::PointCloud>("/scan", 2, scanHandler1);

    ros::Subscriber subTerrainCloud =
        nh.subscribe<sensor_msgs::PointCloud2>("/terrain_map", 2, terrainCloudHandler);

    // for uav control
    ros::Subscriber subControl =
        nh.subscribe<geometry_msgs::TwistStamped>("/attitude_control", 5, controlHandler);

    ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry>("/state_estimation", 5);

    nav_msgs::Odometry odomData;
    odomData.header.frame_id = "map";
    odomData.child_frame_id = "sensor";

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform odomTrans;
    odomTrans.frame_id_ = "map";
    odomTrans.child_frame_id_ = "sensor";

    ros::Publisher pubModelState =
        nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 5);
    gazebo_msgs::ModelState cameraState;
    cameraState.model_name = "rgbd_camera";
    // gazebo_msgs::ModelState lidarState;
    // lidarState.model_name = "lidar";
    gazebo_msgs::ModelState robotState;
    robotState.model_name = "robot";
    gazebo_msgs::ModelState livoxState;
    livoxState.model_name = "livox";

    ros::Publisher pubScan = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 2);
    pubScanPointer = &pubScan;

    terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

    printf("\nSimulation started.\n\n");

    ros::Rate rate(200);
    bool status = ros::ok();
    while (status) {
        ros::spinOnce();

        float vehicleRecRoll = vehicleRoll;
        float vehicleRecPitch = vehiclePitch;

        if (vehicleRollCmd - vehicleRoll > maxRollPitchRate / 200.0)
            vehicleRoll += maxRollPitchRate / 200.0;
        else if (vehicleRollCmd - vehicleRoll < -maxRollPitchRate / 200.0)
            vehicleRoll -= maxRollPitchRate / 200.0;
        else
            vehicleRoll = vehicleRollCmd;
        vehicleRoll =
            rollPitchSmoothRate * vehicleRoll + (1.0 - rollPitchSmoothRate) * vehicleRecRoll;

        if (vehiclePitchCmd - vehiclePitch > maxRollPitchRate / 200.0)
            vehiclePitch += maxRollPitchRate / 200.0;
        else if (vehiclePitchCmd - vehiclePitch < -maxRollPitchRate / 200.0)
            vehiclePitch -= maxRollPitchRate / 200.0;
        else
            vehiclePitch = vehiclePitchCmd;
        vehiclePitch =
            rollPitchSmoothRate * vehiclePitch + (1.0 - rollPitchSmoothRate) * vehicleRecPitch;

        float vehicleAccX = 9.8 * tan(vehiclePitch);
        float vehicleAccY = -9.8 * tan(vehicleRoll) / cos(vehiclePitch);

        if (vehicleVelXG < 0)
            vehicleVelXG += windCoeff * vehicleVelXG * vehicleVelXG / 200.0;
        else
            vehicleVelXG -= windCoeff * vehicleVelXG * vehicleVelXG / 200.0;
        if (vehicleVelYG < 0)
            vehicleVelYG += windCoeff * vehicleVelYG * vehicleVelYG / 200.0;
        else
            vehicleVelYG -= windCoeff * vehicleVelYG * vehicleVelYG / 200.0;

        vehicleVelXG += (vehicleAccX * cos(vehicleYaw) - vehicleAccY * sin(vehicleYaw)) / 200.0;
        vehicleVelYG += (vehicleAccX * sin(vehicleYaw) + vehicleAccY * cos(vehicleYaw)) / 200.0;

        float velX1 = vehicleVelXG * cos(vehicleYaw) + vehicleVelYG * sin(vehicleYaw);
        float velY1 = -vehicleVelXG * sin(vehicleYaw) + vehicleVelYG * cos(vehicleYaw);
        float velZ1 = vehicleVelZG;

        float velX2 = velX1 * cos(vehiclePitch) - velZ1 * sin(vehiclePitch);
        float velY2 = velY1;
        float velZ2 = velX1 * sin(vehiclePitch) + velZ1 * cos(vehiclePitch);

        vehicleVelX = velX2;
        vehicleVelY = velY2 * cos(vehicleRoll) + velZ2 * sin(vehicleRoll);
        vehicleVelZ = -velY2 * sin(vehicleRoll) + velZ2 * cos(vehicleRoll);

        vehicleX += vehicleVelXG / 200.0;
        vehicleY += vehicleVelYG / 200.0;
        vehicleZ += vehicleVelZG / 200.0;
        vehicleYaw += vehicleYawRate / 200.0;

        ros::Time odomTimeRec = odomTime;
        odomTime = ros::Time::now();
        if (odomTime == odomTimeRec) odomTime += ros::Duration(0.005);

        odomSendIDPointer = (odomSendIDPointer + 1) % stackNum;
        odomTimeStack[odomSendIDPointer] = odomTime.toSec();
        vehicleXStack[odomSendIDPointer] = vehicleX;
        vehicleYStack[odomSendIDPointer] = vehicleY;
        vehicleZStack[odomSendIDPointer] = vehicleZ;
        vehicleRollStack[odomSendIDPointer] = vehicleRoll;
        vehiclePitchStack[odomSendIDPointer] = vehiclePitch;
        vehicleYawStack[odomSendIDPointer] = vehicleYaw;
        terrainRollStack[odomSendIDPointer] = terrainRoll;
        terrainPitchStack[odomSendIDPointer] = terrainPitch;

        // publish 200Hz odometry messages
        geometry_msgs::Quaternion geoQuat =
            tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

        odomData.header.stamp = odomTime;
        odomData.pose.pose.orientation = geoQuat;
        odomData.pose.pose.position.x = vehicleX;
        odomData.pose.pose.position.y = vehicleY;
        odomData.pose.pose.position.z = vehicleZ;
        odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
        odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
        odomData.twist.twist.angular.z = vehicleYawRate;
        odomData.twist.twist.linear.x = vehicleVelX;
        odomData.twist.twist.linear.y = vehicleVelY;
        odomData.twist.twist.linear.z = vehicleVelZ;
        pubVehicleOdom.publish(odomData);

        // publish 200Hz tf messages
        odomTrans.stamp_ = odomTime;
        odomTrans.setRotation(tf::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w));
        odomTrans.setOrigin(tf::Vector3(vehicleX, vehicleY, vehicleZ));
        tfBroadcaster.sendTransform(odomTrans);

        // publish 200Hz Gazebo model state messages (this is for Gazebo simulation)

        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, sensorPitch + vehiclePitch,
                                                          vehicleYaw);
        cameraState.pose.orientation = geoQuat;
        cameraState.pose.position.x = vehicleX;
        cameraState.pose.position.y = vehicleY;
        cameraState.pose.position.z = vehicleZ;
        pubModelState.publish(cameraState);

        robotState.pose.orientation = geoQuat;
        robotState.pose.position.x = vehicleX;
        robotState.pose.position.y = vehicleY;
        robotState.pose.position.z = vehicleZ;
        pubModelState.publish(robotState);

        // lidar姿态随着地形起伏而变化
        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(terrainRoll, terrainPitch, 0);
        livoxState.pose.orientation = geoQuat;
        livoxState.pose.position.x = vehicleX;
        livoxState.pose.position.y = vehicleY;
        livoxState.pose.position.z = vehicleZ + 0.03;
        pubModelState.publish(livoxState);

        // lidarState.pose.orientation = geoQuat;
        // lidarState.pose.position.x = vehicleX;
        // lidarState.pose.position.y = vehicleY;
        // lidarState.pose.position.z = vehicleZ;
        // pubModelState.publish(lidarState);

        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
