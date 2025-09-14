/*
	FILE: detector_node.cpp
	--------------------------
	Run detector node
*/
#include <onboard_detector/dynamicDetector.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "dynamic_detector_node");
    ros::NodeHandle nh;

    onboardDetector::dynamicDetector d(nh);

    ros::spin();

    return 0;
}