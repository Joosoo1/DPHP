/*
	FILE: dynamic_predictor_node.cpp
	--------------------------
	Run predictor tesr node
*/
#include <ros/ros.h>
#include <dynamic_predictor/dynamicPredictor.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_predictor_node");
	ros::NodeHandle nh;

	std::shared_ptr<onboardDetector::dynamicDetector> detector;
	std::shared_ptr<mapManager::dynamicMap> map;
	map.reset(new mapManager::dynamicMap (nh, false));
	dynamicPredictor::predictor p (nh);
	p.setMap(map);

	ros::spin();

	return 0;
}