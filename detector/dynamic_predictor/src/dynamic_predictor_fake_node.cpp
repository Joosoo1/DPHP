/*
	FILE: dynamic_predictor_node.cpp
	--------------------------
	Run predictor tesr node
*/
#include <ros/ros.h>
#include <dynamic_predictor/dynamicPredictor.h>

std::shared_ptr<onboardDetector::fakeDetector> detectorGT_;
std::shared_ptr<mapManager::dynamicMap> map_;

void freeMapCB(const ros::TimerEvent&){
	std::vector<onboardDetector::box3D> obstacles;
	std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> freeRegions;
	detectorGT_->getObstacles(obstacles);
	double fov = 1.57;
	for (onboardDetector::box3D ob: obstacles){
		if (detectorGT_->isObstacleInSensorRange(ob, fov)){
			Eigen::Vector3d lowerBound (ob.x-ob.x_width/2-0.3, ob.y-ob.y_width/2-0.3, ob.z-ob.z_width/2-0.3);
			Eigen::Vector3d upperBound (ob.x+ob.x_width/2+0.3, ob.y+ob.y_width/2+0.3, ob.z+ob.z_width/2+0.3);
			freeRegions.push_back(std::make_pair(lowerBound, upperBound));
		}
	}
	map_->updateFreeRegions(freeRegions);
	map_->freeRegions(freeRegions);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "dyanmic_predictor_node");
	ros::NodeHandle nh;
	ros::Timer freeMapTimer = nh.createTimer(ros::Duration(0.01), &freeMapCB);
	
	map_.reset(new mapManager::dynamicMap (nh));
	detectorGT_.reset(new onboardDetector::fakeDetector (nh));
	dynamicPredictor::predictor p (nh);
	p.setMap(map_);
	p.setDetector(detectorGT_);
	

	ros::spin();

	return 0;
}