/*
    FILE: dynamicPredictor.h
    ---------------------------------
    header file of dynamic obstacle predictor
*/

#ifndef DYNAMIC_PREDICTOR_H
#define DYNAMIC_PREDICTOR_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <onboard_detector/dynamicDetector.h>
#include <onboard_detector/fakeDetector.h>
#include <map_manager/dynamicMap.h>
#include <dynamic_predictor/utils.h>

namespace dynamicPredictor{
    class predictor{
    private:
        std::string ns_;
        std::string hint_;

        // ROS
        ros::NodeHandle nh_;
        ros::Timer predTimer_;
        ros::Timer visTimer_;
        ros::Publisher historyTrajPub_;
        ros::Publisher predTrajPub_;
        ros::Publisher intentVisPub_;
        ros::Publisher varPointsPub_;
        ros::Publisher predBBoxPub_;

        // Param
        Eigen::Vector3d robotSize_ = Eigen::Vector3d(0.0,0.0,0.0);
        int numPred_;
        double dt_;
        double minTurningTime_, maxTurningTime_;
        double zScore_;
        // bool useFakeDetector_ = false;
        int numIntent_ = 4;
        double frontAngle_;
        double stopVel_;
        double paramf_,paraml_,paramr_,params_; // Probablity
        double pscale_;
        bool useFakeDetector_ = false;

        bool mapReady_ = false;
        bool detectorReady_ = false;
        bool detectorGTReady_ = false;
        
        std::shared_ptr<onboardDetector::dynamicDetector> detector_; 
        std::shared_ptr<onboardDetector::fakeDetector> detectorGT_;
        std::shared_ptr<mapManager::dynamicMap> map_;       
        
        std::vector<std::vector<Eigen::Vector3d>> posHist_;
        std::vector<std::vector<Eigen::Vector3d>> velHist_;
        std::vector<std::vector<Eigen::Vector3d>> sizeHist_;
        std::vector<std::vector<Eigen::Vector3d>> accHist_;
        std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> allPredPoints_;
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> posPred_;
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> sizePred_;
        std::vector<Eigen::VectorXd> intentProb_;

    public:
        predictor(const ros::NodeHandle& nh);

        void initParam();
        void registerPub();
        void registerCallback();  

        void setDetector(const std::shared_ptr<onboardDetector::dynamicDetector>& detector);
        void setDetector(const std::shared_ptr<onboardDetector::fakeDetector>& detector);
        void setMap(const std::shared_ptr<mapManager::dynamicMap>& map);

        // main function for prediction
        void predict();
        void intentProb(std::vector<Eigen::VectorXd> &intentProbTemp);
        Eigen::MatrixXd genTransitionMatrix(const double &prevAngle, const double &currAngle, const Eigen::Vector3d &currVel);
        Eigen::VectorXd genTransitionVector(const double &theta, const double &r, const Eigen::VectorXd &scale);
        void predTraj(std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> &allPredPointsTemp, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &posPredTemp, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &sizePredTemp);
        void genPoints(const int &intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void genTraj(const std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predPos, std::vector<Eigen::Vector3d> &predSize);
        void modelForward(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void modelTurning(const int &intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void modelStop(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize);
        void positionCorrection(std::vector<Eigen::Vector3d> &mean, const std::vector<std::vector<Eigen::Vector3d>> &predPoints);
        // callback 
        void visCB(const ros::TimerEvent&);  
        void predCB(const ros::TimerEvent&); 

        // visualization
        void publishVarPoints();
        void publishHistoryTraj();
        void publishPredTraj();
        void publishIntentVis();
        void publishPredBBox();
        

        // user function
        void getPrediction(std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predPos, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predSize, std::vector<Eigen::VectorXd> &intentProb);
        void getPrediction(std::vector<dynamicPredictor::obstacle> &predOb);

    };
}

#endif