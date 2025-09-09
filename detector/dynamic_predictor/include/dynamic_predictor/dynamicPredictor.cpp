/*
    FILE: dynamicPredictor.cpp
    ---------------------------------
    function implementation of dynamic osbtacle predictor
*/
#include <dynamic_predictor/dynamicPredictor.h>

namespace dynamicPredictor{
    predictor::predictor(const ros::NodeHandle& nh) : nh_(nh){
        this->ns_ = "dynamic_predictor";
        this->hint_ = "[predictor]";
        this->initParam();
        this->registerPub();
        this->registerCallback();
    }

    void predictor::initParam(){
        // prediction size
        if (not this->nh_.getParam(this->ns_ + "/prediction_size", this->numPred_)){
            this->numPred_ = 5;
            std::cout << this->hint_ << ": No prediction size parameter found. Use default: 5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The prediction size is set to: " << this->numPred_ << std::endl;
        }  

        // prediction time step
        if (not this->nh_.getParam(this->ns_ + "/prediction_time_step", this->dt_)){
            this->dt_ = 0.1;
            std::cout << this->hint_ << ": No prediction time step parameter found. Use default: 0.1." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The prediction time step is set to: " << this->dt_ << std::endl;
        }  

        // prediction confidence level
        if (not this->nh_.getParam(this->ns_ + "/prediction_z_score", this->zScore_)){
            this->zScore_ = 1.645;
            std::cout << this->hint_ << ": No prediction z score parameter found. Use default: 1.645." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The prediction z score is set to: " << this->zScore_ << std::endl;
        } 

        // minimum turning time
        if (not this->nh_.getParam(this->ns_ + "/min_turning_time", this->minTurningTime_)){
            this->minTurningTime_ = 2.0;
            std::cout << this->hint_ << ": No minimum turning time parameter found. Use default: 1.0." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": The minimum turning time is set to: " << this->minTurningTime_ << std::endl;
        }  

        // maximum turning time
        if (not this->nh_.getParam(this->ns_ + "/max_turning_time", this->maxTurningTime_)){
            this->maxTurningTime_ = 3.0;
            std::cout << this->hint_ << ": No maximum turning time parameter found. Use default: 3.0." << std::endl;
        }
        else{
            if (this->maxTurningTime_ < this->minTurningTime_){
                this->maxTurningTime_ = this->minTurningTime_+1.0;
            }
            std::cout << this->hint_ << ": The maximum turning time is set to: " << this->maxTurningTime_ << std::endl;
        }

        // max front prob param
        double maxFrontProb;
        if (not this->nh_.getParam(this->ns_ + "/max_front_prob", maxFrontProb)){
            maxFrontProb = 0.5;
            this->paramr_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            this->paraml_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            std::cout << this->hint_ << ": No max front prob param. Use default: 0.5." << std::endl;
        }
        else{
            this->paramr_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            this->paraml_ = (1-maxFrontProb) / (3*maxFrontProb-1);
            std::cout << this->hint_ << ": Max front prob param is set to: " << maxFrontProb << std::endl;
        } 

        // front angle param
        if (not this->nh_.getParam(this->ns_ + "/front_angle", this->frontAngle_)){
            this->frontAngle_ = 1/6*M_PI;
            this->paramf_ = sqrt(pow(this->frontAngle_,2)/(-2*log(this->paraml_*(1+sin(this->frontAngle_))-this->paraml_)));
            std::cout << this->hint_ << ": No front angle param. Use default: 30 degree." << std::endl;
        }
        else{
            this->frontAngle_ = this->frontAngle_*M_PI/180;
            this->paramf_ = sqrt(pow(this->frontAngle_,2)/(-2*log(this->paraml_*(1+sin(this->frontAngle_))-this->paraml_)));
            std::cout << this->hint_ << ": Front angle param is set to: " << this->frontAngle_ << std::endl;
        } 

        // stop velocity param
        if (not this->nh_.getParam(this->ns_ + "/stop_velocity_thereshold", this->stopVel_)){
            this->stopVel_ = 0.2;
            this->params_ = atanh(0.5)/this->stopVel_;
            std::cout << this->hint_ << ": No stop velocity thereshold param. Use default: 0.2." << std::endl;
        }
        else{
            this->params_ = atanh(0.5)/this->stopVel_;
            std::cout << this->hint_ << ": Stop velocity thereshold param is set to: " << this->stopVel_ << std::endl;
        } 

        std::cout << this->hint_ << ": Front param is set to: " << this->paramf_ << std::endl;
        std::cout << this->hint_ << ": Left param is set to: " << this->paraml_ << std::endl;
        std::cout << this->hint_ << ": Right param is set to: " << this->paramr_ << std::endl;
        std::cout << this->hint_ << ": Stop param is set to: " << this->params_ << std::endl;

        // prob scale param
        if (not this->nh_.getParam(this->ns_ + "/prob_scale_param", this->pscale_)){
            this->pscale_ = 1.5;
            std::cout << this->hint_ << ": No prob scale param. Use default: 1.5." << std::endl;
        }
        else{
            std::cout << this->hint_ << ": Prob scale param is set to: " << this->pscale_ << std::endl;
        } 
    }

    void predictor::setDetector(const std::shared_ptr<onboardDetector::dynamicDetector>& detector){
        this->detector_ = detector;
        this->detectorReady_ = true;
    }

    void predictor::setDetector(const std::shared_ptr<onboardDetector::fakeDetector>& detector){
        this->detectorGT_ = detector;
        this->useFakeDetector_ = true;
        this->detectorGTReady_ = true;
    }

    void predictor::setMap(const std::shared_ptr<mapManager::dynamicMap>& map){
        this->map_ = map;
        this->map_->getRobotSize(this->robotSize_);
        this->setDetector(this->map_->getDetector());
        this->mapReady_ = true;
    }
    
    void predictor::registerPub(){
        // history trajectory pub
        this->historyTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/history_trajectories", 10);
        this->predTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/predict_trajectories", 10);
        this->intentVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/intent_probability", 10);
        this->varPointsPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/var_points", 10);
        this->predBBoxPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/pred_bbox", 10);
    }

    void predictor::registerCallback(){
        // 保持预测计算的频率不变
        this->predTimer_= this->nh_.createTimer(ros::Duration(0.033), &predictor::predCB, this);
        // 降低可视化更新的频率，从0.033秒(30Hz)降低到0.2秒(5Hz)
        this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &predictor::visCB, this);
    }

    void predictor::visCB(const ros::TimerEvent&){
        this->publishHistoryTraj();
        this->publishPredTraj();
        this->publishIntentVis();
        this->publishVarPoints();
        this->publishPredBBox();
    }

    void predictor::predCB(const ros::TimerEvent&){    
        this->predict();
    }

    // main function for prediction
    void predictor::predict(){ 
        // get history
        if (this->useFakeDetector_ and this->detectorGTReady_ and this->mapReady_){
            this->detectorGT_->getDynamicObstaclesHist(this->posHist_, this->velHist_, this->accHist_, this->sizeHist_, this->robotSize_);
        }
        else if (not this->useFakeDetector_ and this->detectorReady_ and this->mapReady_){
            this->detector_->getDynamicObstaclesHist(this->posHist_, this->velHist_, this->accHist_, this->sizeHist_, this->robotSize_);
        }
        if (this->posHist_.size()){
            if (this->posHist_[0].size()){
                // intent prediction
                std::vector<Eigen::VectorXd> intentProbTemp;
                this->intentProb(intentProbTemp);

                // trajectory prediction
                std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> allPredPointsTemp;
                std::vector<std::vector<std::vector<Eigen::Vector3d>>> posPredTemp;
                std::vector<std::vector<std::vector<Eigen::Vector3d>>> sizePredTemp;
                this->predTraj(allPredPointsTemp, posPredTemp, sizePredTemp);

                this->intentProb_ = intentProbTemp;
                this->posPred_ = posPredTemp;
                this->sizePred_ = sizePredTemp;
                this->allPredPoints_ = allPredPointsTemp;
            }
        }
        else{
            this->intentProb_.clear();
            this->allPredPoints_.clear();
            this->posPred_.clear();
            this->sizePred_.clear();
        }
    }

    void predictor::intentProb(std::vector<Eigen::VectorXd> &intentProbTemp){
        int numOb = this->posHist_.size();
        
        intentProbTemp.resize(numOb);
        for (int i=0; i<numOb; ++i){
            // init state prob P
            Eigen::VectorXd P;
            P.resize(this->numIntent_);
            P.setConstant(1.0/this->numIntent_);
            int numHist = this->posHist_[i].size();
            for (int j=2; j<numHist; ++j){
                // transition matrix 
                Eigen::Vector3d prevPos, prevVel;
                Eigen::Vector3d currPos, currVel;
                prevPos = this->posHist_[i][numHist-j-1];
                prevVel = this->velHist_[i][numHist-j-1];
                currPos = this->posHist_[i][numHist-j-2];
                currVel = this->velHist_[i][numHist-j-2];
                double prevAngle = atan2(prevPos(1)-this->posHist_[i][numHist-j](1), prevPos(0)-this->posHist_[i][numHist-j](0));
                double currAngle = atan2(currPos(1)-prevPos(1), currPos(0)-prevPos(0));
                // currVel = this->posHist_[i][numHist-j-2];
                // Eigen::MatrixXd transMat = this->genTransitionMatrix(prevPos, currPos, prevVel, currVel);
                Eigen::MatrixXd transMat = this->genTransitionMatrix(prevAngle, currAngle, currVel);
                Eigen::VectorXd newP= transMat*P;
                P = newP;
            }
            intentProbTemp[i] = P;
            
        }
    }

    // Eigen::MatrixXd predictor::genTransitionMatrix(const Eigen::Vector3d &prevPos, const Eigen::Vector3d &currPos, const Eigen::Vector3d &prevVel, const Eigen::Vector3d &currVel){
    Eigen::MatrixXd predictor::genTransitionMatrix(const double &prevAngle, const double &currAngle, const Eigen::Vector3d &currVel){
        // Initialize transMat
        Eigen::MatrixXd transMat;
        Eigen::VectorXd probVec;        
        probVec.resize(this->numIntent_);
        transMat.resize(this->numIntent_, this->numIntent_);


        // // double theta = atan2(currPos(1)-prevPos(1), currPos(0)-prevPos(0)) -  atan2(currVel(1), currVel(0));
        double theta =  currAngle - prevAngle;
        
        if (theta > M_PI){
            theta = theta - 2*M_PI;
        }
        else if (theta <= -M_PI){
            theta = theta + 2*M_PI;
        }
        
        double r = sqrt(pow(currVel(0), 2) + pow(currVel(1), 2));        
        
        // fill in every column of transition matrix
        for (int i=0; i<this->numIntent_;i++){
            Eigen::VectorXd scale;
            scale.setOnes(this->numIntent_);
            scale(i) = this->pscale_;
            Eigen::VectorXd probVec = this->genTransitionVector(theta, r, scale);
            transMat.block(0,i,this->numIntent_,1) = probVec;
        }        

        return transMat;
    }

    Eigen::VectorXd predictor::genTransitionVector(const double &theta, const double &r, const Eigen::VectorXd &scale){
        Eigen::VectorXd probVec;        
        probVec.resize(this->numIntent_);
        double ps, pf, pr, pl;

        pf = scale(0)*(exp(-0.5*pow(theta/this->paramf_,2))+this->paraml_); //guassian distribution
        pl = scale(1)*(this->paraml_*(1+sin(theta)));
        pr = scale(2)*(this->paramr_*(1-sin(theta)));
        ps = ((1-tanh(this->params_/scale(3)*r)));
        double sum = pr+pl+pf;
        pr = (1-ps)*pr/sum;
        pl = (1-ps)*pl/sum;
        pf = (1-ps)*pf/sum;

        probVec(FORWARD) = pf;
        probVec(LEFT) = pl;
        probVec(RIGHT) = pr;
        probVec(STOP) = ps;

        return probVec;
    }

    void predictor::predTraj(std::vector<std::vector<std::vector<std::vector<Eigen::Vector3d>>>> &allPredPointsTemp,
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> &posPredTemp,
        std::vector<std::vector<std::vector<Eigen::Vector3d>>> &sizePredTemp){

        posPredTemp.clear();
        posPredTemp.resize(this->posHist_.size());
        sizePredTemp.clear();
        sizePredTemp.resize(this->sizeHist_.size());
        allPredPointsTemp.clear();
        allPredPointsTemp.resize(this->posHist_.size());
        
        // predict each obstacle
        for (int i=0; i<int(this->posHist_.size()); i++){
            posPredTemp[i].resize(this->numIntent_);
            sizePredTemp[i].resize(this->numIntent_);
            allPredPointsTemp[i].resize(this->numIntent_);

            // predict for each number of intent
            for (int j=FORWARD; j<=STOP; ++j){
                std::vector<std::vector<Eigen::Vector3d>> predPoints;
                std::vector<Eigen::Vector3d> predSize;
                this->genPoints(j, this->posHist_[i][0], this->velHist_[i][0], this->accHist_[i][0], this->sizeHist_[i][0], predPoints, predSize);
                if (predPoints.size()){
                    std::vector<Eigen::Vector3d> predPos;
                    this->genTraj(predPoints, predPos, predSize);
                    posPredTemp[i][j] = predPos;
                    sizePredTemp[i][j] = predSize;
                    allPredPointsTemp[i][j] = predPoints;
                }
                else{
                    Eigen::Vector3d size = this->sizeHist_[i][0];
                    Eigen::Vector3d currVel = this->velHist_[i][0];
                    Eigen::Vector3d currPos = this->posHist_[i][0];
                    std::vector<Eigen::Vector3d> predPos;
                    double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2)); 
                    for (int i=0;i<this->numPred_+1;i++){
                        predPos.push_back(currPos);
                        predSize.push_back(size);
                        size(0) += 2*min(vel,this->stopVel_)*this->dt_;
                        size(1) += 2*min(vel,this->stopVel_)*this->dt_;
                    }
                    posPredTemp[i][j] = predPos;
                    sizePredTemp[i][j] = predSize;
                }
            }
        }
	}

    void predictor::genPoints(const int &intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        double vel =  sqrt(pow(currVel(0),2)+pow(currVel(1),2));
        if (vel <= this->stopVel_){
           this->modelStop(currPos, currVel, currSize, predPoints, predSize);
        }
        else{
            if (intentType==FORWARD){
                this->modelForward(currPos, currVel, currAcc, currSize, predPoints, predSize);
            }
            else if (intentType==LEFT or intentType==RIGHT){
                this->modelTurning(intentType, currPos, currVel, currAcc, currSize, predPoints, predSize);
            }
            else if (intentType==STOP){
                this->modelStop(currPos, currVel,currSize, predPoints, predSize);
            }
        }
    }

    void predictor::modelForward(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2));
        double angleInit = atan2(currVel(1), currVel(0)); // facing direction
        double minVel, maxVel;
        double minAngle, maxAngle;
        minVel = vel-vel;
        maxVel = vel+vel;
        minAngle = angleInit - this->frontAngle_;
        maxAngle = angleInit + this->frontAngle_;
        bool isValid = true;

        // const velocity
        for (double i=minAngle; i<maxAngle; i+=0.1){
            for (double j=minVel; j<maxVel; j+=0.1){
                std::vector<Eigen::Vector3d> predPointTemp;
                Eigen::VectorXd currState(4);
                currState<<currPos(0), currPos(1), j*cos(i), j*sin(i);
                predPointTemp.clear();
                predPointTemp.push_back(currPos);
                for (int k=0; k<this->numPred_;k++){
                    // TODO: check const acc model
                    Eigen::MatrixXd model;
                    model = MatrixXd::Identity(4,4);
                    model.block(0,2,2,2) = Eigen::MatrixXd::Identity(2,2)*this->dt_;
                    Eigen::VectorXd nextState = model*currState;
                    Eigen::Vector3d p;
                    p << nextState(0), nextState(1), currPos(2);
                    if (this->map_->isInflatedOccupied(p)){
                        isValid = false;
                        break;
                    }
                    else{
                        predPointTemp.push_back(p);
                    }
                    currState = nextState;
                }
                if (isValid){
                    predPoints.push_back(predPointTemp);
                }
                else{
                    isValid = true;
                    break;
                }
            }
        }

        for (int i=0;i<this->numPred_+1;i++){
            predSize.push_back(currSize);
        }
    }

    void predictor::modelTurning(const int & intentType, const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currAcc, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        // double acc = sqrt(pow(currAcc(0),2)+pow(currAcc(1),2));
        double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2));
        double angleInit = atan2(currVel(1), currVel(0));
        double minVel, maxVel;
        double angle;
        minVel = vel-vel;
        maxVel = vel+vel;
        // double minAcc, maxAcc;
        // minAcc = acc-acc;
        // maxAcc = acc+acc;
        double endMin, endMax;

        if (intentType != LEFT and intentType != RIGHT){
            cout << this->hint_ << ": Please enter the correct intent!!!" << endl;
        }
        
        double minAngVel, maxAngVel;
        if(intentType == LEFT){
            endMin = this->frontAngle_+angleInit;
            endMax = (M_PI-this->frontAngle_)+angleInit;
            minAngVel = (M_PI/2)/this->maxTurningTime_;
            maxAngVel = (M_PI/2)/this->minTurningTime_;
        }
        else{
            endMin = -(M_PI-this->frontAngle_)+angleInit;
            endMax = -this->frontAngle_+angleInit;
            minAngVel = (-M_PI/2)/this->minTurningTime_;
            maxAngVel = (-M_PI/2)/this->maxTurningTime_;
        }
        bool isValid = true;

        for (double i = minVel; i<maxVel;i+=0.2){
            for (double j = minAngVel;j<maxAngVel;j+=0.2){
                for (double endAngle = endMin;endAngle<endMax;endAngle+=0.2){
                    std::vector<Eigen::Vector3d> predPointTemp;
                    Eigen::VectorXd currState(4);
                    angle = angleInit;
                    currState<<currPos(0), currPos(1), i*cos(angle), i*sin(angle);
                    predPointTemp.clear();
                    predPointTemp.push_back(currPos);
                    for (int k=0; k<this->numPred_;k++){
                        Eigen::MatrixXd model;
                        model = MatrixXd::Identity(4,4);
                        model.block(0,2,2,2) = Eigen::MatrixXd::Identity(2,2)*this->dt_;
                        Eigen::VectorXd nextState = model*currState;
                        Eigen::Vector3d p;
                        p << nextState(0), nextState(1), currPos(2);
                        if (this->map_->isInflatedOccupied(p)){
                            isValid = false;
                            break;
                        }
                        else{
                            predPointTemp.push_back(p);
                        }
                        currState = nextState;
                        angle += j*this->dt_;
                        if(intentType == LEFT){
                            angle  = min(angle, endAngle);
                        }
                        else if (intentType == RIGHT){
                            angle  = max(angle, endAngle);
                        }
                        double v = sqrt(pow(currState(2),2)+pow(currState(3),2));
                        currState(2) = v*cos(angle);
                        currState(3) = v*sin(angle);
                    }
                    if (isValid){
                        predPoints.push_back(predPointTemp);
                    }
                    else{
                        isValid = true;
                    }
                }
                    
                }
            }
        for (int i=0;i<this->numPred_+1;i++){
            predSize.push_back(currSize);
        }
    }
    
    void predictor::modelStop(const Eigen::Vector3d &currPos, const Eigen::Vector3d &currVel, const Eigen::Vector3d &currSize, std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predSize){
        predPoints.clear();
        predSize.clear();
        std::vector<Eigen::Vector3d> predPointTemp;
        Eigen::Vector3d size = currSize;
        double vel = sqrt(pow(currVel(0),2)+pow(currVel(1),2)); 
        for (int i=0;i<this->numPred_+1;i++){
            predPointTemp.push_back(currPos);
            predSize.push_back(size);
            size(0) += 2*min(vel,this->stopVel_)*this->dt_;
            size(1) += 2*min(vel,this->stopVel_)*this->dt_;
        }
        predPoints.push_back(predPointTemp);
    }

    void predictor::genTraj(const std::vector<std::vector<Eigen::Vector3d>> &predPoints, std::vector<Eigen::Vector3d> &predPos, std::vector<Eigen::Vector3d> &predSize){
        predPos.clear();
        for (int i=0;i<this->numPred_+1;i++){
            double meanx, meany;
            double variancex, variancey;
            double sumx = 0 , sumy = 0;
            double sumVarx = 0, sumVary = 0;
            int counter = 0;
            for (int j=0; j<int(predPoints.size());j++){
                if (i < int(predPoints[j].size())){
                    sumx += predPoints[j][i](0);
                    sumy += predPoints[j][i](1);
                    counter += 1;
                }
            }
            if (counter){
                meanx = sumx/counter;
                meany = sumy/counter;
                for (int j=0; j<int(predPoints.size()); j++){
                    sumVarx += pow(predPoints[j][i](0)-meanx,2);
                    sumVary += pow(predPoints[j][i](1)-meany,2);
                }
                variancex = sumVarx/counter;
                variancey = sumVary/counter;
                Eigen::Vector3d p;
                p<<meanx, meany, predPoints[0][0](2);
                predPos.push_back(p);
                predSize[i](0) += 2*sqrt(variancex)*this->zScore_; // confidence level under gaussian
                predSize[i](1) += 2*sqrt(variancey)*this->zScore_;
            }
            else{
                break;
            }
        }
        this->positionCorrection(predPos, predPoints); // if mean trajectory has collision, find the closest. Otherwise, use the mean.
    }

    void predictor::positionCorrection(std::vector<Eigen::Vector3d> &mean, const std::vector<std::vector<Eigen::Vector3d>> &predPoints){
        bool isCollide = false;
        for (int i=0; i<int(mean.size()); i++){
            if (this->map_->isInflatedOccupied(mean[i])){
                isCollide = true;
                break;
            }
        }
        double sum = 0;
        double minSum = INFINITY;
        int minIdx = -1;
        if (isCollide){
            for (int i=0; i<int(predPoints.size()); i++){
                sum = 0;
                for (int j=0; j<int(mean.size()); j++){
                    sum += sqrt(pow(predPoints[i][j](0)-mean[j](0),2)+pow(predPoints[i][j](1)-mean[j](1),2));
                    if (sum > minSum){
                        break;
                    }
                }
                if (sum < minSum){
                    minSum = sum;
                    minIdx = i;
                }
            }
            mean = predPoints[minIdx];
        }
    }

    void predictor::publishVarPoints(){
        visualization_msgs::MarkerArray varPointsMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->allPredPoints_.size(); ++i){
            visualization_msgs::Marker traj;
            traj.header.frame_id = "map";
            traj.header.stamp = ros::Time::now();
            traj.ns = "var_points";
            traj.id = countMarker;
            traj.type = visualization_msgs::Marker::POINTS;
            traj.action = visualization_msgs::Marker::ADD;
            traj.scale.x = 0.05;
            traj.scale.y = 0.05;
            traj.scale.z = 0.05;
            traj.color.a = 0.5;
            traj.lifetime = ros::Duration(0.1);
            for (size_t j=0; j<this->allPredPoints_[i].size(); ++j){
                for (size_t k=0; k<this->allPredPoints_[i][j].size(); k++){
                    for (size_t l=0; l<this->allPredPoints_[i][j][k].size();l++){
                        geometry_msgs::Point p;
                        Eigen::Vector3d pos = this->allPredPoints_[i][j][k][l];
                        p.x = pos(0); p.y = pos(1); p.z = pos(2);
                        double meanx, meany;
                        double stdx, stdy;
                        meanx = this->posPred_[i][j][l](0);
                        meany = this->posPred_[i][j][l](1);
                        stdx = this->sizePred_[i][j][l](0)-this->sizeHist_[i][0](0);
                        stdy = this->sizePred_[i][j][l](1)-this->sizeHist_[i][0](1);
                        if (p.x <= meanx+stdx/2 and p.x >= meanx-stdx/2){
                            if (p.y <= meany+stdy/2 and p.y >= meany-stdy/2){
                                traj.points.push_back(p);
                            }
                        }
                    }
                    
                }
            }

            ++countMarker;
            varPointsMsg.markers.push_back(traj);
        }
        this->varPointsPub_.publish(varPointsMsg);
    }

    void predictor::publishHistoryTraj(){
        visualization_msgs::MarkerArray historyTrajMsg;
        int countMarker = 0;
        for (size_t i=0; i<this->posHist_.size(); ++i){
            visualization_msgs::Marker traj;
            traj.header.frame_id = "map";
            traj.header.stamp = ros::Time::now();
            traj.ns = "history_traj";
            traj.id = countMarker;
            traj.type = visualization_msgs::Marker::LINE_STRIP;
            traj.action = visualization_msgs::Marker::ADD;
            traj.scale.x = 0.1;
            traj.color.a = 1.0;
            traj.color.r = 0.0;
            traj.color.g = 0.0;
            traj.color.b = 1.0;
            traj.lifetime = ros::Duration(0.1);
            for (size_t j=0; j<this->posHist_[i].size(); ++j){
                geometry_msgs::Point p;
                Eigen::Vector3d pos = this->posHist_[i][j];
                p.x = pos(0); p.y = pos(1); p.z = pos(2);
                traj.points.push_back(p);
            }

            ++countMarker;
            historyTrajMsg.markers.push_back(traj);
        }
        this->historyTrajPub_.publish(historyTrajMsg);
    }

    void predictor::publishPredTraj(){
        visualization_msgs::MarkerArray predTrajMsg;
        for (int i=0; i<int(this->posPred_.size()); ++i){
            for (int j=0; j<int(this->posPred_[i].size()); ++j){
                visualization_msgs::Marker traj;
                traj.header.frame_id = "map";
                traj.header.stamp = ros::Time::now();
                traj.ns = "pred_traj";
                traj.id = i*10+j;
                traj.type = visualization_msgs::Marker::LINE_STRIP;
                traj.action = visualization_msgs::Marker::ADD;
                traj.scale.x = 0.1;
                traj.color.a = 1.0;
                
                // 设置不同意图的预测轨迹颜色
                if (j == FORWARD) {
                    // 前进意图 - 亮绿色
                    traj.color.r = 0.0;
                    traj.color.g = 1.0;
                    traj.color.b = 0.0;
                } else if (j == LEFT) {
                    // 左转意图 - 亮蓝色
                    traj.color.r = 0.0;
                    traj.color.g = 0.5;
                    traj.color.b = 1.0;
                } else if (j == RIGHT) {
                    // 右转意图 - 亮红色
                    traj.color.r = 1.0;
                    traj.color.g = 0.0;
                    traj.color.b = 0.0;
                } else if (j == STOP) {
                    // 停止意图 - 亮黄色
                    traj.color.r = 1.0;
                    traj.color.g = 1.0;
                    traj.color.b = 0.0;
                }
                
                traj.lifetime = ros::Duration(0.1);
                
                // // 修改这里：只显示前15个预测点
                // int max_points = std::min(15, int(this->posPred_[i][j].size()));
                for (int k=0; k<int(this->posPred_[i][j].size()); ++k){

                    geometry_msgs::Point p;
                    Eigen::Vector3d pos = this->posPred_[i][j][k];
                    p.x = pos(0); p.y = pos(1); p.z = pos(2);
                    traj.points.push_back(p);
                }
                predTrajMsg.markers.push_back(traj);
            }
        }
        this->predTrajPub_.publish(predTrajMsg);
    }

    void predictor::publishIntentVis(){ 
        visualization_msgs::MarkerArray intentVisMsg;
        int countMarker = 0;
        for (int i=0; i<int(this->posHist_.size()); ++i){
            visualization_msgs::Marker intentMarker;
            intentMarker.header.frame_id = "map";
            intentMarker.header.stamp = ros::Time::now();
            intentMarker.ns = "predictor";
            intentMarker.id =  countMarker;
            intentMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            intentMarker.pose.position.x = this->posHist_[i][0](0);
            intentMarker.pose.position.y = this->posHist_[i][0](1);
            intentMarker.pose.position.z = this->posHist_[i][0](2) + this->sizeHist_[i][0](2)/2. + 0.3;
            intentMarker.scale.x = 0.35;
            intentMarker.scale.y = 0.35;
            intentMarker.scale.z = 0.35;
            intentMarker.color.a = 1.0;
            intentMarker.color.r = 1.0;
            intentMarker.color.g = 0.0;
            intentMarker.color.b = 0.0;
            intentMarker.lifetime = ros::Duration(0.1);
            // std::string intentText = "Front: " + std::to_string(this->intentProb_[i](0)) 
            //                         + "Left: " + std::to_string(this->intentProb_[i](1)) 
            //                         + "Right: " + std::to_string(this->intentProb_[i](2))
            //                         + "Stop: " + std::to_string(this->intentProb_[i](3));
            // intentMarker.text = intentText;
            std::vector<std::string> intentText(4);
            intentText[FORWARD] = "Front: "+ std::to_string(this->intentProb_[i](FORWARD));
            intentText[LEFT] = "Left: " + std::to_string(this->intentProb_[i](LEFT)) ;
            intentText[RIGHT] = "Right: " + std::to_string(this->intentProb_[i](RIGHT));
            intentText[STOP] = "Stop: " + std::to_string(this->intentProb_[i](STOP));
            int maxIdx = 0;
            double max = 0;
            for (int j=0; j<this->numIntent_;j++){
                if (this->intentProb_[i](j)>max){
                    maxIdx = j;
                    max = this->intentProb_[i](j);
                }
            }
            intentMarker.text = intentText[maxIdx];
            intentVisMsg.markers.push_back(intentMarker);
            ++countMarker;
        }
        this->intentVisPub_.publish(intentVisMsg);
    }

    void predictor::publishPredBBox(){
        if (this->posPred_.size() == 0) return;
        // publish top N intent future bounding box with the inflate size
        visualization_msgs::MarkerArray predBBoxMsg;
        visualization_msgs::Marker line;
        line.header.frame_id = "map";
        line.type = visualization_msgs::Marker::LINE_LIST;
        line.action = visualization_msgs::Marker::ADD;
        line.ns = "box3D";  
        line.scale.x = 0.06;
        line.color.r = 0;
        line.color.g = 1;
        line.color.b = 0;
        line.color.a = 1.0;
        line.lifetime = ros::Duration(0.1);

        visualization_msgs::Marker range;
        range.header.frame_id = "map";
        range.header.stamp = ros::Time::now();
        range.ns = "pred_ob_size";
        range.id = 0;
        range.type = visualization_msgs::Marker::SPHERE;
        range.action = visualization_msgs::Marker::ADD;
        // 修改阴影颜色为红色，并提高不透明度
        range.color.a = 0.6;  // 提高不透明度从0.4到0.6
        range.color.r = 1.0;  // 红色
        range.color.g = 0.0;
        range.color.b = 0.0;
        range.lifetime = ros::Duration(0.2);
        for (int i=0; i<int(this->intentProb_.size()); ++i){
            std::vector<std::pair<double, int>> intentProb;
            for (int j=0; j<this->numIntent_; ++j){
                intentProb.push_back({this->intentProb_[i](j), j});
            }
            std::sort(intentProb.begin(), intentProb.end(), 
            [](const std::pair<double, int> &left, const std::pair<double, int> &right) 
            {return left.first > right.first;});

            for (int n=0; n<1; ++n){ // top N intent
                int intentIdx = intentProb[n].second;
                std::vector<Eigen::Vector3d> predTraj = this->posPred_[i][intentIdx];
                std::vector<Eigen::Vector3d> predSize = this->sizePred_[i][intentIdx];
                
                // 修改这里: 只显示前面15个点中的几个点
                // 原来是: for (int t=10; t<int(predTraj.size()); t+=10){
                // 现在限制最大显示到15，并且从第5个点开始显示
                for (int t=5; t<std::min(10, int(predTraj.size())); t+=10){
                    Eigen::Vector3d obPos = predTraj[t];
                    Eigen::Vector3d predObSize = predSize[t];
                    Eigen::Vector3d obSize = this->sizeHist_[i][0];

                    double x = obPos(0); 
                    double y = obPos(1); 
                    double z = (obPos(2)+obSize(2)/2)/2; 

                    double x_width = obSize(0);
                    double y_width = obSize(1);
                    double z_width = 2*obPos(2);

                    
                    vector<geometry_msgs::Point> verts;
                    geometry_msgs::Point p;
                    // vertice 0
                    p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 1
                    p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 2
                    p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 3
                    p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
                    verts.push_back(p);

                    // vertice 4
                    p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);

                    // vertice 5
                    p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);

                    // vertice 6
                    p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);

                    // vertice 7
                    p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
                    verts.push_back(p);
                    
                    int vert_idx[12][2] = {
                        {0,1},
                        {1,2},
                        {2,3},
                        {0,3},
                        {0,4},
                        {1,5},
                        {3,7},
                        {2,6},
                        {4,5},
                        {5,6},
                        {4,7},
                        {6,7}
                    };
                    
                    for (size_t i=0;i<12;i++){
                        line.points.push_back(verts[vert_idx[i][0]]);
                        line.points.push_back(verts[vert_idx[i][1]]);
                    }
                    
                    predBBoxMsg.markers.push_back(line);
                    
                    line.id++;


                    range.pose.position.x = obPos(0);
                    range.pose.position.y = obPos(1);
                    range.pose.position.z = obPos(2);
                    range.scale.x = predObSize(0);
                    range.scale.y = predObSize(1);
                    range.scale.z = 0.1;
                    range.id++;
                    predBBoxMsg.markers.push_back(range);
                }
            }
        }
        this->predBBoxPub_.publish(predBBoxMsg);
    }

    void predictor::getPrediction(std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predPos, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predSize, std::vector<Eigen::VectorXd> &intentProb){
        if (this->sizePred_.size()){
            predPos = this->posPred_;
            predSize = this->sizePred_;
            intentProb = this->intentProb_;
        }
        else{
            predPos.clear();
            predSize.clear();
            intentProb.clear();
        }
    }

    void predictor::getPrediction(std::vector<dynamicPredictor::obstacle> &predOb){
        for (int i=0;i<int(this->posPred_.size());i++){
            dynamicPredictor::obstacle ob;
            ob.posPred = this->posPred_[i];
            ob.sizePred = this->sizePred_[i];
            ob.intentProb = this->intentProb_[i];
            predOb.push_back(ob);
        }
    }
}