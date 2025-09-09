#ifndef DYNAMIC_PREDICTOR_UTILS_H
#define DYNAMIC_PREDICTOR_UTILS_H

#include<Eigen/Eigen>
namespace dynamicPredictor{
    struct obstacle
    {
        /* data */
        // std::vector<Eigen::Vector3d> posHist;
        std::vector<std::vector<Eigen::Vector3d>> posPred;
        std::vector<std::vector<Eigen::Vector3d>> sizePred;
        Eigen::VectorXd intentProb; // front, left, right, stop
    };

    enum intentType{
        FORWARD,
        LEFT,
        RIGHT,
        STOP
    };
    
    
}
#endif