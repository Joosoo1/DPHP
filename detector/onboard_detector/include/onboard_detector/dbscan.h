/*
    FILE: dbscan.h
    ------------------
    helper class header for dbscan
*/
#ifndef DBSCAN_H
#define DBSCAN_H

#include <cmath>
#include <vector>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

using namespace std;
namespace onboardDetector {
    typedef struct Point_ {
        float x, y, z;  // X, Y, Z position
        int clusterID;  // clustered ID
    } Point;

    class DBSCAN {
    public:
        DBSCAN(const unsigned int minPts, const float eps, const vector<Point>& points) {
            m_minPoints = minPts;
            m_epsilon = eps;
            // m_points = points;
            m_pointSize = points.size();
        }
        ~DBSCAN() {}

        int run();
        vector<int> calculateCluster(Point point);
        int expandCluster(Point point, int clusterID);
        inline double calculateDistance(const Point& pointCore, const Point& pointTarget);
        int getTotalPointSize() const {
            return m_pointSize;
        }
        int getMinimumClusterSize() const {
            return m_minPoints;
        }
        int getEpsilonSize() const {
            return m_epsilon;
        }

        vector<Point> m_points;

    private:
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;
    };
}  // namespace onboardDetector
#endif  // DBSCAN_H
