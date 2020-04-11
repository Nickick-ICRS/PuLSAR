#ifndef __DBSCAN_HPP__
#define __DBSCAN_HPP__

// Code slightly modified from james-yoo's implementation: 
// https://github.com/james-yoo/DBSCAN

#include <vector>
#include <geometry_msgs/Pose.h>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

namespace dbscan {

struct Point {
    double x, y, z;  // X, Y, Z position
    int clusterID;  // clustered ID
    // Which pose it comes from
    geometry_msgs::Pose* pose;
};

class DBSCAN {
public:    
    DBSCAN(unsigned int minPts, float eps, std::vector<Point> points){
        m_minPoints = minPts;
        m_epsilon = eps;
        m_points = points;
        m_pointSize = points.size();
    }
    ~DBSCAN(){}

    int run();
    std::vector<int> calculateCluster(Point point);
    int expandCluster(Point point, int clusterID);
    inline double calculateDistance(Point pointCore, Point pointTarget);

    int getTotalPointSize() {return m_pointSize;}
    int getMinimumClusterSize() {return m_minPoints;}
    int getEpsilonSize() {return m_epsilon;}

    std::vector<Point> m_points;
private:
    unsigned int m_pointSize;
    unsigned int m_minPoints;
    float m_epsilon;
};

}

#endif // __DBSCAN_HPP__
