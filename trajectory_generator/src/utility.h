#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include "geometry_msgs/Pose2D.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Path.h"

#define PI 3.14159f

class Utility {
  public:
    const float euclideanDistance(const std::vector<float> a, const std::vector<float> b) const;

    const float findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    
    const float findDistanceBetweenAngles(const float a1, const float a2) const;
    
    const float displaceAngle(const float a1, float a2) const;
    
    const float getEuclideanDist(const std::vector<float> a, std::vector<float> b) const;
    
    const std::string toString(const ramp_msgs::TrajectoryRequest::Request tr) const;
    const std::string toString(const ramp_msgs::Path p) const;
    const std::string toString(const geometry_msgs::Pose2D p) const;
    const std::string toString(const ramp_msgs::Trajectory traj) const;

};
#endif
