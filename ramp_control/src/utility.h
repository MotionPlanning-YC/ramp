#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <math.h>
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Range.h"


#define PI 3.14159f

class Utility {
  public:
    
    Utility();
    ~Utility() {}

    std::vector<ramp_msgs::Range> standardRanges;
    
    const float euclideanDistance(const std::vector<float> a, const std::vector<float> b) const;

    const float findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    const float findAngleFromAToB(const trajectory_msgs::JointTrajectoryPoint a, const trajectory_msgs::JointTrajectoryPoint b) const;
    
    const float findDistanceBetweenAngles(const float a1, const float a2) const;
    
    const float displaceAngle(const float a1, float a2) const;
    
    const float getEuclideanDist(const std::vector<float> a, std::vector<float> b) const;
    
    const std::string toString(const trajectory_msgs::JointTrajectoryPoint p) const;
    const std::string toString(const ramp_msgs::Trajectory traj) const;
    const std::string toString(const ramp_msgs::Path p) const;
    const std::string toString(const ramp_msgs::MotionState c) const;
    const std::string toString(const ramp_msgs::KnotPoint kp) const;
};
#endif
