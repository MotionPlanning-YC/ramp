#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Trajectory.h"
#include <tf/transform_datatypes.h>
#include "range.h"
#define PI 3.14159f


class Utility {
  public:
    Utility();

    std::vector<Range> standardRanges;
    
    const float positionDistance(const std::vector<float> a, const std::vector<float> b) const;

    const float findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    
    const float findDistanceBetweenAngles(const float a1, const float a2) const;
    
    const float displaceAngle(const float a1, float a2) const;
    
    const float getEuclideanDist(const std::vector<float> a, std::vector<float> b) const;
    
    const std::string toString(const ramp_msgs::KnotPoint kp) const;
    const std::string toString(const ramp_msgs::Configuration c) const;
    const std::string toString(const ramp_msgs::Path path) const;
    const std::string toString(const ramp_msgs::Trajectory traj) const;
};

#endif 
