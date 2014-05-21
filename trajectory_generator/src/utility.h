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
#include "ramp_msgs/Configuration.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include <tf/transform_datatypes.h>
#define PI 3.14159f


class Utility {
  public:
    Utility();
    
    const double positionDistance(const std::vector<double> a, const std::vector<double> b) const;

    const double findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const;
    
    const double findDistanceBetweenAngles(const double a1, const double a2) const;
    
    const double displaceAngle(const double a1, double a2) const;
    
    const double getEuclideanDist(const std::vector<double> a, std::vector<double> b) const;
    const double getEuclideanDist(const ramp_msgs::KnotPoint a, const ramp_msgs::KnotPoint b) const;
    
    const std::string toString(const ramp_msgs::KnotPoint kp) const;
    const std::string toString(const ramp_msgs::MotionState mp) const;
    const std::string toString(const ramp_msgs::Configuration c) const;
    const std::string toString(const ramp_msgs::Path path) const;
    const std::string toString(const ramp_msgs::Trajectory traj) const;
    const std::string toString(const ramp_msgs::TrajectoryRequest::Request tr) const;
};

#endif 
