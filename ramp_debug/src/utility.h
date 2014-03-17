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
#include "nav_msgs/Odometry.h"
#include "motion_type.h"
#include <tf/transform_datatypes.h>
#include "range.h"

#define PI 3.14159f


class Utility {
  public:
    Utility();

    std::vector<Range> standardRanges;
    std::vector<ramp_msgs::Range> ranges_;
    
    const float euclideanDistance(const std::vector<float> a, const std::vector<float> b) const;

    const float findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    
    const float findDistanceBetweenAngles(const float a1, const float a2) const;
    
    const float displaceAngle(const float a1, float a2) const;
    
    const ramp_msgs::Configuration getConfigurationFromPoint(const trajectory_msgs::JointTrajectoryPoint p) const;
    const ramp_msgs::Path getPath(const std::vector<ramp_msgs::KnotPoint> configs) const;
    
    
    const std::vector<float> getCenter(std::vector<float> p, float orientation) const;
    
    const std::string toString(const ramp_msgs::Path path) const;
    const std::string toString(const ramp_msgs::Trajectory traj) const;
};

#endif 
