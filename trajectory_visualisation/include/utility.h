#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <math.h>
#include <limits>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include <tf/transform_datatypes.h>
#include <ros/console.h>

#define PI 3.14159f

enum TrajectoryType {
  ALL_STRAIGHT_SEGMENTS = 0,
  ALL_BEZIER            = 1,
  PARTIAL_BEZIER        = 2,
  TRANSITION            = 3
};


class Utility {
  public:
    Utility();
    
    const double positionDistance(const std::vector<double> a, const std::vector<double> b) const;

    const double findAngleFromAToB(const trajectory_msgs::JointTrajectoryPoint a, const trajectory_msgs::JointTrajectoryPoint b) const;
    const double findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    const double findAngleFromAToB(const std::vector<double> a, const std::vector<double> b) const;
    
    const double findDistanceBetweenAngles(const double a1, const double a2) const;
    
    const double displaceAngle(const double a1, double a2) const;
    
    const double getEuclideanDist(const std::vector<double> a, std::vector<double> b) const;

    const ramp_msgs::Path getPath(const std::vector<ramp_msgs::MotionState> mps) const;
    const ramp_msgs::Path getPath(const std::vector<ramp_msgs::KnotPoint>   kps) const;

    
    const std::string toString(const ramp_msgs::MotionState mp) const;
    const std::string toString(const ramp_msgs::KnotPoint kp) const;
    const std::string toString(const ramp_msgs::Path path) const;
    const std::string toString(const ramp_msgs::BezierCurve bi) const;
    const std::string toString(const ramp_msgs::RampTrajectory traj) const;
    const std::string toString(const trajectory_msgs::JointTrajectoryPoint p) const;
};


#endif 
