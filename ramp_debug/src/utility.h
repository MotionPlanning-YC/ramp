#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <math.h>
#include <vector>
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/Path.h"
#include <tf/transform_datatypes.h>


#define PI 3.14159f

//Modified: Add a structure
struct obstacle_struct // defines where an obstacle is
{
  double x1;
  double x2;
  double y1;
  double y2;
};


class Utility {
  public:
    Utility();
    ~Utility();

    std::vector<ramp_msgs::Range> ranges_;
    
    const float euclideanDistance(const std::vector<float> a, const std::vector<float> b) const;

    const float findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const;
    
    const float findDistanceBetweenAngles(const float a1, const float a2) const;
    
    const float displaceAngle(const float a1, float a2) const;
    
    const ramp_msgs::Configuration getConfigurationFromPoint(const trajectory_msgs::JointTrajectoryPoint p) const;
    const ramp_msgs::Path getPath(const std::vector<ramp_msgs::Configuration> configs) const;
    const ramp_msgs::Path getPath(const std::vector<ramp_msgs::KnotPoint> kps) const;


    const std::string toString(const ramp_msgs::Trajectory traj) const;
    const std::string toString(const ramp_msgs::Path p) const;
    const std::string toString(const ramp_msgs::Configuration c) const;
    const std::string toString(const ramp_msgs::KnotPoint kp) const;
};

#endif
