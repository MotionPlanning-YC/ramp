#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include "range.h"
#include "ramp_msgs/Configuration.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

class Configuration {
  public:
    Configuration();
    Configuration(ramp_msgs::Configuration c);
    Configuration(const trajectory_msgs::JointTrajectoryPoint p, const std::vector<Range> r);
    ~Configuration();
    

    //This holds the values for all the DOFs of a robot that represent a pose
    std::vector<float> K_; 

    //This holds the ranges for each DOF
    std::vector<Range> ranges_;
    
    void random();
    const bool equals(const Configuration& c) const; 
    const double compare(const Configuration& c) const;
    const ramp_msgs::Configuration buildConfigurationMsg() const;
    const std::string toString() const;
    void updatePosition(float x, float y, float theta);
};
#endif
