#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include "range.h"
#include "ramp_msgs/Configuration.h"

class Configuration {
  public:
    Configuration();
    Configuration(ramp_msgs::Configuration c);
    ~Configuration();
    

    //This holds the values for all the DOFs of a robot that represent a pose
    std::vector<float> K_; 

    //This holds the ranges for each DOF
    std::vector<Range> ranges_;
    
    Configuration random();
    const ramp_msgs::Configuration buildConfigurationMsg() const;
    const std::string toString() const;
};
#endif
