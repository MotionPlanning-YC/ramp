#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include "range.h"

class Configuration {
  public:
    Configuration();
    ~Configuration();
    
    Configuration random();

    //This holds the values for all the DOFs of a robot that represent a pose
    std::vector<float> K_; 

    //This holds the ranges for each DOF
    std::vector<Range> ranges_;

    const std::string toString() const;
};
#endif
