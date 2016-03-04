#ifndef ORIENTATION_H
#define ORIENTATION_H
#include "utility.h"
#include "ramp_msgs/RampTrajectory.h"

class Orientation {
  public:
    Orientation();

    const double perform();
    const double getPenalty() const;

    double currentTheta_;
    ramp_msgs::RampTrajectory trajectory_;

    double Q_;
    
    Utility utility_;
};

#endif
