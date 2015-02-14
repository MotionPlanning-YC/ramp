#ifndef TIME_H
#define TIME_H
#include "utility.h"
#include "ramp_msgs/RampTrajectory.h"

class Time {
  public:
    Time() {}

    const double perform();

    ramp_msgs::RampTrajectory trajectory_;

    Utility utility_;
};

#endif
