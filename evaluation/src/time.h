#ifndef TIME_H
#define TIME_H
#include "utility.h"
#include "ramp_msgs/Trajectory.h"

class Time {
  public:
    Time() {}

    const double perform();

    ramp_msgs::Trajectory trajectory_;
};

#endif
