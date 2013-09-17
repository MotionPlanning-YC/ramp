#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <math.h>
#include "ramp_msgs/Trajectory.h"

class Utility {
  public:

    const std::string toString(const ramp_msgs::Trajectory traj) const;
};

#endif
