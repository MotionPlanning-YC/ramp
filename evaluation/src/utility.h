#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <math.h>
#include "ramp_msgs/Trajectory.h"

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

    const std::string toString(const ramp_msgs::Trajectory traj) const;
};

#endif
