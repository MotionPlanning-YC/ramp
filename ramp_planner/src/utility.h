#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <vector>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "ramp_msgs/Trajectory.h"

class Utility {
  public:
    const std::string toString(const ramp_msgs::Trajectory traj) const;
};

#endif 
