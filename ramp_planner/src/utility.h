#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <vector>
#include <queue>
#include <sstream>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Trajectory.h"
#include <tf/transform_datatypes.h>

#define PI 3.14159f


class Range;

class Utility {
  public:
    Utility();

    std::vector<Range> standardRanges;
    const std::string toString(const ramp_msgs::Path path) const;
    const std::string toString(const ramp_msgs::Trajectory traj) const;
};

#endif 
