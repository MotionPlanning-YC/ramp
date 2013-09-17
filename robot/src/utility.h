#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include "ramp_msgs/Range.h"

class Utility {
  public:
    Utility();

    std::vector<ramp_msgs::Range> standardRanges;
};

#endif
