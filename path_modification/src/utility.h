#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <sstream>
#include "ramp_msgs/Path.h"
#include "ramp_msgs/Range.h"

#define PI 3.14159f

class Utility {
  public:
    Utility();

    std::vector<ramp_msgs::Range> standardRanges_;

    const std::string toString(const ramp_msgs::Path p) const;    
};
#endif
