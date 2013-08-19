#ifndef UTILITY_H
#define UTILITY_H
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <sstream>
#include "ramp_msgs/Path.h"

class Utility {
  public:
    const std::string toString(const ramp_msgs::Path p) const;    
    const std::string toString(const ramp_msgs::Configuration c) const;
};
#endif
