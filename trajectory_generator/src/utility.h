#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Path.h"

class Utility {
  public:
    const std::string toString(const ramp_msgs::TrajectoryRequest::Request tr) const;
    const std::string toString(const ramp_msgs::Path p) const;
};
#endif
