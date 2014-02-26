#include "knot_point.h"

KnotPoint::KnotPoint() : stop_time_(0) {}

KnotPoint::KnotPoint(const Configuration c) : configuration_(c), stop_time_(0) {}

KnotPoint::KnotPoint(const ramp_msgs::KnotPoint kp) {
  configuration_ = kp.configuration;
}


const ramp_msgs::KnotPoint KnotPoint::buildKnotPointMsg() const {
  ramp_msgs::KnotPoint result;

  result.configuration = configuration_.buildConfigurationMsg();
  result.stop_time = stop_time_;

  return result;
}


const std::string KnotPoint::toString() const {
  std::ostringstream result;
  
  result<<"Configuration: "<<configuration_.toString();
  result<<"Stop time: "<<stop_time_;

  return result.str(); 
}
