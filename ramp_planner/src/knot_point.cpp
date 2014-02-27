#include "knot_point.h"

KnotPoint::KnotPoint() : stop_time_(0) {}

KnotPoint::KnotPoint(const Configuration c) : configuration_(c), stop_time_(0) {}

KnotPoint::KnotPoint(const ramp_msgs::KnotPoint kp) {
  configuration_ = kp.configuration;
  stop_time_ = kp.stop_time;
}


const ramp_msgs::KnotPoint KnotPoint::buildKnotPointMsg() const {
  ramp_msgs::KnotPoint result;

  result.configuration = configuration_.buildConfigurationMsg();
  result.stop_time = stop_time_;
  //std::cout<<"\nstop_time_: "<<stop_time_;
  //std::cout<<"\nresult.stop_time: "<<result.stop_time<<"\n";

  return result;
}


const std::string KnotPoint::toString() const {
  std::ostringstream result;
  
  result<<"Configuration: "<<configuration_.toString();
  result<<" Stop time: "<<stop_time_;

  return result.str(); 
}
