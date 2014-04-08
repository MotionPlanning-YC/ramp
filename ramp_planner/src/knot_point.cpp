#include "knot_point.h"

KnotPoint::KnotPoint() : stopTime_(0) {}

KnotPoint::KnotPoint(const MotionState mp) : motionState_(mp), stopTime_(0) {}

KnotPoint::KnotPoint(const ramp_msgs::KnotPoint kp) {
  motionState_ = kp.motionState;
  stopTime_ = kp.stopTime;
}


const ramp_msgs::KnotPoint KnotPoint::buildKnotPointMsg() const {
  ramp_msgs::KnotPoint result;

  result.motionState = motionState_.buildMotionStateMsg();
  result.stopTime = stopTime_;
  //std::cout<<"\nstop_time_: "<<stop_time_;
  //std::cout<<"\nresult.stop_time: "<<result.stop_time<<"\n";

  return result;
}


const std::string KnotPoint::toString() const {
  std::ostringstream result;
  
  result<<"Configuration: "<<motionState_.toString();
  result<<" Stop time: "<<stopTime_;

  return result.str(); 
}
