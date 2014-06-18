#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory(const float resRate, unsigned int id) : id_(id), fitness_(-1.0), feasible_(true), time_until_collision_(9999.0f), resolutionRate_(resRate) {
  msg_trajec_.feasible = true;
  msg_trajec_.fitness = -1;  
}

RampTrajectory::RampTrajectory(const ramp_msgs::Trajectory msg) : msg_trajec_(msg), fitness_(msg.fitness), feasible_(msg.feasible), time_until_collision_(9999.0f) {}


const bool RampTrajectory::equal(const RampTrajectory& other) const {
  return id_ == other.id_;
}


const Path RampTrajectory::getPath() const {
  Path result;

  for(unsigned int i=0;i<msg_trajec_.index_knot_points.size();i++) {

    MotionState ms(msg_trajec_.trajectory.points.at( msg_trajec_.index_knot_points.at(i)));
  
    result.all_.push_back(ms);
  }

  result.start_ = result.all_.at(0);
  result.goal_  = result.all_.at( result.all_.size()-1 );
  
  return result;
}


/** Time is in seconds */
const trajectory_msgs::JointTrajectoryPoint RampTrajectory::getPointAtTime(const float t) const {
  if( (t/resolutionRate_) > msg_trajec_.trajectory.points.size() ) {
    return msg_trajec_.trajectory.points.at( msg_trajec_.trajectory.points.size()-1 );
  }

  return msg_trajec_.trajectory.points.at( (t / resolutionRate_) );
}





/** Returns the direction of the trajectory, i.e. the
* orientation the base needs to move on the trajectory */
const double RampTrajectory::getDirection() const {
  trajectory_msgs::JointTrajectoryPoint a = msg_trajec_.trajectory.points.at(0);
  trajectory_msgs::JointTrajectoryPoint b = msg_trajec_.trajectory.points.at(msg_trajec_.index_knot_points.at(1));

  return utility_.findAngleFromAToB(a, b);
}



const RampTrajectory RampTrajectory::clone() const { 
  return *this;
}


const std::string RampTrajectory::fitnessFeasibleToString() const {
  std::ostringstream result;
 
  result<<"\nTrajectory ID: "<<id_;
  result<<"\n Number of knot points: "<<msg_trajec_.index_knot_points.size(); 
  result<<"\n Path: "<<path_.toString();
  result<<"\n Fitness: "<<fitness_<<" Feasible: "<<feasible_<<" Collision Time: "<<time_until_collision_;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<id_<<"\n"<<utility_.toString(msg_trajec_);
  result<<fitnessFeasibleToString();
  
  return result.str();
}


