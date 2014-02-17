#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory(unsigned int id) : id_(id), fitness_(-1.0), feasible_(true), time_until_collision_(9999.0f) {
  msg_trajec_.feasible = true;
  msg_trajec_.fitness = -1;  
}

RampTrajectory::RampTrajectory(const ramp_msgs::Trajectory msg) : fitness_(-1.0), feasible_(true), msg_trajec_(msg), time_until_collision_(9999.0f) {}

const bool RampTrajectory::equal(const RampTrajectory& other) const {
  return id_ == other.id_;
}


const Path RampTrajectory::getPath() const {
  Path result;

  for(unsigned int i=0;i<msg_trajec_.index_knot_points.size();i++) {

    Configuration c(msg_trajec_.trajectory.points.at( msg_trajec_.index_knot_points.at(i)), u.standardRanges );
  
    result.all_.push_back(c);
  }

  result.start_ = result.all_.at(0);
  result.goal_ = result.all_.at( result.all_.size()-1 );
  
  return result;
}



const std::string RampTrajectory::fitnessFeasibleToString() const {
  std::ostringstream result;
 
  result<<"\nTrajectory "<<id_;
  result<<"\n Number of knot points: "<<msg_trajec_.index_knot_points.size(); 
  result<<"\n Fitness: "<<fitness_<<" Feasible: "<<feasible_<<" Collision Time: "<<time_until_collision_;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory "<<id_<<": "<<u.toString(msg_trajec_);
  result<<fitnessFeasibleToString();
  
  return result.str();
}


