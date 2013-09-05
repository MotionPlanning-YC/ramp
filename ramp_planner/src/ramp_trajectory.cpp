#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory() : fitness_(-1.0), feasible_(true) {}



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
  
  result<<"\nFitness: "<<fitness_<<" Feasible: "<<feasible_;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory: "<<u.toString(msg_trajec_);
  result<<fitnessFeasibleToString();
  
  return result.str();
}
