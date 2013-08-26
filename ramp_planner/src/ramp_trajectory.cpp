#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory() : fitness_(-1.0) {}


const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory: "<<u.toString(trajec_);
  result<<"\nFitness: "<<fitness_<<" Feasible: "<<feasible_;
  
  return result.str();
}
