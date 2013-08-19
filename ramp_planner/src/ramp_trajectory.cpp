#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory() : fitness_(-1.0) {}


const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  result<<"\n"<<u.toString(trajec_);
  result<<"\nFitness: "<<fitness_;
  return result.str();
}
