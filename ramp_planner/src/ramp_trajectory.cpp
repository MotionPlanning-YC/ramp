#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory() : fitness_(-1.0) {}


const std::string RampTrajectory::toString() const {
  std::cout<<"\nIn RampTrajectory tostring\n";
  std::ostringstream result;
  result<<"\n"<<u.toString(trajec_);
  std::cout<<"\nAfter u.toString(trajec_)\n";
  result<<"\nFitness: "<<fitness_;
  return result.str();
}
