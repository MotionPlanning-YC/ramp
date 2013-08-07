#include "motion_state.h"


const std::string MotionState::toString() const {
  std::ostringstream result;
  
  //Positions
  result<<"\nPositions: ["<<p_.at(0);
  for(unsigned int i=1;i<p_.size();i++) {
    result<<", "<<p_.at(i);
  }
  result<<"]";

  //Velocities
  result<<"\nVelocities: ["<<v_.at(0);
  for(unsigned int i=1;i<v_.size();i++) {
    result<<", "<<v_.at(i);
  }
  result<<"]";

  //Accelerations
  result<<"\nAccelerations: ["<<a_.at(0);
  for(unsigned int i=1;i<a_.size();i++) {
    result<<", "<<a_.at(i);
  }
  result<<"]";

  return result.str();
}
