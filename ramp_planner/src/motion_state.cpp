#include "motion_state.h"


const std::string MotionState::toString() const {
  std::ostringstream result;
  
  //Positions
  result<<"\np: ["<<positions_.at(0);
  for(unsigned int i=1;i<positions_.size();i++) {
    result<<", "<<positions_.at(i);
  }
  result<<"]";

  //Velocities
  result<<"\nv: ["<<velocities_.at(0);
  for(unsigned int i=1;i<velocities_.size();i++) {
    result<<", "<<velocities_.at(i);
  }
  result<<"]";

  //Accelerations
  result<<"\na: ["<<accelerations_.at(0);
  for(unsigned int i=1;i<accelerations_.size();i++) {
    result<<", "<<accelerations_.at(i);
  }
  result<<"]";

  // Jerks
  result<<"\nj: ["<<jerks_.at(0);
  for(unsigned int i=1;i<jerks_.size();i++) {
    result<<", "<<jerks_.at(i);
  }
  result<<"]";


  return result.str();
}
