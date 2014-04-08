#include "motion_state.h"

MotionState::MotionState() : mobile_base_k_(2) {}


MotionState::MotionState(const trajectory_msgs::JointTrajectoryPoint p) : mobile_base_k_(2) {
  for(unsigned int i=0;i<p.positions.size();i++) {
    positions_.push_back(p.positions.at(i));
  }
  
  for(unsigned int i=0;i<p.velocities.size();i++) {
    velocities_.push_back(p.velocities.at(i));
  }
  
  for(unsigned int i=0;i<p.accelerations.size();i++) {
    accelerations_.push_back(p.accelerations.at(i));
  }
}

MotionState::MotionState(const ramp_msgs::MotionState ms) {
  for(unsigned int i=0;i<ms.positions.size();i++) {
    positions_.push_back(ms.positions.at(i));
  }
  
  for(unsigned int i=0;i<ms.velocities.size();i++) {
    velocities_.push_back(ms.velocities.at(i));
  }
  
  for(unsigned int i=0;i<ms.accelerations.size();i++) {
    accelerations_.push_back(ms.accelerations.at(i));
  }
  
  for(unsigned int i=0;i<ms.jerks.size();i++) {
    jerks_.push_back(ms.jerks.at(i));
  }

  time_ = ms.time;
}


MotionState::MotionState(const Configuration c) : mobile_base_k_(2) {
  for(unsigned int i=0;i<c.K_.size();i++) {
    positions_.push_back(c.K_.at(i));
  }
}



/** 
 * This method returns the euclidean distance between this configuration and c 
 * if base_theta is true, we are considering the base orientation, otherwise do not
 * add base orientation difference into the result
 * */
double MotionState::comparePosition(const MotionState& c, bool base_theta) const {
  //std::cout<<"\nComparing: "<<toString()<<" and "<<c.toString();
  double result = 0; 

  // For each DOF, sum the (X2-X1)^2
  for(unsigned int i=0;i<positions_.size();i++) {
    // If we are not taking base theta into account, skip i
    if(i == mobile_base_k_ && !base_theta) {}

    // Else if we are considering base theta, use utility function
    else if(i == mobile_base_k_) {
      result += pow(utility.displaceAngle(c.positions_.at(i), positions_.at(i)), 2);
    }
    else
      result += pow(positions_.at(i) - positions_.at(i), 2);
  }

  // Get square root to complete euclidean distance...
  result = sqrt(result);

  return result;
}




const ramp_msgs::MotionState MotionState::buildMotionStateMsg() const {
  ramp_msgs::MotionState result;

  for(unsigned int i=0;i<positions_.size();i++) {
    result.positions.push_back(positions_.at(i));
  }

  for(unsigned int i=0;i<velocities_.size();i++) {
    result.velocities.push_back(velocities_.at(i));
  }

  for(unsigned int i=0;i<accelerations_.size();i++) {
    result.accelerations.push_back(accelerations_.at(i));
  }

  for(unsigned int i=0;i<jerks_.size();i++) {
    result.jerks.push_back(jerks_.at(i));
  }

  result.time = time_;
  
  return result;
}



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
