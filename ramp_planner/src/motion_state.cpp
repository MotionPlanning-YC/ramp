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

MotionState::MotionState(const ramp_msgs::MotionState ms) : mobile_base_k_(2) {
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





/** 
 * This method returns the euclidean distance between this configuration and c 
 * if base_theta is true, we are considering the base orientation, otherwise do not
 * add base orientation difference into the result
 * */
double MotionState::comparePosition(const MotionState& c, const bool base_theta) const {
  double result = 0; 

  if(mobile_base_k_ != 2)
    std::cout<<"\nmobile_base_k: "<<mobile_base_k_;

  // For each DOF, sum the (X2-X1)^2
  for(unsigned int i=0;i<positions_.size();i++) {
    // If we are not taking base theta into account, skip i
    if(i == mobile_base_k_ && !base_theta) {}

    // Else if we are considering base theta, use utility function
    else if(i == mobile_base_k_) {
      result += pow(utility.displaceAngle(c.positions_.at(i), positions_.at(i)), 2);
    }
    else {
      result += pow(c.positions_.at(i) - positions_.at(i), 2);
    }
  }

  // Get square root to complete euclidean distance
  result = sqrt(result);

  return result;
}



/** This method returns the new position vector of the Configuration given some transformation matrix */
tf::Vector3 MotionState::transformBasePosition(const tf::Transform t) {

  tf::Vector3 p(positions_.at(0), positions_.at(1), 0);
  tf::Vector3 result = t * p;

  return result;
} //End transformBasePosition



/** This method will transform the configuration by the transformation T
 *  It transforms the position and displaces the orientation by the rotation in T 
 *  The most used source of this method is for updating the robot's configuration */
void MotionState::transformBase(const tf::Transform t) {

  // Get the new position
  tf::Vector3 p = transformBasePosition(t);
  positions_.at(0) = p.getX();
  positions_.at(1) = p.getY();
  
  // Get the new orientation
  positions_.at(2) = utility.displaceAngle(positions_.at(2), tf::getYaw(t.getRotation()));
} //End transformBase



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
  if(positions_.size() == 0) {
    result<<"\np: []";
  }
  else {
    result<<"\np: ["<<positions_.at(0);
    for(unsigned int i=1;i<positions_.size();i++) {
      result<<", "<<positions_.at(i);
    }
    result<<"]";
  }

  //Velocities
  if(velocities_.size() == 0) {
    result<<"\nv: []";
  }
  else {
    result<<"\nv: ["<<velocities_.at(0);
    for(unsigned int i=1;i<velocities_.size();i++) {
      result<<", "<<velocities_.at(i);
    }
    result<<"]";
  }

  //Accelerations
  if(accelerations_.size() > 0) {
    result<<"\na: ["<<accelerations_.at(0);
    for(unsigned int i=1;i<accelerations_.size();i++) {
      result<<", "<<accelerations_.at(i);
    }
    result<<"]";
  }

  // Jerks
  if(jerks_.size() > 0) {
    result<<"\nj: ["<<jerks_.at(0);
    for(unsigned int i=1;i<jerks_.size();i++) {
      result<<", "<<jerks_.at(i);
    }
    result<<"]";
  }


  return result.str();
}
