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
const double MotionState::comparePosition(const MotionState& c, const bool base_theta) const {
  double result = 0; 

  // For each DOF, sum the (X2-X1)^2
  for(unsigned int i=0;i<positions_.size();i++) {
    // If we are not taking base theta into account, skip i
    if(i == mobile_base_k_ && !base_theta) {}

    // Else if we are considering base theta, use utility_ function
    else if(i == mobile_base_k_) {
      result += pow(utility_.displaceAngle(c.positions_.at(i), positions_.at(i)), 2);
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
  positions_.at(2) = utility_.displaceAngle(positions_.at(2), tf::getYaw(t.getRotation()));
} //End transformBase





/** */
const MotionState MotionState::add(const MotionState m) const {
  MotionState result = *this;

  for(int i=0;i<positions_.size() && i<m.positions_.size();i++) {
    if(i != mobile_base_k_) {
      result.positions_.at(i) += m.positions_.at(i);
    }
    else {
      result.positions_.at(i) = utility_.displaceAngle(positions_.at(i), m.positions_.at(i));
    }
  }

  /** Separate loops because it's not guaranteed that every MS will have
   * same # of each vector */
  for(int i=0;i<velocities_.size() && i<m.velocities_.size();i++) {
    result.velocities_.at(i) += m.velocities_.at(i);
  }

  for(int i=0;i<accelerations_.size() && i<m.accelerations_.size();i++) {
    result.accelerations_.at(i) += m.accelerations_.at(i);
  }

  for(int i=0;i<jerks_.size() && i<m.jerks_.size();i++) {
    result.jerks_.at(i) += m.jerks_.at(i);
  }

  return result;
} // End add




/** */
const MotionState MotionState::subtract(const MotionState m) const {
  MotionState result = *this;

  for(int i=0;i<positions_.size() && i<m.positions_.size();i++) {
    if(i != mobile_base_k_) {
      result.positions_.at(i) -= m.positions_.at(i);
    }
    else {
      result.positions_.at(i) = utility_.displaceAngle(positions_.at(i), -m.positions_.at(i));
    }
  }

  /** Separate loops because it's not guaranteed that every MS will have
   * same # of each vector */
  for(int i=0;i<velocities_.size() && i<m.velocities_.size();i++) {
    result.velocities_.at(i) -= m.velocities_.at(i);
  }

  for(int i=0;i<accelerations_.size() && i<m.accelerations_.size();i++) {
    result.accelerations_.at(i) -= m.accelerations_.at(i);
  }

  for(int i=0;i<jerks_.size() && i<m.jerks_.size();i++) {
    result.jerks_.at(i) -= m.jerks_.at(i);
  }

  return result;
} // End subtract



/** Probably not going to stay here 
 * Used for dividing delta_m in the planner code
 * may need to use displaceAngle for theta position */
const MotionState MotionState::multiply(const int num) const {
  MotionState result = *this;

  for(int i=0;i<positions_.size();i++) {
    result.positions_.at(i) *= num;
  }
  
  for(int i=0;i<velocities_.size();i++) {
    result.velocities_.at(i) *= num;
  }

  for(int i=0;i<accelerations_.size();i++) {
    result.accelerations_.at(i) *= num;
  }

  for(int i=0;i<jerks_.size();i++) {
    result.jerks_.at(i) *= num;
  }

  return result;
}




/** Probably not going to stay here 
 * Used for dividing delta_m in the planner code */
const MotionState MotionState::divide(const int num) const {
  MotionState result = *this;

  for(int i=0;i<positions_.size();i++) {
      result.positions_.at(i) /= num;
  }
  
  for(int i=0;i<velocities_.size();i++) {
      result.velocities_.at(i) /= num;
  }

  for(int i=0;i<accelerations_.size();i++) {
      result.accelerations_.at(i) /= num;
  }

  for(int i=0;i<jerks_.size();i++) {
      result.jerks_.at(i) /= num;
  }

  return result;
}



const MotionState MotionState::abs() const {
  MotionState result = *this;

  for(unsigned int i=0;i<positions_.size();i++) {
    result.positions_.at(i) = fabs(result.positions_.at(i));
  }

  for(unsigned int i=0;i<velocities_.size();i++) {
    result.velocities_.at(i) = fabs(result.velocities_.at(i));
  }

  for(unsigned int i=0;i<accelerations_.size();i++) {
    result.accelerations_.at(i) = fabs(result.accelerations_.at(i));
  }
  
  for(unsigned int i=0;i<jerks_.size();i++) {
    result.jerks_.at(i) = fabs(result.jerks_.at(i));
  }

  return result;
}



const double MotionState::normPosition() const {
  double result = 0;

  for(uint16_t i=0;i<positions_.size();i++) {
    result += pow(positions_.at(i), 2);
  }

  result = sqrt(result);

  return result;
}


const double MotionState::normVelocity() const {
  double result = 0;

  for(uint16_t i=0;i<velocities_.size();i++) {
    result += pow(velocities_.at(i), 2);
  }

  result = sqrt(result);

  return result;
}


const double MotionState::normAcceleration() const {
  double result = 0;

  for(uint16_t i=0;i<accelerations_.size();i++) {
    result += pow(accelerations_.at(i), 2);
  }

  result = sqrt(result);

  return result;
}



const double MotionState::normJerk() const {
  double result = 0;

  for(uint16_t i=0;i<jerks_.size();i++) {
    result += pow(jerks_.at(i), 2);
  }

  result = sqrt(result);

  return result;
}


const double MotionState::norm() const {
  double result = 0;

  result += normPosition();
  result += normVelocity();
  result += normAcceleration();
  result += normJerk();

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

  result<<"\n----\n";

  return result.str();
}
