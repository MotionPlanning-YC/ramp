#include "configuration.h"


Configuration::Configuration() : mobile_base_k_(2) {}

Configuration::Configuration(ramp_msgs::Configuration c) : mobile_base_k_(2) {
  
  for(unsigned int i=0;i<c.K.size();i++) {
    K_.push_back(c.K.at(i));
    ranges_.push_back(c.ranges.at(i));
  }
}


Configuration::Configuration(const trajectory_msgs::JointTrajectoryPoint p, const std::vector<Range> r) {

  for(unsigned int i=0;i<r.size();i++) {
    ranges_.push_back(r.at(i));
  }
  
  for(unsigned int i=0;i<p.positions.size();i++) {
    K_.push_back(p.positions.at(i));
  }
}

Configuration::~Configuration() {}

void Configuration::updatePosition(float x, float y, float theta)
{
  K_.clear();
  K_.push_back(x);
  K_.push_back(y);
  K_.push_back(theta);
}

/** Set the configuration to be of random values and return the value of this configuration */
void Configuration::random() {

  for(unsigned int i=0;i<ranges_.size();i++) {
    K_.push_back(ranges_.at(i).random());
  }

}


bool Configuration::equals(const Configuration& c) const {
  
  if(K_.size() != c.K_.size()) {
    return false;
  }
  
  for(unsigned int i=0;i<K_.size();i++) {
    if(K_.at(i) != c.K_.at(i)) {
      return false;
    }
  }

  return true;
}

/** 
 * This method returns the euclidean distance between this configuration and c 
 * if base_theta is true, we are considering the base orientation, otherwise do not
 * add base orientation difference into the result
 * */
double Configuration::compare(const Configuration& c, bool base_theta) const {
  //std::cout<<"\nComparing: "<<toString()<<" and "<<c.toString();
  double result = 0; 

  // For each DOF, sum the (X2-X1)^2
  for(unsigned int i=0;i<ranges_.size();i++) {
    // If we are not taking base theta into account, skip i
    if(i == mobile_base_k_ && !base_theta) {}

    // Else if we are considering base theta, use utility function
    else if(i == mobile_base_k_) {
      result += pow(u.displaceAngle(c.K_.at(i), K_.at(i)), 2);
    }
    else
      result += pow(c.K_.at(i) - K_.at(i), 2);
  }

  // Get square root to complete euclidean distance...
  result = sqrt(result);

  return result;
}



/** This method creates a Configuration msg out of this object that can be published on a ROS topic */
const ramp_msgs::Configuration Configuration::buildConfigurationMsg() const {
  ramp_msgs::Configuration result;

  //Push all of the DOF values onto the msg's DOF values
  for(unsigned int i=0;i<K_.size();i++) {
    result.K.push_back(K_.at(i));
    result.ranges.push_back(ranges_.at(i).buildRangeMsg());
  }

  return result;
}

/** This method returns the new position vector of the Configuration given some transformation matrix */
tf::Vector3 Configuration::transformBasePosition(const tf::Transform t) {

  tf::Vector3 p(K_.at(0), K_.at(1), 0);
  tf::Vector3 result = t * p;

  return result;
} //End transformBasePosition


/** This method will transform the configuration by the transformation T
 *  It transforms the position and displaces the orientation by the rotation in T 
 *  The most used source of this method is for updating the robot's configuration */
void Configuration::transformBase(const tf::Transform t) {

  // Get the new position
  tf::Vector3 p = transformBasePosition(t);
  K_.at(0) = p.getX();
  K_.at(1) = p.getY();
  
  // Get the new orientation
  K_.at(2) = u.displaceAngle(K_.at(2), tf::getYaw(t.getRotation()));
} //End transformBase



/** toString for Configurations */
const std::string Configuration::toString() const {
  std::ostringstream result;
  
  result<<"("<<K_.at(0);
  for(unsigned int i=1;i<K_.size();i++) {
    result<<", "<<K_.at(i);
  }
  result<<")";

  return result.str(); 
} //End toString


