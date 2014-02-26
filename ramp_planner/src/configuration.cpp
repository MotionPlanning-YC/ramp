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


const bool Configuration::equals(const Configuration& c) const {
  
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
const double Configuration::compare(const Configuration& c, bool base_theta) const {
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
std::vector<float> Configuration::transformBasePosition(const Eigen::Transform<float, 2, Eigen::Affine> T_od_w) {

  std::vector<float> result;
    
  Eigen::Vector3f p(K_.data());
  p[2] = 1;

  Eigen::Vector3f p2 = T_od_w * p;

  result.push_back(p2[0]);
  result.push_back(p2[1]);

  return result;
}



/** This method returns the orientation of the mobile base rotated by theta */
float Configuration::transformBaseOrientation(const float theta) {
  float result=0.;
  float sum = K_.at(2) + theta;

  // If two angles are positive, but result should be negative
  if(sum > PI) {
    sum     = fmodf(sum, PI);
    result  = sum - PI;
  }

  // If 2 angles are negative, but result should be positive
  else if(sum < -PI) {
    result  = sum + (2*PI);
  }

  else {
    result  = sum;
  }

  return result;
} //End getOrientation



/** This method will transform the configuration by the matrix T 
 *  The rotation of T is also passed in for ease 
 *  The most used source of this method is for updating the robot's configuration */
void Configuration::transformBase(const Eigen::Transform<float, 2, Eigen::Affine> T_od_w, float theta) {

  // Get the new position
  std::vector<float> p_w = transformBasePosition(T_od_w);
  K_.at(0) = p_w.at(0);
  K_.at(1) = p_w.at(1);
  
  // Get the new orientation
  K_.at(2) = transformBaseOrientation(theta);
} //End add



const std::string Configuration::toString() const {
  std::ostringstream result;
  
  result<<"("<<K_.at(0);
  for(unsigned int i=1;i<K_.size();i++) {
    result<<", "<<K_.at(i);
  }
  result<<")";

  return result.str(); 
} //End toString


