#include "configuration.h"


Configuration::Configuration() {}

Configuration::Configuration(ramp_msgs::Configuration c) {
  //std::cout<<"\nc.K.size(): "<<c.K.size();
  //std::cout<<"\nc.ranges.size(): "<<c.ranges.size()<<"\n";
  
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

/** This method returns the euclidean distance between this configuration and c */
const double Configuration::compare(const Configuration& c) const {
  //std::cout<<"\nComparing: "<<toString()<<" and "<<c.toString();
  double result = 0; 

  //For each DOF, sum the (X2-X1)^2
  for(unsigned int i=0;i<ranges_.size();i++) {
    result += pow(c.K_.at(i) - K_.at(i), 2);
  }

  //Get square root to complete euclidean distance...
  result = sqrt(result);

  //std::cout<<"\nReturning "<<result;
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

const std::string Configuration::toString() const {
  std::ostringstream result;
  
  result<<"("<<K_.at(0);
  for(unsigned int i=1;i<K_.size();i++) {
    result<<", "<<K_.at(i);
  }
  result<<")";

  return result.str(); 
}
