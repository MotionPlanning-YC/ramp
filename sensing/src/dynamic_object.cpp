#include "dynamic_object.h"

DynamicObject::DynamicObject() {
  nav_msgs::Odometry temp;
  odom_.push_back(temp);
  odom_.push_back(temp);
}

DynamicObject::DynamicObject(nav_msgs::Odometry o) {
  odom_.push_back(o);
  odom_.push_back(o);
}

DynamicObject::~DynamicObject() {}


void DynamicObject::update(nav_msgs::Odometry o) {

  //Update the odom vector
  //Set t odom to t-1 odom
  for(unsigned int i=0;i<odom_.size()-1;i++) {
    odom_.at(i) = odom_.at(i+1);
  }

  //Remove oldest odometry
  odom_.pop_back();

  //Push on new odom
  odom_.push_back(o);

  //Update time
  last_updated_ = ros::Time::now();
}


ramp_msgs::Object DynamicObject::buildObjectMsg() {
  ramp_msgs::Object result;

  result.odom = odom_.back();
  
  return result;
}
