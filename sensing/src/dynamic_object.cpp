#include "dynamic_object.h"

DynamicObject::DynamicObject() {
  trajectory_.push_back(0.0f);
  trajectory_.push_back(0.0f);
  trajectory_.push_back(0.0f);

  current_pose_.position.x = 0;
  current_pose_.position.y = 0;
  current_pose_.position.z = 0;
}

DynamicObject::DynamicObject(geometry_msgs::Pose p) : current_pose_(p) {
  trajectory_.push_back(0.0f);
  trajectory_.push_back(0.0f);
  trajectory_.push_back(0.0f);
}

DynamicObject::~DynamicObject() {}


void DynamicObject::updatePose(geometry_msgs::Pose p, ros::Time t) {
  
  //Find velocities
  ros::Duration delta_t = last_updated_ - t;

  //X
  trajectory_.at(0) = (p.position.x - current_pose_.position.x) / delta_t.toSec();
  //Y
  trajectory_.at(1) = (p.position.y - current_pose_.position.y) / delta_t.toSec();
  //Z
  trajectory_.at(2) = (p.position.z - current_pose_.position.z) / delta_t.toSec();

  //Set new pose 
  current_pose_ = p; 
}


ramp_msgs::Object DynamicObject::buildObjectMsg() {
  ramp_msgs::Object result;

  result.pose = current_pose_;
  result.trajectory = trajectory_;
  
  return result;
}
