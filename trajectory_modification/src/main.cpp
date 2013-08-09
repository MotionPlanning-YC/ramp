#include <iostream>
#include "ros/ros.h"
#include "subscribe_and_publish.h"


void callback(const ramp_msgs::ModificationRequest::ConstPtr& msg) {
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_modification");
  
  ros::NodeHandle handle;

  SubscribeAndPublish sap(handle);
  

  std::cout<<"\nSpinning...\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
