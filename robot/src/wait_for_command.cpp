#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

void twistCallback(const geometry_msgs::Twist msg) {
  std::cout<<"\nGot twist message!\n";
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "waitForCommand");
  ros::NodeHandle handle;

  ros::Subscriber sub_twist = handle.subscribe("twist", 1000, twistCallback);
  

  std::cout<<"\nExiting Normally\n";
  return 0;
}
