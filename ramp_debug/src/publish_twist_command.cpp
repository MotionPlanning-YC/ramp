#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_twist_command");
  ros::NodeHandle handle;

  ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("twist_single", 1000);

  geometry_msgs::Twist t;
  t.linear.x = 0.25f;
  t.linear.y = 0.f;
  t.linear.z = 0.f;
  t.angular.x = 0.f;
  t.angular.y = 0.f;
  t.angular.z = 0.f;

  
  std::cout<<"\nPress Enter to publish the twist message\n";
  std::cin.get();

  pub_twist.publish(t);

  std::cout<<"\nExiting Normally\n";
  return 0;
}
