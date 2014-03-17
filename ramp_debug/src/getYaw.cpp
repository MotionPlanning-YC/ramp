#include <iostream>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

float value;

void odometryCallback(const nav_msgs::Odometry& msg) {
  std::cout<<"\nYaw: "<<tf::getYaw(msg.pose.pose.orientation) * 180 / 3.14159f;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "getYaw");
  ros::NodeHandle handle;

  ros::Subscriber sub = handle.subscribe("odometry", 1000, odometryCallback);

  ros::spin();
}
