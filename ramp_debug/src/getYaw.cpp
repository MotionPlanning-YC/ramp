#include <iostream>
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_datatypes.h"

float value;

void odometryCallback(const nav_msgs::Odometry& msg) {
<<<<<<< HEAD
  std::cout<<"\nYaw: "<<tf::getYaw(msg.pose.pose.orientation) * 180 / 3.14159f;
=======
  float orientation_degrees = tf::getYaw(msg.pose.pose.orientation)*180/M_PI;
  std::cout<<"\nYaw: "<<orientation_degrees;
>>>>>>> 7da1e7f4117a1203d05c2baf16e14e02a955d338
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "getYaw");
  ros::NodeHandle handle;

  ros::Subscriber sub = handle.subscribe("odometry", 1000, odometryCallback);

  ros::spin();
}
