#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"

#define PI 3.14159f

float theta;

void odometryCallback(const nav_msgs::Odometry& msg) {
  theta = tf::getYaw(msg.pose.pose.orientation);
  std::cout<<"\nTheta: "<<theta;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_twist_command");
  ros::NodeHandle handle;

  ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>("twist", 1000);
  ros::Subscriber sub_odom = handle.subscribe("odometry", 1000, odometryCallback); 

  geometry_msgs::Twist t;
  t.linear.x = 0.f;
  t.linear.y = 0.f;
  t.linear.z = 0.f;
  t.angular.x = 0.f;
  t.angular.y = 0.f;
  t.angular.z = -PI/3;

  
  std::cout<<"\nPress Enter to publish the twist message\n";
  std::cin.get();

  theta = 0.f;
  ros::Rate r(50);
  while(ros::ok() && theta >= -PI/2) {
    pub_twist.publish(t);
    r.sleep();
    ros::spinOnce();
  }

  std::cout<<"\nExiting Normally\n";
  return 0;
}
