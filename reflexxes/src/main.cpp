#include <stdio.h>
#include <stdlib.h>
#include "Reflexxes.h"
#include "ros/ros.h"

// Main function
int main(int argc, char** argv)
{
  // Initialises the ROS node 
  ros::init(argc, argv, "reflexxes");

  // Variable Declaration
  Reflexxes reflexxes;
  ros::NodeHandle n;
  geometry_msgs::Twist twist;

  // Subscribes to the odometry that will give us the current position and velocity.
  // The callback is in the reflexxes object
  ros::Subscriber odom_sub = n.subscribe("odometry", 100, &Reflexxes::odometryCallback, &reflexxes);

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", &Reflexxes::trajectoryRequest, &reflexxes);

  ros::spin();

  return 0; 
}
