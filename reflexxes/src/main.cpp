#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "reflexxes.h"
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


  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", &Reflexxes::trajectoryRequest, &reflexxes);

  std::cout<<"\nWaiting for requests...\n";
  ros::spin();

  return 0; 
}
