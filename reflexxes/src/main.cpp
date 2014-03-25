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
  ros::Subscriber odom_sub = n.subscribe("odometry",100, &Reflexxes::updateStatus, &reflexxes);

  // Publishes the twist message, that will command the robot's movement
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("motor_command",100);


  // Starting the control loop
  while(ros::ok())
  { 
    // We only want to calculate the new twist command if the robot hasn't reached the destination yet
    if (!reflexxes.isFinalStateReached()) 
    {   
      twist = reflexxes.spinOnce();
      twist_pub.publish(twist);
    }
    ros::spinOnce();
  }

  return 0; 
}
