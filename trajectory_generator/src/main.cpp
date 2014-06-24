#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "reflexxes.h"
#include "ros/ros.h"

#include "ramp_msgs/Population.h"



// Main function
int main(int argc, char** argv) {

  // Initialises the ROS node 
  ros::init(argc, argv, "reflexxes");

  // Variable Declaration
  Reflexxes reflexxes;
  ros::NodeHandle n;

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", &Reflexxes::trajectoryRequest, &reflexxes);


/*********************************************************************/
/********************* Testing Bezier ********************************/
/*********************************************************************/

  
  // Creating MotionStates
  ramp_msgs::MotionState p0, p1, p2, p3;
  
  p0.positions.push_back(1);
  p0.positions.push_back(1);
  p0.positions.push_back(0);
  
  p1.positions.push_back(1);
  p1.positions.push_back(3);
  
  p2.positions.push_back(2);
  p2.positions.push_back(2);

  p3.positions.push_back(3);
  p3.positions.push_back(3);
  

  // Create Path
  ramp_msgs::Path p;

  ramp_msgs::KnotPoint kp0;
  kp0.motionState = p0;
  
  ramp_msgs::KnotPoint kp1;
  kp1.motionState = p1;
  
  ramp_msgs::KnotPoint kp2;
  kp2.motionState = p2;

  ramp_msgs::KnotPoint kp3;
  kp3.motionState = p3;
  
  p.points.push_back(kp0);
  p.points.push_back(kp1);
  p.points.push_back(kp2);
  //p.points.push_back(kp3);

  
  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  
  reflexxes.trajectoryRequest(tr.request, tr.response);
  

  
  std::cout<<"\nPress Enter to publish the population\n";
  std::cin.get();
  //pub.publish(pop);


  std::cout<<"\nPublished Population";
/*********************************************************************/


  std::cout<<"\nWaiting for requests...\n";
  ros::spin();

  return 0; 
}
