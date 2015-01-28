#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "mobile_base.h"
#include "prediction.h"
#include "line.h"
#include "circle.h"
#include "ros/ros.h"

#include "bezier_curve.h"
#include "ramp_msgs/Population.h"

Utility u;



bool requestCallback( ramp_msgs::TrajectoryRequest::Request& req,
                      ramp_msgs::TrajectoryRequest::Response& res) 
{
  ROS_INFO("Request Received: %s", u.toString(req).c_str());

  if(req.type != PREDICT) {
    MobileBase mobileBase;
    mobileBase.trajectoryRequest(req, res);
  }
  else if(req.path.points.size() > 0) {
    Prediction prediction;
    prediction.trajectoryRequest(req, res);
  }

  ROS_INFO("Sending back: %s", u.toString(res.trajectory).c_str());
  return true;
}

// Main function
int main(int argc, char** argv) {

  // Initialize the ROS node 
  ros::init(argc, argv, "reflexxes");
  ros::NodeHandle n;

  // Variable Declaration
  MobileBase mobileBase;

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", requestCallback);


  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();
  //ros::spin();

  return 0; 
}
