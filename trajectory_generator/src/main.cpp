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

Utility utility;


void fixDuplicates(ramp_msgs::TrajectoryRequest::Request& req)
{
  int i=0;
  while(i<req.path.points.size()-1)
  {
    ramp_msgs::MotionState a = req.path.points.at(i).motionState;
    ramp_msgs::MotionState b = req.path.points.at(i+1).motionState;

    if(utility.positionDistance(a.positions, b.positions) < 0.1)
    {
      ROS_WARN("Consecutive duplicate knot points in path:\nPath[%i]:\n%s\nand\nPath[%i]\n%s\nRemoving knot point at index %i", 
          i+1,
          utility.toString(a).c_str(),
          i+1,
          utility.toString(b).c_str(),
          i);
      req.path.points.erase(req.path.points.begin()+i+1);
      i--;
    }

    i++;
  }
}



bool requestCallback( ramp_msgs::TrajectoryRequest::Request& req,
                      ramp_msgs::TrajectoryRequest::Response& res) 
{
  ROS_INFO("Request Received: %s", utility.toString(req).c_str());

  if(req.type != PREDICT) 
  {
    fixDuplicates(req);
    
    MobileBase mobileBase;
    if(!mobileBase.trajectoryRequest(req, res))
    {
      res.error = true;
    }
  }
  else if(req.path.points.size() > 0) 
  {
    Prediction prediction;
    prediction.trajectoryRequest(req, res);
  }

  ROS_INFO("Trajectory Done");
  //ROS_INFO("Sending back: %s", utility.toString(res.trajectory).c_str());
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
