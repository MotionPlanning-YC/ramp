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

    if(utility.positionDistance(a.positions, b.positions) < 0.01)
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


bool checkGoal(ramp_msgs::TrajectoryRequest::Request req)
{
  ramp_msgs::MotionState a = req.path.points.at(0).motionState;
  ramp_msgs::MotionState b = req.path.points.at(1).motionState;

  if(utility.positionDistance(a.positions, b.positions) < 0.1)
  {
    return true;
  }

  return false;
}


bool requestCallback( ramp_msgs::TrajectoryRequest::Request& req,
                      ramp_msgs::TrajectoryRequest::Response& res) 
{
  ROS_INFO("Request Received: %s", utility.toString(req).c_str());

  /*
   * Check for start == goal
   */
  if(req.path.points.size() == 2 && checkGoal(req))
  {
    res.trajectory.trajectory.points.push_back(utility.getTrajectoryPoint(req.path.points.at(0).motionState));
    res.trajectory.i_knotPoints.push_back(0);
    return true;
  }



  // Why req.segments == 1?
  if(req.type != PREDICTION && req.type != TRANSITION && (req.path.points.size() < 3 || req.segments == 1))
  {
    ROS_WARN("Changing type to HOLONOMIC");
    req.type = HOLONOMIC;
    req.segments++;
  }

  if(req.type != PREDICTION) 
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

  //ROS_INFO("Trajectory Done");
  //ROS_INFO("Response: %s", utility.toString(res).c_str());
 
  return true;
}


 //Main function
int main(int argc, char** argv) {

  // Initialize the ROS node 
  ros::init(argc, argv, "reflexxes");
  ros::NodeHandle n;

  // Variable Declaration
  MobileBase mobileBase;

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", requestCallback);


  ros::AsyncSpinner spinner(8);
  spinner.start();
  ROS_INFO("Spinning ...");
  ros::waitForShutdown();

  return 0; 
}
