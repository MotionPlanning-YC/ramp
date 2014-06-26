#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "mobile_base.h"
#include "ros/ros.h"

#include "bezier_curve.h"
#include "ramp_msgs/Population.h"

Utility u;


// Main function
int main(int argc, char** argv) {

  // Initialises the ROS node 
  ros::init(argc, argv, "reflexxes");

  // Variable Declaration
  MobileBase mobileBase;
  ros::NodeHandle n;

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", &MobileBase::trajectoryRequest, &mobileBase);


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

  ramp_msgs::Trajectory trj;

  /** Turn path inth Bezier */
  std::cout<<"\nPath before Bezier: "<<u.toString(p)<<"\n";
  ramp_msgs::Path p_bezier = mobileBase.Bezier(p);
  std::cout<<"\nPath after Bezier: "<<u.toString(p_bezier)<<"\n";

  /** Make a trajectory from the path */
  for(uint8_t i=0;i<p_bezier.points.size();i++) {
    trj.trajectory.points.push_back(u.getTrajectoryPoint(p_bezier.points.at(i).motionState));
  }

  /** Create Bezier curve 
  BezierCurve bc;
  std::vector<ramp_msgs::MotionState> sp;
  sp.push_back(kp0.motionState);
  sp.push_back(kp1.motionState);
  sp.push_back(kp2.motionState);
  bc.init(sp, 0.5);
  
  std::vector<ramp_msgs::MotionState> p_bc = bc.generateCurve();
  for(uint8_t i=0;i<p_bc.size();i++) {
    trj.trajectory.points.push_back(u.getTrajectoryPoint(p_bc.at(i)));
  }*/

  // Make a Population
  ramp_msgs::Population pop;
  pop.population.push_back(trj);
 

  /** Get a trajectory 
  
  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  
  mobileBase.trajectoryRequest(tr.request, tr.response);
  
  std::cout<<"\nTrajectory: "<<u.toString(tr.response.trajectory);*/

  /** Publish the Population */
  ros::Publisher pub = n.advertise<ramp_msgs::Population>("population", 1000);
  
  std::cout<<"\nPress Enter to publish the population\n";
  std::cin.get();
  pub.publish(pop);


  std::cout<<"\nPublished Population";
/*********************************************************************/


  std::cout<<"\nWaiting for requests...\n";
  ros::spin();

  return 0; 
}
