#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include "mobile_base.h"
#include "ros/ros.h"

#include "bezier_curve.h"
#include "ramp_msgs/Population.h"

Utility u;
bool bezier;

void loadParameters(const ros::NodeHandle& handle) {
  
  if(handle.hasParam("bezier")) {
    handle.getParam("bezier", bezier);
  }
  else {
    ROS_ERROR("Cannot find trajectory_generator parameter: bezier");
    ROS_ERROR("Defaulting bezier to true");
    bezier = true;
  }
}


// Main function
int main(int argc, char** argv) {

  // Initialises the ROS node 
  ros::init(argc, argv, "reflexxes");

  // Variable Declaration
  MobileBase mobileBase;
  ros::NodeHandle n;

  // Declare the service that gives a path and returns a trajectory
  ros::ServiceServer service = n.advertiseService("trajectory_generator", &MobileBase::trajectoryRequest, &mobileBase);


  // Load parameters
  loadParameters(n);

  // Set Bezier
  mobileBase.bezier_ = bezier;
  std::cout<<"\nbezier: "<<mobileBase.bezier_;


  std::vector<double> a, b;
  a.push_back(1.49);
  a.push_back(0.5099);

  b.push_back(1.5);
  b.push_back(0.5);

  std::cout<<"\nDifference: "<<u.findAngleFromAToB(a,b)<<"\n";

  /*********************************************************************/
  /********************* Testing Bezier ********************************/
  /*********************************************************************/

    
    // Creating MotionStates
    /*ramp_msgs::MotionState p0, p1, p2, p3, p4;
    
    p0.positions.push_back(0);
    p0.positions.push_back(0);
    p0.positions.push_back(PI/4);
    
    p1.positions.push_back(1);
    p1.positions.push_back(1);
    p1.positions.push_back(0);
    //p1.velocities.push_back(0);
    //p1.velocities.push_back(0);
    //p1.velocities.push_back(0);
    
    p2.positions.push_back(2);
    p2.positions.push_back(0);
    p2.positions.push_back(0);
    
    //p2.velocities.push_back(0);
    //p2.velocities.push_back(0);
    //p2.velocities.push_back(0);

    p3.positions.push_back(3);
    p3.positions.push_back(1);
    p3.positions.push_back(0);

    p4.positions.push_back(4);
    p4.positions.push_back(0);
    p4.positions.push_back(0);*/
    

    // Create Path
    /*ramp_msgs::Path p;

    ramp_msgs::KnotPoint kp0;
    kp0.motionState = p0;
    
    ramp_msgs::KnotPoint kp1;
    kp1.motionState = p1;
    
    ramp_msgs::KnotPoint kp2;
    kp2.motionState = p2;

    ramp_msgs::KnotPoint kp3;
    kp3.motionState = p3;

    ramp_msgs::KnotPoint kp4;
    kp4.motionState = p4;
    
    p.points.push_back(kp0);
    p.points.push_back(kp1);
    p.points.push_back(kp2);
    p.points.push_back(kp3);
    p.points.push_back(kp4);*/


    /** Get a trajectory */
    
    /*ramp_msgs::TrajectoryRequest tr;
    tr.request.path = p;
    
    mobileBase.trajectoryRequest(tr.request, tr.response);
    
    std::cout<<"\nTrajectory: "<<u.toString(tr.response.trajectory);*/

    /** Publish the Population */

    /*ros::Publisher pub = n.advertise<ramp_msgs::Population>("population", 1000);
    ros::Publisher pub_trj = n.advertise<ramp_msgs::Trajectory>("bestTrajec", 1000);
    
    // Make a Population
    ramp_msgs::Population pop;
    pop.population.push_back(tr.response.trajectory);
    
    std::cout<<"\nPress Enter to publish the population\n";
    std::cin.get();
    pub.publish(pop);
    pub_trj.publish(tr.response.trajectory);


    std::cout<<"\nPublished Population";*/
  /*********************************************************************/



  std::cout<<"\nWaiting for requests...\n";
  ros::spin();

  return 0; 
}
