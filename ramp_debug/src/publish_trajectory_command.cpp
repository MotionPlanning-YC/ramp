#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/Population.h"
#include "ramp_msgs/BezierInfo.h"

Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_trajectory_command");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("population", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectoryRequest>("trajectory_generator");



  ramp_msgs::KnotPoint zero;
  zero.motionState.positions.push_back(0);
  zero.motionState.positions.push_back(0);
  zero.motionState.positions.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);


  // Build a Path
  ramp_msgs::KnotPoint c1;
  c1.motionState.positions.push_back(0.798632); //0.798632
  c1.motionState.positions.push_back(2.20926); //2.20926
  c1.motionState.positions.push_back(-0.68681); //-0.68682
  
  ramp_msgs::KnotPoint c2;
  c2.motionState.positions.push_back(0.5); // 0.5
  c2.motionState.positions.push_back(3.); // 2
  c2.motionState.positions.push_back(PI/4); //pi/4


  ramp_msgs::KnotPoint c3;
  c3.motionState.positions.push_back(2.); //0.95
  c3.motionState.positions.push_back(0.); //1.09
  c3.motionState.positions.push_back(-PI/4);  //-1.09


  ramp_msgs::KnotPoint c4;
  c4.motionState.positions.push_back(3.5);
  c4.motionState.positions.push_back(2.);
  c4.motionState.positions.push_back(PI/4);

  ramp_msgs::KnotPoint c5;
  c5.motionState.positions.push_back(4.);
  c5.motionState.positions.push_back(0.);
  c5.motionState.positions.push_back(0);
  

  // Velocities
  c1.motionState.velocities.push_back(0.196879);  //.196879
  c1.motionState.velocities.push_back(-0.16958); //-.16958
  c1.motionState.velocities.push_back(-0.526643); //-.526643
 
  c2.motionState.velocities.push_back(0.);
  c2.motionState.velocities.push_back(0.);
  c2.motionState.velocities.push_back(0.);

  c3.motionState.velocities.push_back(0.);
  c3.motionState.velocities.push_back(0.);
  c3.motionState.velocities.push_back(0.); 
 
  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0); 
 
  c5.motionState.velocities.push_back(0);
  c5.motionState.velocities.push_back(0);
  c5.motionState.velocities.push_back(0); 


  // Accelerations
  c1.motionState.accelerations.push_back(0.0279278); //.0279278
  c1.motionState.accelerations.push_back(-0.18573);  //-.18573
  c1.motionState.accelerations.push_back(0.616528); //.616528

  c2.motionState.accelerations.push_back(0.);
  c2.motionState.accelerations.push_back(0.);
  c2.motionState.accelerations.push_back(0.);
  
  c3.motionState.accelerations.push_back(0.);
  c3.motionState.accelerations.push_back(0.);
  c3.motionState.accelerations.push_back(0.);
  
  c4.motionState.accelerations.push_back(0.);
  c4.motionState.accelerations.push_back(0.);
  c4.motionState.accelerations.push_back(0.);
  
  ramp_msgs::Path p;
  p.points.push_back(zero);
  p.points.push_back(c2);
  p.points.push_back(c3);
  //p.points.push_back(c4);
  //p.points.push_back(c5);
  

  ramp_msgs::MotionState cp0;
  cp0.positions.push_back(0.25);
  cp0.positions.push_back(1.5);
  cp0.positions.push_back(1.40565);
  cp0.velocities.push_back(0.055);
  cp0.velocities.push_back(0.33);
  cp0.velocities.push_back(0);
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);

  ramp_msgs::MotionState cp2;
  cp2.positions.push_back(1.18915);
  cp2.positions.push_back(1.62161);
  cp2.positions.push_back(-1.10761);
  
  
  ramp_msgs::MotionState sp0;
  sp0.positions.push_back(0.);
  sp0.positions.push_back(0.);
  sp0.positions.push_back(0.);
  sp0.velocities.push_back(0);
  sp0.velocities.push_back(0.);
  sp0.velocities.push_back(0);
  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);
  
 
  // Make BezierInfo from Path
  ramp_msgs::BezierInfo bi;

  // Segment points
  //bi.segmentPoints.push_back(p.points.at(0).motionState);
  bi.segmentPoints.push_back(sp0);
  bi.segmentPoints.push_back(p.points.at(1).motionState);
  bi.segmentPoints.push_back(p.points.at(2).motionState);
  
  // Control points
  bi.controlPoints.push_back(cp0);
  bi.controlPoints.push_back(c2.motionState);
  bi.controlPoints.push_back(cp2);

  // u
  //bi.u_0 = 0.807208; 
  //bi.u_dot_0 = 0.11;
  
  ramp_msgs::MotionState ms_initVA;
  ms_initVA.velocities.push_back(0.055);
  ms_initVA.velocities.push_back(0.33);
  ms_initVA.velocities.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA;
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.velocities.push_back(PI/3);
  ms_maxVA.accelerations.push_back(0.66);
  ms_maxVA.accelerations.push_back(0.66);
  ms_maxVA.accelerations.push_back(PI/3);

  bi.ms_initialVA = ms_initVA;
  bi.ms_maxVA = ms_maxVA;
  
  //bi.ms_begin = p.points.at(0).motionState;
  bi.lambda = 0.5;

  
  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  tr.request.type = PARTIAL_BEZIER;
  tr.request.startBezier = false;
  tr.request.print = true;
  tr.request.bezierInfo = bi;


  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  // Get and publish trajectory
  if(client_.call(tr)) {
    std::cout<<"\nSending Trajectory "<<u.toString(tr.response.trajectory);
    pub_traj.publish(tr.response.trajectory);
  }
  else {
    std::cout<<"\nSome error getting trajectory\n";
  }

  std::cout<<"\n\nPress Enter to Publish population\n";
  std::cin.get();

  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop;
  pop.population.push_back(tr.response.trajectory);
  
  pub_pop.publish(pop);


  std::cout<<"\nDifference: "<<u.findAngleFromAToB(c1.motionState.positions, c2.motionState.positions);
  std::cout<<"\nDifference: "<<u.findAngleFromAToB(c2.motionState.positions, c3.motionState.positions);


  std::cout<<"\nExiting Normally\n";
  return 0;
}
