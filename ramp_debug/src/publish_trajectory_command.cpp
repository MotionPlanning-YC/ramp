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
  zero.motionState.positions.push_back(0.);
  zero.motionState.velocities.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);

  // Build a Path
  ramp_msgs::KnotPoint c1;
  
  c1.motionState.positions.push_back(0.37455); // 0.70455
  c1.motionState.positions.push_back(0.214029); // 0.4026
  c1.motionState.positions.push_back(0.); // 0.519013

  ramp_msgs::KnotPoint c2;
  c2.motionState.positions.push_back(0.70455); // 0.70455
  c2.motionState.positions.push_back(0.4026); // 0.4026
  c2.motionState.positions.push_back(0.519146); // 0.519146


  ramp_msgs::KnotPoint c3;
  c3.motionState.positions.push_back(0.857146); // 0.857146
  c3.motionState.positions.push_back(0.71115); // 0.71115
  c3.motionState.positions.push_back(1.11151);  // 1.11151


  ramp_msgs::KnotPoint c4;
  c4.motionState.positions.push_back(3.5);
  c4.motionState.positions.push_back(2.);
  c4.motionState.positions.push_back(PI);

  
  // Velocities
  c1.motionState.velocities.push_back(0.33);  //.151426
  c1.motionState.velocities.push_back(0.188571); //-.297903
  c1.motionState.velocities.push_back(0.); //-.118126
 
  c2.motionState.velocities.push_back(0.33);
  c2.motionState.velocities.push_back(0.188571);
  c2.motionState.velocities.push_back(0.);

  c3.motionState.velocities.push_back(0.163205);
  c3.motionState.velocities.push_back(0.33);
  c3.motionState.velocities.push_back(0.);
 
  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0); 
 

  // Accelerations
  c1.motionState.accelerations.push_back(0.); //.0114877
  c1.motionState.accelerations.push_back(0.);  //-.10465
  c1.motionState.accelerations.push_back(0.); //.0746295

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
  //p.points.push_back(zero);
  p.points.push_back(c1);
  p.points.push_back(c2);
  p.points.push_back(c3);
  //p.points.push_back(c4);
  

  /***************************************************/
  /**************** Create Curves ********************/
  /***************************************************/
 
  // Make BezierInfo from Path
  ramp_msgs::BezierInfo bi;
  
  ramp_msgs::MotionState sp0;
  sp0 = p.points.at(0).motionState;
  
  /*sp0.positions.push_back(0.70455);
  sp0.positions.push_back(0.4026);
  sp0.positions.push_back(0.519146);

  sp0.velocities.push_back(0.33);
  sp0.velocities.push_back(0.188571);
  sp0.velocities.push_back(0.);

  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);*/

  
  ramp_msgs::MotionState sp1;
  sp1 = p.points.at(1).motionState;
  //sp1 = c2.motionState;

  /*sp1.positions.push_back(0.103043);
  sp1.positions.push_back(0.5775);
  sp1.positions.push_back(1.41282);

  sp1.velocities.push_back(0.0525683);
  sp1.velocities.push_back(0.33);
  sp1.velocities.push_back(0);

  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState sp2;
  sp2 = p.points.at(2).motionState;
  //sp2 = c3.motionState;

  /*sp2.positions.push_back(0.25);
  sp2.positions.push_back(1.5);
  sp2.positions.push_back(1.40565);

  sp2.velocities.push_back(0.055);
  sp2.velocities.push_back(0.33);
  sp2.velocities.push_back(0);

  sp2.accelerations.push_back(0);
  sp2.accelerations.push_back(0);
  sp2.accelerations.push_back(0);*/

  // *** Push on the Segment points ***
  bi.segmentPoints.push_back(sp0);
  bi.segmentPoints.push_back(sp1);
  bi.segmentPoints.push_back(sp2);


  // Control Points
  
  //ramp_msgs::MotionState cp0 = sp0;
  ramp_msgs::MotionState cp0;
  cp0.positions.push_back(0.852275);
  cp0.positions.push_back(0.7013);
  cp0.positions.push_back(1.11151);

  cp0.velocities.push_back(0.163205);
  cp0.velocities.push_back(0.33);
  cp0.velocities.push_back(0);
  
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  
  ramp_msgs::MotionState cp1;
  cp1 = p.points.at(1).motionState;
  /*cp1.positions.push_back(0.0252633);
  cp1.positions.push_back(0.11055);
  cp1.positions.push_back(1.34613);

  cp1.velocities.push_back(0.0525683);
  cp1.velocities.push_back(0.33);
  cp1.velocities.push_back(0);

  cp1.accelerations.push_back(0);
  cp1.accelerations.push_back(0);
  cp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState cp2;
  cp2.positions.push_back(1.3094);
  cp2.positions.push_back(1.12376);
  cp2.positions.push_back(0.380506);
  
  // *** Push on the Control points ***
  /*bi.controlPoints.push_back(cp0);
  bi.controlPoints.push_back(cp1);
  bi.controlPoints.push_back(cp2);*/


  /** Why 0 initial velocity? **/
  ramp_msgs::MotionState ms_initVA;
  ms_initVA.velocities.push_back(0.163205);
  ms_initVA.velocities.push_back(0.33);
  ms_initVA.velocities.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA;
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.accelerations.push_back(1.);
  ms_maxVA.accelerations.push_back(1.);



  ramp_msgs::MotionState ms_begin;
  ms_begin.positions.push_back(0.841323);
  ms_begin.positions.push_back(1.31916);
  ms_begin.positions.push_back(-1.08399);

  ms_begin.velocities.push_back(0.166735);
  ms_begin.velocities.push_back(-.297418);
  ms_begin.velocities.push_back(0.231612);

  ms_begin.accelerations.push_back(0.0173546);
  ms_begin.accelerations.push_back(0.325817);
  ms_begin.accelerations.push_back(2.31612);

  //bi.ms_begin = ms_begin;
  bi.ms_initialVA = ms_initVA;
  bi.ms_maxVA = ms_maxVA;
 

  // u
  bi.u_0 = 0.;
  bi.u_dot_0 = 0.;
  bi.l = 0.;

  bi.numOfPoints = 0;


  /**************************************************/
  /**************** Curve 0 Done ********************/
  /**************************************************/

  // Curve 1
  ramp_msgs::BezierInfo bi2;


  // Segment points
  ramp_msgs::MotionState sp2_0;
  sp2_0.positions.push_back(0.076759);
  sp2_0.positions.push_back(0.4125);
  sp2_0.positions.push_back(1.41282);

  sp2_0.velocities.push_back(0.0525683);
  sp2_0.velocities.push_back(0.33);
  sp2_0.velocities.push_back(0.);

  sp2_0.accelerations.push_back(0);
  sp2_0.accelerations.push_back(0);
  sp2_0.accelerations.push_back(0);


  ramp_msgs::MotionState sp2_1 = c2.motionState;
  ramp_msgs::MotionState sp2_2 = c3.motionState;

  bi2.segmentPoints.push_back(sp2_0);
  bi2.segmentPoints.push_back(sp2_1);
  bi2.segmentPoints.push_back(sp2_2);


  // Control points
  ramp_msgs::MotionState cp2_0;
  cp2_0.positions.push_back(0.355992);
  cp2_0.positions.push_back(1.45985);
  cp2_0.positions.push_back(1.31025);
  
  cp2_0.velocities.push_back(0.0879808);
  cp2_0.velocities.push_back(0.33);
  cp2_0.velocities.push_back(0.);
  
  cp2_0.accelerations.push_back(0);
  cp2_0.accelerations.push_back(0);
  cp2_0.accelerations.push_back(0);
  
  ramp_msgs::MotionState cp2_1 = sp2_1;

  ramp_msgs::MotionState cp2_2;
  cp2_2.positions.push_back(0.75);
  cp2_2.positions.push_back(1.5);
  cp2_2.positions.push_back(-1.10715);
  
  bi2.controlPoints.push_back(cp2_0);
  bi2.controlPoints.push_back(cp2_1);
  bi2.controlPoints.push_back(cp2_2);

  // u
  bi2.u_0 = 0.; 
  bi2.u_dot_0 = 0.305471;


  ramp_msgs::MotionState ms_initVA2;
  ms_initVA2.velocities.push_back(0.0879808);
  ms_initVA2.velocities.push_back(0.33);
  ms_initVA2.velocities.push_back(0);
  ms_initVA2.accelerations.push_back(0);
  ms_initVA2.accelerations.push_back(0);
  ms_initVA2.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA2;
  ms_maxVA2.velocities.push_back(0.33);
  ms_maxVA2.velocities.push_back(0.33);
  ms_maxVA2.velocities.push_back(PI/3);
  ms_maxVA2.accelerations.push_back(0.66);
  ms_maxVA2.accelerations.push_back(0.66);
  ms_maxVA2.accelerations.push_back(PI/3);

  bi2.ms_initialVA = ms_initVA2;
  bi2.ms_maxVA = ms_maxVA2;
  
  //bi.ms_begin = p.points.at(0).motionState;
  bi2.l = 0.5;


  /**************************************************/
  /**************** Curve 1 Done ********************/
  /**************************************************/




  // Trajec 2
  // Build a Path
  ramp_msgs::KnotPoint c12;
  
  c12.motionState.positions.push_back(0.70455); // 0.70455
  c12.motionState.positions.push_back(0.4026); // 0.4026
  c12.motionState.positions.push_back(0.519146); // 0.519013

  ramp_msgs::KnotPoint c22;
  c22.motionState.positions.push_back(1); // 0.70455
  c22.motionState.positions.push_back(1); // 0.4026
  c22.motionState.positions.push_back(1.41282); // 0.519146


  ramp_msgs::KnotPoint c32;
  c32.motionState.positions.push_back(3.5); // 0.857146
  c32.motionState.positions.push_back(2); // 0.71115
  c32.motionState.positions.push_back(PI);  // 1.11151


  
  // Velocities
  c12.motionState.velocities.push_back(0.33);  //.151426
  c12.motionState.velocities.push_back(0.188571); //-.297903
  c12.motionState.velocities.push_back(0.); //-.118126
 

  c32.motionState.velocities.push_back(0.);
  c32.motionState.velocities.push_back(0.);
  c32.motionState.velocities.push_back(0.);
 

  // Accelerations
  c12.motionState.accelerations.push_back(0.); //.0114877
  c12.motionState.accelerations.push_back(0.);  //-.10465
  c12.motionState.accelerations.push_back(0.); //.0746295

  c22.motionState.accelerations.push_back(0.);
  c22.motionState.accelerations.push_back(0.);
  c22.motionState.accelerations.push_back(0.);
  
  c32.motionState.accelerations.push_back(0.);
  c32.motionState.accelerations.push_back(0.);
  c32.motionState.accelerations.push_back(0.);
  
  ramp_msgs::Path p3;
  p3.points.push_back(c12);
  p3.points.push_back(c22);
  p3.points.push_back(c32);
  

  /***************************************************/
  /**************** Create Curves ********************/
  /***************************************************/
 
  // Make BezierInfo from Path
  ramp_msgs::BezierInfo bi3;
  
  ramp_msgs::MotionState sp02;
  sp02 = p3.points.at(0).motionState;
  
  /*sp0.positions.push_back(0.70455);
  sp0.positions.push_back(0.4026);
  sp0.positions.push_back(0.519146);

  sp0.velocities.push_back(0.33);
  sp0.velocities.push_back(0.188571);
  sp0.velocities.push_back(0.);

  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);*/

  
  ramp_msgs::MotionState sp12;
  sp12 = p3.points.at(1).motionState;
  //sp1 = c2.motionState;

  /*sp1.positions.push_back(0.103043);
  sp1.positions.push_back(0.5775);
  sp1.positions.push_back(1.41282);

  sp1.velocities.push_back(0.0525683);
  sp1.velocities.push_back(0.33);
  sp1.velocities.push_back(0);

  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState sp22;
  sp22 = p3.points.at(2).motionState;
  //sp2 = c3.motionState;

  /*sp2.positions.push_back(0.25);
  sp2.positions.push_back(1.5);
  sp2.positions.push_back(1.40565);

  sp2.velocities.push_back(0.055);
  sp2.velocities.push_back(0.33);
  sp2.velocities.push_back(0);

  sp2.accelerations.push_back(0);
  sp2.accelerations.push_back(0);
  sp2.accelerations.push_back(0);*/

  // *** Push on the Segment points ***
  bi3.segmentPoints.push_back(sp02);
  bi3.segmentPoints.push_back(sp12);
  bi3.segmentPoints.push_back(sp22);


  // u
  bi3.u_0 = 0.;
  bi3.u_dot_0 = 0.;
  bi3.l = 0.;

  bi3.numOfPoints = 0;
  
  std::vector<ramp_msgs::BezierInfo> curves2;
  curves2.push_back(bi3);
  //curves.push_back(bi2);
  
  ramp_msgs::TrajectoryRequest tr2;
  tr2.request.path = p3;
  tr2.request.type = PARTIAL_BEZIER;
  tr2.request.startBezier = false;
  tr2.request.print = true;
  tr2.request.bezierInfo = curves2;


  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  ramp_msgs::RampTrajectory n, s;

  // Get and publish trajectory
  if(client_.call(tr2)) {
    std::cout<<"\nSending Trajectory "<<u.toString(tr2.response.trajectory);
    n = tr2.response.trajectory;
  }
  else {
    std::cout<<"\nSome error getting trajectory\n";
  }

  

  std::cout<<"\nDone with 1st trajectory\n";

  std::cout<<"\n\nPress Enter to Publish population\n";
  std::cin.get();

  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop2;
  pop2.population.push_back(n);
  
  pub_pop.publish(pop2);





  std::vector<ramp_msgs::BezierInfo> curves;
  curves.push_back(bi);
  //curves.push_back(bi2);
  
  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  tr.request.type = TRANSITION;
  tr.request.startBezier = false;
  tr.request.print = true;
  tr.request.bezierInfo = curves;

  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  // Get and publish trajectory
  if(client_.call(tr)) {
    std::cout<<"\nSending Trajectory "<<u.toString(tr.response.trajectory);
    //pub_traj.publish(tr.response.trajectory);
    s = tr.response.trajectory;
  }
  else {
    std::cout<<"\nSome error getting trajectory\n";
  }



  std::cout<<"\nDone with 2nd\n";
  std::cout<<"\n\nPress Enter to Publish population\n";
  std::cin.get();

  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop3;
  pop3.population.push_back(s);
  
  pub_pop.publish(pop3);



  ramp_msgs::RampTrajectory rt = s;

  // Set the cycle time and latest point's time
  ros::Duration t_cycle   = rt.trajectory.points.at(1).time_from_start - 
                            rt.trajectory.points.at(0).time_from_start;
  ros::Duration t_latest  = rt.trajectory.points.at(
                            rt.trajectory.points.size()-1).time_from_start 
                            + t_cycle;

  // Keep a counter for the knot points
  int c_kp = n.i_knotPoints.size() < 3 ? 1 : 3;
  std::cout<<"\nc_kp: "<<c_kp;
  //std::cout<<"\ntrgt path: "<<n.toString();

  // Start at the bezier curve in n and 
  // push on the rest of the trajectory to result
  for(uint16_t i=n.i_knotPoints.at(c_kp-1); 
      i<n.trajectory.points.size(); 
      i++) 
  {
    trajectory_msgs::JointTrajectoryPoint temp = n.trajectory.points.at(i);

    // Set proper time
    temp.time_from_start = t_latest;
    t_latest += t_cycle;
    
    // Push on new point
    rt.trajectory.points.push_back( temp );
   
    // If knot point, push on the index
    // and push the point onto the trajectory's path
    if( i == n.i_knotPoints.at(c_kp) ) {
      //std::cout<<"\ni: "<<(int)i<<" n.i_knotPoints.at("<<c_kp<<"): "<<n.i_knotPoints.at(c_kp);
      rt.i_knotPoints.push_back(rt.trajectory.points.size()-1);
      c_kp++;
    }
  } // end for
 
  

  //std::cout<<"\nTrajectory with curve path: "<<rt.path_.toString();
  
  // Push on the target trajectory's Bezier curve
  for(uint8_t i_curve=0;i_curve<n.curves.size();i_curve++) {
    rt.curves.push_back(n.curves.at(i_curve));
  }









  std::cout<<"\n\nPress Enter to Publish population\n";
  std::cin.get();

  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop;
  //pop.population.push_back(tr.response.trajectory);
  pop.population.push_back(rt);
  
  pub_pop.publish(pop);
  
  
  std::cout<<"\nFinal trajec: "<<u.toString(rt);
  pub_traj.publish(rt);


  std::cout<<"\nDifference: "<<u.findAngleFromAToB(c1.motionState.positions, c2.motionState.positions);
  std::cout<<"\nDifference: "<<u.findAngleFromAToB(c2.motionState.positions, c3.motionState.positions);


  std::cout<<"\nExiting Normally\n";
  return 0;
}
