#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
#include "ramp_msgs/Population.h"
#include "ramp_msgs/BezierCurve.h"

Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_trajectory_command");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("population", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectoryRequest>("trajectory_generator");
  ros::ServiceClient client_eval = handle.serviceClient<ramp_msgs::EvaluationRequest>("trajectory_evaluation");


  ramp_msgs::KnotPoint zero;
  zero.motionState.positions.push_back(0);
  zero.motionState.positions.push_back(2);
  zero.motionState.positions.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.velocities.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);
  zero.motionState.accelerations.push_back(0);


  // Build a Path
  ramp_msgs::KnotPoint c1;
  c1.motionState.positions.push_back(1.5); // 0.70455
  c1.motionState.positions.push_back(2.); // 0.4026
  c1.motionState.positions.push_back(-2.3561); // 0.519146
  
  ramp_msgs::KnotPoint c2;
  c2.motionState.positions.push_back(1.75); // 0.70455
  c2.motionState.positions.push_back(0.); // 0.4026
  c2.motionState.positions.push_back(-PI/2.f); // 0.519146


  ramp_msgs::KnotPoint c3;
  c3.motionState.positions.push_back(3.25); // 0.857146
  c3.motionState.positions.push_back(0); // 0.71115
  c3.motionState.positions.push_back(3.f*PI/4.f);  // 1.11151


  ramp_msgs::KnotPoint c4;
  c4.motionState.positions.push_back(1.f);
  c4.motionState.positions.push_back(1.f);
  c4.motionState.positions.push_back(PI/4);


  ramp_msgs::KnotPoint c5;
  c5.motionState.positions.push_back(3.5);
  c5.motionState.positions.push_back(0.);
  c5.motionState.positions.push_back(0.);

  ramp_msgs::KnotPoint c6;
  c6.motionState.positions.push_back(1.86675);
  c6.motionState.positions.push_back(0.248112);
  c6.motionState.positions.push_back(2.69603);

  ramp_msgs::KnotPoint c7;
  c7.motionState.positions.push_back(0.854165);
  c7.motionState.positions.push_back(2.97861);
  c7.motionState.positions.push_back(0.288039);

  ramp_msgs::KnotPoint c8;
  c8.motionState.positions.push_back(0.219097);
  c8.motionState.positions.push_back(0.934402);
  c8.motionState.positions.push_back(1.25361);

  ramp_msgs::KnotPoint c9;
  c9.motionState.positions.push_back(3.5);
  c9.motionState.positions.push_back(3.5);
  c9.motionState.positions.push_back(PI/4);
  
  // Velocities
  c1.motionState.velocities.push_back(-0.23);  //.151426
  c1.motionState.velocities.push_back(-0.23); //-.297903
  c1.motionState.velocities.push_back(0.22); //-.118126
 
  c2.motionState.velocities.push_back(-0.2);  //.151426
  c2.motionState.velocities.push_back(-0.15); //-.297903
  c2.motionState.velocities.push_back(-0.13); //-.118126

  c3.motionState.velocities.push_back(0.);
  c3.motionState.velocities.push_back(0.);
  c3.motionState.velocities.push_back(0.);

  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0);
  c4.motionState.velocities.push_back(0);
 

  c5.motionState.velocities.push_back(0);
  c5.motionState.velocities.push_back(0);
  c5.motionState.velocities.push_back(0);

  c6.motionState.velocities.push_back(0);
  c6.motionState.velocities.push_back(0);
  c6.motionState.velocities.push_back(0);

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
  
  c5.motionState.accelerations.push_back(0.); //.0114877
  c5.motionState.accelerations.push_back(0.);  //-.10465
  c5.motionState.accelerations.push_back(0.); //.0746295
  
  ramp_msgs::Path p;
  //p.points.push_back(zero);
  p.points.push_back(c1);
  p.points.push_back(c2);
  p.points.push_back(c3);
  //p.points.push_back(c4);
  //p.points.push_back(c5);
  //p.points.push_back(c6);
  //p.points.push_back(c7);
  //p.points.push_back(c8);
  //p.points.push_back(c9);
  

  /***************************************************/
  /**************** Create Curves ********************/
  /***************************************************/
 
  // Make BezierCurve from Path
  ramp_msgs::BezierCurve bi;
  
  ramp_msgs::MotionState sp0;
  sp0 = p.points.at(0).motionState;
  //sp0 = zero.motionState;
  
  /*sp0.positions.push_back(0.);
  sp0.positions.push_back(1.5);
  sp0.positions.push_back(PI/4);

  sp0.velocities.push_back(0.);
  sp0.velocities.push_back(0.);
  sp0.velocities.push_back(0.);

  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);
  sp0.accelerations.push_back(0);*/

  
  ramp_msgs::MotionState sp1;
  sp1 = p.points.at(1).motionState;
  //sp1 = p.points.at(0).motionState;
  //sp1 = c2.motionState;

  /*sp1.positions.push_back(1.39389);
  sp1.positions.push_back(1.39292);
  sp1.positions.push_back(0.39449);

  sp1.velocities.push_back(0.);
  sp1.velocities.push_back(0.);
  sp1.velocities.push_back(0);

  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);
  sp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState sp2;
  sp2 = p.points.at(2).motionState;
  //sp2 = p.points.at(1).motionState;
  //sp2 = c3.motionState;

  /*sp2.positions.push_back(0.857146);
  sp2.positions.push_back(0.71115);
  sp2.positions.push_back(1.11151);

  sp2.velocities.push_back(0.163205);
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
  cp0.positions.push_back(1.14459);
  cp0.positions.push_back(2.79138);
  cp0.positions.push_back(1.18166);

  cp0.velocities.push_back(0.144063);
  cp0.velocities.push_back(0.444224);
  cp0.velocities.push_back(0);
  
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  cp0.accelerations.push_back(0);
  
  ramp_msgs::MotionState cp1;
  cp1 = p.points.at(1).motionState;
  /*cp1.positions.push_back(1.);
  cp1.positions.push_back(0.5);
  cp1.positions.push_back(-PI/4);

  cp1.velocities.push_back(0.);
  cp1.velocities.push_back(0.);
  cp1.velocities.push_back(0);

  cp1.accelerations.push_back(0);
  cp1.accelerations.push_back(0);
  cp1.accelerations.push_back(0);*/

  ramp_msgs::MotionState cp2;
  cp2.positions.push_back(1.60729);
  cp2.positions.push_back(3.16445);
  cp2.positions.push_back(0.17546);
  
  // *** Push on the Control points ***
  /*bi.controlPoints.push_back(cp0);
  bi.controlPoints.push_back(cp1);
  bi.controlPoints.push_back(cp2);*/


  /** Why 0 initial velocity? **/
  ramp_msgs::MotionState ms_initVA;
  ms_initVA.velocities.push_back(0.);
  ms_initVA.velocities.push_back(0.);
  ms_initVA.velocities.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);
  ms_initVA.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA;
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.velocities.push_back(0.33);
  ms_maxVA.accelerations.push_back(1);
  ms_maxVA.accelerations.push_back(1);



  ramp_msgs::MotionState ms_begin = p.points.at(0).motionState;
  /*ms_begin.positions.push_back(0.391254);
  ms_begin.positions.push_back(0.227599);
  ms_begin.positions.push_back(0.526867);

  ms_begin.velocities.push_back(0.328534);
  ms_begin.velocities.push_back(0.191114);
  ms_begin.velocities.push_back(0.);

  ms_begin.accelerations.push_back(0);
  ms_begin.accelerations.push_back(0.);
  ms_begin.accelerations.push_back(0.);*/

  //bi.ms_begin = ms_begin;
  //bi.ms_initialVA = ms_initVA;
  //bi.ms_maxVA = ms_maxVA;
 

  // u
  bi.u_0 = 0.;
  /*bi.u_target = 0.990762;
  bi.u_dot_0 = 0.707688;
  bi.u_dot_max = 0.; 
  bi.l = 0.15;*/



  /**************************************************/
  /**************** Curve 0 Done ********************/
  /**************************************************/

  // Curve 1
  ramp_msgs::BezierCurve bi2;


  // Segment points
  ramp_msgs::MotionState sp2_0;
  sp2_0.positions.push_back(0.70455);
  sp2_0.positions.push_back(0.4026);
  sp2_0.positions.push_back(0.519146);

  sp2_0.velocities.push_back(0.33);
  sp2_0.velocities.push_back(0.188571);
  sp2_0.velocities.push_back(0.);

  sp2_0.accelerations.push_back(0);
  sp2_0.accelerations.push_back(0);
  sp2_0.accelerations.push_back(0);


  ramp_msgs::MotionState sp2_1 = c3.motionState;
  ramp_msgs::MotionState sp2_2 = c4.motionState;

  bi2.segmentPoints.push_back(sp2_0);
  bi2.segmentPoints.push_back(sp2_1);
  bi2.segmentPoints.push_back(sp2_2);


  // Control points
  ramp_msgs::MotionState cp2_0;
  cp2_0.positions.push_back(0.852275);
  cp2_0.positions.push_back(0.7013);
  cp2_0.positions.push_back(1.11151);
  
  cp2_0.velocities.push_back(0.163205);
  cp2_0.velocities.push_back(0.33);
  cp2_0.velocities.push_back(0.);
  
  cp2_0.accelerations.push_back(0);
  cp2_0.accelerations.push_back(0);
  cp2_0.accelerations.push_back(0);
  
  ramp_msgs::MotionState cp2_1 = sp2_1;

  ramp_msgs::MotionState cp2_2;
  cp2_2.positions.push_back(1.3094);
  cp2_2.positions.push_back(1.12376);
  cp2_2.positions.push_back(0.380506);
  
  bi2.controlPoints.push_back(cp2_0);
  bi2.controlPoints.push_back(cp2_1);
  bi2.controlPoints.push_back(cp2_2);

  // u
  bi2.l = 0.5;
  bi2.u_0 = 0.; 
  bi2.u_dot_0 = 0.552394;
  bi2.u_target = 0.959989;


  ramp_msgs::MotionState ms_initVA2;
  ms_initVA2.velocities.push_back(0.163205);
  ms_initVA2.velocities.push_back(0.33);
  ms_initVA2.velocities.push_back(0);
  ms_initVA2.accelerations.push_back(0);
  ms_initVA2.accelerations.push_back(0);
  ms_initVA2.accelerations.push_back(0);

  
  ramp_msgs::MotionState ms_maxVA2;
  ms_maxVA2.velocities.push_back(0.33);
  ms_maxVA2.velocities.push_back(0.33);
  ms_maxVA2.accelerations.push_back(1);
  ms_maxVA2.accelerations.push_back(1);

  bi2.ms_initialVA = ms_initVA2;
  bi2.ms_maxVA = ms_maxVA2;
  
  //bi.ms_begin = p.points.at(0).motionState;


  /**************************************************/
  /**************** Curve 1 Done ********************/
  /**************************************************/





  std::vector<ramp_msgs::BezierCurve> curves;
  curves.push_back(bi);
  //curves.push_back(bi2);
  
  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  tr.request.type = PARTIAL_BEZIER;
  tr.request.print = true;
  tr.request.bezierCurves = curves;
  tr.request.segments = 0;

  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  // Get and publish trajectory
  if(client_.call(tr)) 
  {
    std::cout<<"\nSending Trajectory "<<u.toString(tr.response.trajectory)<<"\n";
    pub_traj.publish(tr.response.trajectory);
  }
  else {
    std::cout<<"\nSome error getting trajectory\n";
  }


  /*ramp_msgs::EvaluationRequest er;
  er.request.trajectory = tr.response.trajectory;
  
  if(client_eval.call(er)) {
    std::cout<<"\nEvaluated traj, fitness: "<<er.response.fitness;
    std::cout<<"\nEvaluated traj, feasible: "<<er.response.feasible;
  }


  std::cout<<"\n\nPress Enter to Publish population\n";
  std::cin.get();


  er.request.trajectory.t_start = ros::Duration(2.0);
  
  std::cout<<"\nSetting t_start to "<<2.0;
  if(client_eval.call(er)) {
    std::cout<<"\nEvaluated traj, fitness: "<<er.response.fitness;
    std::cout<<"\nEvaluated traj, feasible: "<<er.response.feasible;
  }*/


  // Create Population to send to trajectory_visualization
  ramp_msgs::Population pop;
  pop.population.push_back(tr.response.trajectory);
  
  pub_pop.publish(pop);
  



  std::cout<<"\nExiting Normally\n";
  return 0;
}
