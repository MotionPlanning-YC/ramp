#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
#include "ramp_msgs/Population.h"
#include "ramp_msgs/BezierInfo.h"

Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_trajectory_command");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("/robot_0/population", 1000);
  ros::ServiceClient clientTrajGen  = handle.serviceClient<ramp_msgs::TrajectoryRequest>("trajectory_generator");
  ros::ServiceClient clientTrajEval = handle.serviceClient<ramp_msgs::EvaluationRequest>("trajectory_evaluation");



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
  c1.motionState.positions.push_back(3.5); //0.951779
  c1.motionState.positions.push_back(2.); //1.09629
  c1.motionState.positions.push_back(0.); //-1.094888
 
  // Velocities
  c1.motionState.velocities.push_back(0.);  //.151426
  c1.motionState.velocities.push_back(0.); //-.297903
  c1.motionState.velocities.push_back(0.); //-.118126

  // Accelerations
  c1.motionState.accelerations.push_back(0.); //.0114877
  c1.motionState.accelerations.push_back(0.);  //-.10465
  c1.motionState.accelerations.push_back(0.); //.0746295

 
  ramp_msgs::Path p;
  p.points.push_back(zero);
  p.points.push_back(c1);
 
  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  tr.request.type = PARTIAL_BEZIER;
  tr.request.startBezier = false;
  tr.request.print = true;


  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  ramp_msgs::RampTrajectory trajectory;

  // Get and publish trajectory
  if(clientTrajGen.call(tr)) {
    std::cout<<"\nSending Trajectory "<<u.toString(tr.response.trajectory);
    trajectory = tr.response.trajectory;
  }
  else {
    std::cout<<"\nSome error getting trajectory\n";
  }

  ros::Duration d(2);
  d.sleep();

  ros::Duration d_while(0.2);
  std::cout<<"\nContinuously drawing the trajectory\n";
  while(ros::ok()) {
 
    ramp_msgs::EvaluationRequest er;
    er.request.trajectory = trajectory;
 
    // Evaluate the trajectory
    if(clientTrajEval.call(er)) {
      //std::cout<<"\nEvaluating Trajectory "<<u.toString(tr.response.trajectory);
      trajectory.feasible = er.response.feasible;
      trajectory.fitness = er.response.fitness;
      trajectory.t_firstCollision = er.response.t_firstCollision;
      std::cout<<"\nFitness: "<<trajectory.fitness<<" Feasible: "<<(int)trajectory.feasible;
      if(!trajectory.feasible) {
        std::cout<<" t_firstCollision: "<<trajectory.t_firstCollision;
      }
    }
    else {
      std::cout<<"\nSome error getting trajectory\n";
    }
 
    // Publish the trajectory to traj_vis
    ramp_msgs::Population pop;
    pop.population.push_back(trajectory);
    pub_pop.publish(pop);

    d_while.sleep();
  } // end while



  std::cout<<"\nExiting Normally\n";
  return 0;
}
