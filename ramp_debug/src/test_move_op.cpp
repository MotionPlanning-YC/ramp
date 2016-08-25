#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/RampTrajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectorySrv.h"
#include "ramp_msgs/EvaluationSrv.h"
#include "ramp_msgs/Population.h"
#include "ramp_msgs/BezierCurve.h"
#include "ramp_msgs/ModificationRequest.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_move_op");
  ros::NodeHandle handle;

  // Create publishers
  ros::Publisher pub_traj = handle.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  ros::Publisher pub_pop = handle.advertise<ramp_msgs::Population>("population", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectorySrv>("trajectory_generator");
  ros::ServiceClient client_path = handle.serviceClient<ramp_msgs::ModificationRequest>("path_modification");
  ros::ServiceClient client_eval = handle.serviceClient<ramp_msgs::EvaluationSrv>("trajectory_evaluation");

  ramp_msgs::Path p1;
  for(unsigned int i=0;i<4;i++) 
  {
    ramp_msgs::KnotPoint kp;
    kp.motionState.positions.push_back(i+1);
    kp.motionState.positions.push_back(i+1);
    kp.motionState.positions.push_back(i+2);

    p1.points.push_back(kp);
  }
  trajectory_msgs::JointTrajectoryPoint jp;
  jp.positions = p1.points[0].motionState.positions;

  ramp_msgs::RampTrajectory traj;
  traj.trajectory.points.push_back(jp);
  traj.i_knotPoints.push_back(0);
  traj.holonomic_path = p1;

  ROS_INFO("Press Enter to publish path");
  std::cin.get();
  
  ramp_msgs::Population pop;
  pop.population.push_back(traj);
  pub_pop.publish(pop);


  // Do the modification
  ramp_msgs::ModificationRequest mod;
  mod.request.paths.push_back(p1);
  mod.request.op = "move";
  mod.request.move_dir = PI/4.;
  mod.request.move_dist = 1.5;

  if(client_path.call(mod))
  {
    ROS_INFO("Path successfully modified");
    ROS_INFO("Press Enter to publish new path");
    std::cin.get();
  
    pop.population[0].holonomic_path = mod.response.mod_paths[0];
    pub_pop.publish(pop);
  }
  else
  {
    ROS_ERROR("Error getting modified path");
  }
  

  printf("\nExiting Normally\n");
  return 0;
}
