#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectoryRequest.h"

Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_trajectory_command");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::Trajectory>("bestTrajec", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectoryRequest>("trajectory_generator");

  
  // Build knotpoints for a path
  ramp_msgs::KnotPoint c1;
  c1.motionState.positions.push_back(0.513698);
  c1.motionState.positions.push_back(3.12844);
  c1.motionState.positions.push_back(2.65464);
  
  ramp_msgs::KnotPoint c2;
  c2.motionState.positions.push_back(0);
  c2.motionState.positions.push_back(3.5);
  c2.motionState.positions.push_back(PI);


  ramp_msgs::KnotPoint c3;
  c3.motionState.positions.push_back(0.5f);
  c3.motionState.positions.push_back(2.f);
  c3.motionState.positions.push_back(PI/18);



  ramp_msgs::KnotPoint c4;
  c4.motionState.positions.push_back(2.5f);
  c4.motionState.positions.push_back(2.f);
  c4.motionState.positions.push_back(0);

  c1.motionState.velocities.push_back(-0);
  c1.motionState.velocities.push_back(0);
  c1.motionState.velocities.push_back(-0.785398);

  c1.motionState.accelerations.push_back(0);
  c1.motionState.accelerations.push_back(0);
  c1.motionState.accelerations.push_back(-1.96696);

  c2.motionState.velocities.push_back(0);
  c2.motionState.velocities.push_back(0);
  c2.motionState.velocities.push_back(0);
 
  c3.motionState.velocities.push_back(0);
  c3.motionState.velocities.push_back(0);
  c3.motionState.velocities.push_back(0); 
  

  // Build a Path
  ramp_msgs::Path p;
  p.points.push_back(c1);
  p.points.push_back(c2);
  //p.points.push_back(c3);
  //p.points.push_back(c4);

  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;


  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  if(client_.call(tr)) {
    // Publish trajectory
    pub_traj.publish(tr.response.trajectory);
  }

  std::cout<<"\nSending Trajectory "<<u.toString(tr.response.trajectory);


  std::cout<<"\nExiting Normally\n";
  return 0;
}
