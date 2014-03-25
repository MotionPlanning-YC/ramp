#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/Path.h"
#include "utility.h"
#include "ramp_msgs/TrajectoryRequest.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "publish_trajectory_command");
  ros::NodeHandle handle;

  ros::Publisher pub_traj = handle.advertise<ramp_msgs::Trajectory>("trajectory", 1000);
  ros::ServiceClient client_ = handle.serviceClient<ramp_msgs::TrajectoryRequest>("trajectory_generator");

  
  // Build a Path
  ramp_msgs::KnotPoint c1;
  c1.configuration.K.push_back(0);
  c1.configuration.K.push_back(0);
  c1.configuration.K.push_back(0);

  ramp_msgs::KnotPoint c2;
  c2.configuration.K.push_back(0.f);
  c2.configuration.K.push_back(0);
  c2.configuration.K.push_back(-PI/1.5);

  /*ramp_msgs::KnotPoint c3;
  c3.configuration.K.push_back(3);
  c3.configuration.K.push_back(2);
  c3.configuration.K.push_back(PI/2);
  c3.stop_time = 2;

  ramp_msgs::KnotPoint c4;
  c4.configuration.K.push_back(3);
  c4.configuration.K.push_back(3);
  c4.configuration.K.push_back(PI/2);*/

  std::vector<float> v_s;
  v_s.push_back(0.35);
  //v_s.push_back(0.25);
  //v_s.push_back(0.25);

  std::vector<float> v_g;
  v_g.push_back(0.35);
  //v_g.push_back(0.25);
  //v_g.push_back(0.25);

  ramp_msgs::Path p;
  p.points.push_back(c1);
  p.points.push_back(c2);
  //p.points.push_back(c3);
  //p.points.push_back(c4);

  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p;
  
  tr.request.v_start = v_s;
  tr.request.v_end = v_g;
  tr.request.resolutionRate = 5;


  std::cout<<"\nPress Enter to request and send the trajectory\n";
  std::cin.get();

  if(client_.call(tr)) {
    // Publish trajectory
    pub_traj.publish(tr.response.trajectory);
  }


  std::cout<<"\nExiting Normally\n";
  return 0;
}
