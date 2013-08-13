#include "ros/ros.h"
#include "trajectory.h"
#include "ramp_msgs/TrajectoryRequest.h"


bool handleRequest(ramp_msgs::TrajectoryRequest::Request& req, 
                   ramp_msgs::TrajectoryRequest::Response& res) 
{
  Trajectory traj(req);
  traj.generate(); 
  res.trajectory = traj.buildTrajectoryMsg();
  return true;
}


int main(int argc, char** argv) {
  

  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("trajectory_generator", handleRequest);

  std::cout<<"\nSpinning...\n";  
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
