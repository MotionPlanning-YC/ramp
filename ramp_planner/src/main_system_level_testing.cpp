#include "ros/ros.h"
#include "utility.h"
#include "motion_state.h"
#include "range.h"
#include "ramp_msgs/TrajectorySrv.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_world_state");
  ros::NodeHandle handle;



  // Create a trajectory request client
  ros::ServiceClient traj_client = handle.serviceClient<ramp_msgs::TrajectorySrv>("/trajectory_generator");  

  // Create a vector of trajectory requests
  std::vector<ramp_msgs::TrajectoryRequest> ob_trj_reqs;

  // Create a single trajectory request
  ramp_msgs::TrajectoryRequest ob_tr1;

  // Create ranges for positions and velocities
  Range xy_pos(0, 3.5);
  Range theta_pos(-PI, PI);
  Range linear_vels(-0.33f, 0.33f);
  Range angular_vels(-PI/4.f, PI/4.f); 



  // Get random position
  MotionState ob_motion_state;
  ob_motion_state.msg_.positions.push_back(xy_pos.random());

  // Generate random velocities
  double v = 0.25;
  double w = 0.455;

  // No acceleration because we assume constant velocity
  


  

  return 0;
}
