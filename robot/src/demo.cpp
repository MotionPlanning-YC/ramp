#include <ros/ros.h>
#include "ramp_msgs/Trajectory.h"

int main(int argc, char** argv) {
   
  ros::init(argc, argv, "robot");
  ros::NodeHandle handle;
  ros::Subscriber sub_traj = handle.advertise<ramp_msgs::Trajectory>("bestTrajec", 1000);

  ramp_msgs::Trajectory msg;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(0);
  point.positions.push_back(0);
  point.positions.push_back(0);
  point.time_from_start = ros::Duration(0);

  msg.trajectory.points.push_back(point);

  point.positions.at(0) = 0.5;
  point.positions.at(1) = 0.5;
  point.time_from_start = ros::Duration(4);
  msg.trajectory.points.push_back(point);

  point.positions.at(0) = 1.0;
  point.positions.at(1) = 0;
  point.time_from_start = ros::Duration(6);
  msg.trajectory.points.push_back(point);
  
  point.positions.at(0) = 1.5;
  point.positions.at(1) = 0.5;
  point.positions.at(1) = 0;
  point.time_from_start = ros::Duration(9);
  msg.trajectory.points.push_back(point);

  msg.index_knot_points.push_back(0);
  msg.index_knot_points.push_back(1);
  msg.index_knot_points.push_back(2);
  msg.index_knot_points.push_back(3);
  robot.moveOnTrajectory();
  
  
  ros::spin();

  return 0;
}
