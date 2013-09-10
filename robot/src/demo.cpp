#include <ros/ros.h>
#include "ramp_msgs/Trajectory.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"

int main(int argc, char** argv) {
   
  ros::init(argc, argv, "robotDemo");
  ros::NodeHandle handle;
  ros::Publisher traj_pub = handle.advertise<ramp_msgs::Trajectory>("bestTrajec", 100);

  ramp_msgs::Trajectory msg;

  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(0);
  point.positions.push_back(0);
  point.positions.push_back(0);
  point.time_from_start = ros::Duration(0);

  msg.trajectory.points.push_back(point);

  point.positions.at(0) = 1.067; // + 42inches
  point.positions.at(1) = 0.0;
  point.time_from_start = ros::Duration(5);
  msg.trajectory.points.push_back(point);

  point.positions.at(0) = 1.067;
  point.positions.at(1) = 0.89; // + 35 in
  point.time_from_start = ros::Duration(10);
  msg.trajectory.points.push_back(point);
  
  point.positions.at(0) = 1.067 + 6.558; // + 270in - 0.3m
  point.positions.at(1) = 0.89;
  point.time_from_start = ros::Duration(35);
  msg.trajectory.points.push_back(point);
  
 /* point.positions.at(0) = 1.067 + 6.858;
  point.positions.at(1) = -1.41; // + 91 in
  point.time_from_start = ros::Duration(45);
  msg.trajectory.points.push_back(point);
  /*
  point.positions.at(0) = 1.143; // - 263in
  point.positions.at(1) = -1.41;
  point.time_from_start = ros::Duration(60);
  msg.trajectory.points.push_back(point);
  /*
  point.positions.at(0) = 1.143;
  point.positions.at(1) =0; // - 56in
  point.time_from_start = ros::Duration(65);
  msg.trajectory.points.push_back(point);
/*
  point.positions.at(0) = 0; // - 45in
  point.positions.at(1) = 0;
  point.positions.at(1) = 0;
  point.time_from_start = ros::Duration(70);
  msg.trajectory.points.push_back(point);


*/
  msg.index_knot_points.push_back(0);
  msg.index_knot_points.push_back(1);
  msg.index_knot_points.push_back(2);
  msg.index_knot_points.push_back(3);
 /* msg.index_knot_points.push_back(4);
 /* msg.index_knot_points.push_back(5);
 /* msg.index_knot_points.push_back(6);
 /* msg.index_knot_points.push_back(7);
  */
  
  ros::spinOnce();
  printf("Press any key to publish trajectory\n");
  char temp;
  scanf("%c",&temp);
  printf("Publishing\n");
 traj_pub.publish(msg);
  
  
  ros::spin();

  return 0;
}
