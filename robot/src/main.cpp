#include "ros/ros.h"
#include "corobot.h"

Corobot robot;

void trajCallback(const ramp_msgs::Trajectory::ConstPtr& msg) {
  std::cout<<"\nGot the message!";
  robot.trajectory_ = *msg;

  //Move robot along trajectory
  robot.moveOnTrajectory(); 
}


/** Initialize the Corobot's publishers and subscibers*/
void init_advertisers_subscribers(Corobot& robot, ros::NodeHandle& handle) {

  //Publishers
  robot.pub_phidget_motor_ = handle.advertise<corobot_msgs::MotorCommand>(Corobot::TOPIC_STR_PHIDGET_MOTOR, 1000);
  robot.pub_twist_ = handle.advertise<geometry_msgs::Twist>(Corobot::TOPIC_STR_TWIST, 1000);
 
  //Subscribers
  //robot.sub_odometry_ = handle.subscribe(Corobot::TOPIC_STR_ODOMETRY, 1000, &Corobot::updateState, &robot);
  robot.sub_odometry_ = handle.subscribe("odometry", 1000, &Corobot::updateState, &robot);
}


int main(int argc, char** argv) {
   
  ros::init(argc, argv, "robot");
  ros::NodeHandle handle;
  ros::Subscriber sub_traj = handle.subscribe("traj", 1000, trajCallback);
  
  init_advertisers_subscribers(robot, handle);
/*
  trajectory_msgs::JointTrajectoryPoint point;
  point.positions.push_back(0);
  point.positions.push_back(0);
  point.positions.push_back(0);
  point.time_from_start = ros::Duration(0);

  robot.trajectory_.trajectory.points.push_back(point);

  point.positions.at(0) = 0.5;
  point.time_from_start = ros::Duration(2);
  robot.trajectory_.trajectory.points.push_back(point);

  point.positions.at(0) = 1.0;
  point.time_from_start = ros::Duration(6);
  robot.trajectory_.trajectory.points.push_back(point);

  robot.moveOnTrajectory();*/
  ros::spin();

  std::cout<<"\nExiting Normally\n";
  return 0;
}
