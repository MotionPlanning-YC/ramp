#include <iostream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "corobot.h"

Corobot robot;


void twistCallback(const geometry_msgs::Twist msg) {
  std::cout<<"\nGot twist message!\n";

  // If we got a twist msg, move that speed for 1 second
  ros::Duration d(1.0f / msg.linear.x); 
  ros::Rate r(50);

  ros::Time end = ros::Time::now() + d;
  while(ros::Time::now() < end) {
    robot.sendTwist(msg);
    r.sleep();
  }
}


void trajCallback(const ramp_msgs::Trajectory& msg) {
  std::cout<<"\nGot traj message!\n";

  // If we got a trajectory, set the robot's trajectory 
  robot.updateTrajectory(msg);
  
  // Make the robot move along the trajectory
  robot.moveOnTrajectory();
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "waitForCommand");
  ros::NodeHandle handle;

  // Subscribers
  ros::Subscriber sub_twist = handle.subscribe("twist_single", 1000, twistCallback);
  ros::Subscriber sub_traj = handle.subscribe("bestTrajec", 1000, trajCallback);
  robot.sub_odometry_ = handle.subscribe(Corobot::TOPIC_STR_ODOMETRY, 1000, &Corobot::updateState, &robot);
  
  // Publishers
  robot.pub_twist_ = handle.advertise<geometry_msgs::Twist>(Corobot::TOPIC_STR_TWIST, 1000);
  
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
