#include "ros/ros.h"
#include "utility.h"
#include "ramp_msgs/Trajectory.h"
#include "corobot.h"

#define PI 3.14159f

Corobot robot;

void bestTrajecCallback(const ramp_msgs::Trajectory& msg) {
  robot.updateTrajectory(msg);
}


int main(int argc, char** argv) {
    
  ros::init(argc, argv, "simulation");
  ros::NodeHandle handle;

  ros::Subscriber sub_trajec = handle.subscribe("bestTrajec", 1000, &bestTrajecCallback);

  robot.pub_cmd_vel_ = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  robot.pub_twist_ = handle.advertise<geometry_msgs::Twist>(Corobot::TOPIC_STR_TWIST, 1000);
  robot.pub_update_ = handle.advertise<ramp_msgs::Update>(Corobot::TOPIC_STR_UPDATE, 1000);
  robot.timer_ = handle.createTimer(ros::Duration(0.1), &Corobot::updatePublishTimer, &robot);
  robot.sub_odometry_ = handle.subscribe("odom", 1000, &Corobot::updateState, &robot);



  // Make a blank ramp_msgs::Trajectory
  ramp_msgs::Trajectory init;
  robot.trajectory_ = init;

  while(ros::ok()) {
    robot.moveOnTrajectory(true);
    ros::spinOnce();
  }
    
  std::cout<<"\nExiting normally\n";
  return 0;
}
