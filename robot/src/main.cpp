#include "ros/ros.h"
#include "corobot.h"

Corobot robot;

void trajCallback(const ramp_msgs::TrajectoryWithKnots::ConstPtr& msg) {
  std::cout<<"\nGot the message!";
  robot.trajectory_ = *msg;
  //robot.updateTrajectory();
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

  ros::spin();

  std::cout<<"\nExiting Normally\n";
  return 0;
}
