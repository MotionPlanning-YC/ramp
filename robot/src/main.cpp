#include "ros/ros.h"
#include "corobot.h"
#include "ramp_msgs/Update.h"

Corobot robot;
ros::Timer updateTimer;

void trajCallback(const ramp_msgs::Trajectory::ConstPtr& msg) {
  std::cout<<"\nGot the message!";
  std::cout<<"\nPress enter to update!\n";
  std::cin.get();

  //Update the robot's trajectory
  robot.updateTrajectory(*msg);
}


/** Initialize the Corobot's publishers and subscibers*/
void init_advertisers_subscribers(Corobot& robot, ros::NodeHandle& handle) {

  
  //Publishers
  robot.pub_phidget_motor_ = handle.advertise<corobot_msgs::MotorCommand>(Corobot::TOPIC_STR_PHIDGET_MOTOR, 1000);
  robot.pub_twist_ = handle.advertise<geometry_msgs::Twist>(Corobot::TOPIC_STR_TWIST, 1000);
  robot.pub_update_ = handle.advertise<ramp_msgs::Update>(Corobot::TOPIC_STR_UPDATE, 1000);
 
  //Subscribers
  robot.sub_odometry_ = handle.subscribe(Corobot::TOPIC_STR_ODOMETRY, 1000, &Corobot::updateState, &robot);
  
  //Timers
  updateTimer = handle.createTimer(ros::Duration(3), &Corobot::updatePublishTimer, &robot);
}

int main(int argc, char** argv) {
   
  ros::init(argc, argv, "robot");
  ros::NodeHandle handle;
  ros::Subscriber sub_traj = handle.subscribe("bestTrajec", 1000, trajCallback);
  
  init_advertisers_subscribers(robot, handle);
  
  std::cout<<"\nWaiting for trajectories...\n";
  while(ros::ok()) {
    robot.moveOnTrajectory();
    std::cout<<"\nDone";
    ros::spinOnce();
  }

  std::cout<<"\nExiting Normally\n";
  return 0;
}
