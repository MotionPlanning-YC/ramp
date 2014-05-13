#include "ros/ros.h"
#include "corobot.h"
#include "ramp_msgs/MotionState.h"

Corobot robot;

void trajCallback(const ramp_msgs::Trajectory::ConstPtr& msg) {
  //std::cout<<"\nGot the trajectory message!\n";
  //std::cout<<"\nPress enter to call updateTrajectory\n";
  //std::cin.get();
  robot.updateTrajectory(*msg);
}


/** Initialize the Corobot's publishers and subscibers*/
void init_advertisers_subscribers(Corobot& robot, ros::NodeHandle& handle, bool simulation) {

  
  // Publishers
  robot.pub_phidget_motor_ = handle.advertise<corobot_msgs::MotorCommand>(Corobot::TOPIC_STR_PHIDGET_MOTOR, 1000);
  robot.pub_twist_ = handle.advertise<geometry_msgs::Twist>(Corobot::TOPIC_STR_TWIST, 1000);
  robot.pub_update_ = handle.advertise<ramp_msgs::MotionState>(Corobot::TOPIC_STR_UPDATE, 1000);

  if(simulation) {
    robot.pub_cmd_vel_ = handle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  }
 
  // Subscribers
  robot.sub_odometry_ = handle.subscribe(Corobot::TOPIC_STR_ODOMETRY, 1000, &Corobot::updateState, &robot);
  
  // Timers
  robot.timer_ = handle.createTimer(ros::Duration(0.04), &Corobot::updatePublishTimer, &robot);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "ramp_control");
  ros::NodeHandle handle;  
  ros::Subscriber sub_traj = handle.subscribe("bestTrajec", 1000, trajCallback);
  
  handle.param("orientation", robot.initial_theta_, 0.);
  std::cout<<"\nrobot.orientation: "<<robot.initial_theta_;
  
  bool sim=false;
  handle.param("ramp_control/simulation", sim, false);
  std::cout<<"\nsim: "<<sim<<"\n";
 
 

  // Initialize publishers and subscribers
  robot.init(handle);
  init_advertisers_subscribers(robot, handle, sim);


  // Make a blank ramp_msgs::Trajectory
  ramp_msgs::Trajectory init;
  robot.trajectory_ = init;

  while(ros::ok()) {
    robot.moveOnTrajectory(sim);
    ros::spinOnce();
  }


  std::cout<<"\nExiting Normally\n";
  return 0;
}
