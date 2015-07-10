#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "obstacle.h"


ros::Publisher pub_obj;
//ramp_msgs::ObstacleList list;
Obstacle obstacle;

/** Get the other robot's current odometry information and update the dynamicObject */
void updateOtherRobotCb(const nav_msgs::Odometry& o) 
{
  obstacle.update(o);
} //End updateOtherRobotCb




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) 
{
  pub_obj.publish(obstacle.buildObstacleMsg());
} //End sendList




int main(int argc, char** argv) 
{
  ros::init(argc, argv, "ramp_sensing");
  ros::NodeHandle handle;
  

  //Get parameters
  std::string other_robot_odom;
  handle.getParam("ramp_sensing/other_robot_odom", other_robot_odom);
  std::cout<<"\nother_robot_odom:"<<other_robot_odom;
  
  //Subscribers
  ros::Subscriber sub_other_robot = handle.subscribe(other_robot_odom, 100, updateOtherRobotCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::Obstacle>("obstacles", 1000);

  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(1.f / 5.f), publishList);
  //timer.start();
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
