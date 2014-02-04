#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/ObstacleList.h"
#include "ir_object.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Update.h"
#include "obstacle.h"


ros::Publisher pub_obj;
//ramp_msgs::ObstacleList list;
Obstacle otherRobot;

/** Get the other robot's current odometry information and update the dynamicObject */
void updateOtherRobotCb(const nav_msgs::Odometry& o) {
  otherRobot.update(o);
} //End updateOtherRobotCb



/** Prepare the object list by adding on all of the objects */
const ramp_msgs::ObstacleList prepareList() {
  ramp_msgs::ObstacleList list;

  //Other Robot
  list.obstacles.push_back(otherRobot.buildObstacleMsg());

  //Misc objects...

  return list;
} //End prepareList




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) {
  //pub_obj.publish(prepareList());
  pub_obj.publish(otherRobot.buildObstacleMsg());
} //End sendList




int main(int argc, char** argv) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle handle;
  

  //Get parameters
  std::string other_robot_odom;
  handle.getParam("sensing/other_robot_odom", other_robot_odom);
  std::cout<<"\nother_robot_odom:"<<other_robot_odom;
  
  //Subscribers
  ros::Subscriber sub_other_robot = handle.subscribe(other_robot_odom, 100, updateOtherRobotCb);

  //Publishers
  //pub_obj = handle.advertise<ramp_msgs::ObstacleList>("object_list", 1000);
  pub_obj = handle.advertise<ramp_msgs::Obstacle>("object_list", 1000);

  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(0.1), publishList);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
