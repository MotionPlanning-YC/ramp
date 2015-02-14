#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_datatypes.h"
#include "obstacle.h"


ros::Publisher pub_obj;
//ramp_msgs::ObstacleList list;
Obstacle obstacle;

/** Get the other robot's current odometry information and update the dynamicObject */
void updateOtherRobotCb(const nav_msgs::Odometry& o) {
  obstacle.update(o);
} //End updateOtherRobotCb



/** Prepare the object list by adding on all of the objects */
/*const ramp_msgs::ObstacleList prepareList() {
  ramp_msgs::ObstacleList list;

  //Other Robot
  list.obstacles.push_back(obstacle.buildObstacleMsg());

  //Misc objects...

  return list;
} //End prepareList*/




/** Publish the list of objects */
void publishList(const ros::TimerEvent& e) {
  //pub_obj.publish(prepareList());
  pub_obj.publish(obstacle.buildObstacleMsg());
} //End sendList




int main(int argc, char** argv) {
  ros::init(argc, argv, "ramp_sensing");
  ros::NodeHandle handle;
  

  //Get parameters
  std::string other_robot_odom;
  handle.getParam("ramp_sensing/other_robot_odom", other_robot_odom);
  std::cout<<"\nother_robot_odom:"<<other_robot_odom;
  
  //Subscribers
  ros::Subscriber sub_other_robot = handle.subscribe(other_robot_odom, 100, updateOtherRobotCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::Obstacle>("object_list", 1000);

  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(1.f / 2.f), publishList);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
