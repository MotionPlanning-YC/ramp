#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/ObjectList.h"
#include "ir_object.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Update.h"
#include "dynamic_object.h"


ros::Publisher pub_obj;
ramp_msgs::ObjectList list;
DynamicObject otherRobot;

/** Get the other robot's current location and update its pose */
void updateOtherRobotCb(const nav_msgs::Odometry& o) {
  otherRobot.update(o);
} //End updateOtherRobotCb



/** Prepare the object list by adding on all of the objects */
ramp_msgs::ObjectList prepareList() {
  ramp_msgs::ObjectList list;

  //Other Robot
  list.objects.push_back(otherRobot.buildObjectMsg());

  //Misc objects...

} //End prepareList

/** Publish the list of objects */
void sendList(const ros::TimerEvent& e) {
  pub_obj.publish(prepareList());
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
  pub_obj = handle.advertise<ramp_msgs::ObjectList>("object_list", 1000);

  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(0.01), sendList);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
