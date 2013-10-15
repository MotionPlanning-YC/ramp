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
void updateOtherRobotCb(const ramp_msgs::Update& u) {
  geometry_msgs::Pose current_pose;

  //Set the x, y, and z positions
  current_pose.position.x = u.configuration.K.at(0);
  current_pose.position.y = u.configuration.K.at(1);
  current_pose.position.z = 0;

  //Create quaternion for pose
  geometry_msgs::Quaternion q;
  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, u.configuration.K.at(2)), q);  
  
  //Set quaternion for pose
  current_pose.orientation = q;

  //Update the other robot's current pose
  otherRobot.updatePose(current_pose, ros::Time::now());

  //Update the object in the list
  list.objects.at(0) = otherRobot.buildObjectMsg();
} //End updateOtherRobotCb



/** Prepare the object list by adding on all of the objects */
ramp_msgs::ObjectList prepareList() {
  ramp_msgs::ObjectList list;

  //Other Robot
  list.objects.push_back(otherRobot.buildObjectMsg());
} //End prepareList

/** Publish the list of objects */
void sendList(const ros::TimerEvent& e) {
  pub_obj.publish(prepareList());
} //End sendList




int main(int argc, char** argv) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle handle;
  

  //Get parameters
  std::string other_robot_topic_name;
  handle.getParam("sensing/other_robot_topic", other_robot_topic_name);
  std::cout<<"\nother_robot_topic_name:"<<other_robot_topic_name;
  
  //Subscribers
  ros::Subscriber sub_other_robot = handle.subscribe(other_robot_topic_name, 100, updateOtherRobotCb);

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObjectList>("object_list", 1000);

  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(0.01), sendList);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
