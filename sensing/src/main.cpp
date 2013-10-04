#include <iostream>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/ObjectList.h"
#include "ir_object.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Update.h"

ramp_msgs::ObjectList list;
ramp_msgs::Configuration current;
ros::Publisher pub_obj;


/** Get the robot's current configuration so we know where the IR objects are at */
/*void currentCallback(const nav_msgs::Odometry& msg) {
  current.K.clear();

  current.K.push_back(msg.pose.pose.position.x);
  current.K.push_back(msg.pose.pose.position.y);
  current.K.push_back(tf::getYaw(msg.pose.pose.orientation));
}*/

/** Get the IR sensor msgs */
/*void irCallback(const corobot_msgs::SensorMsg& msg) {

  //If infrared
  if(msg.type == msg.INFRARED_FRONT) {

    //If there is an object in the way
    if(msg.value < 20) {
      IrObject obj(msg, current);
      list.ir_object = obj.buildIRObjectMsg();
    }
  }
}*/


void updateCallback(const ramp_msgs::Update& u) {
  
}


/** Publish the list of objects */
void sendList(const ros::TimerEvent& e) {
  pub_obj.publish(list);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle handle;
  

  //Get parameters
  std::string object_topic_name;
  handle.getParam("sensing/update_topic", object_topic_name);
  std::cout<<"\nobject_topic_name:"<<object_topic_name;
  
  
  //Subscribers
  ros::Subscriber sub_other_robot = handle.subscribe(object_topic_name, 100, updateCallback);
  //ros::Subscriber sub_current = handle.subscribe("odometry", 100, currentCallback);
  //ros::Subscriber sub_ir = handle.subscribe("infrared_data", 100, irCallback);
  

  //Publishers
  pub_obj = handle.advertise<ramp_msgs::ObjectList>("object_list", 1000);


  //Timers
  ros::Timer timer = handle.createTimer(ros::Duration(0.01), sendList);
   

  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
