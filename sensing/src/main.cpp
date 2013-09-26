#include <iostream>
#include "ros/ros.h"
#include "ramp_msgs/Update.h"
#include "ir_object.h"

ramp_msgs::Configuration current;


/** Get the robot's current configuration so we know where the IR objects are at */
void updateCallback(const ramp_msgs::Update& msg) {
  current.K.clear();

  for(unsigned int i=0;i<msg.configuration.K.size();i++) {
    current.K.push_back(msg.configuration.K.at(i));
  }
}

/** Get the IR sensor msgs */
void irCallback(const corobot_msgs::SensorMsg& msg) {

  //If infrared
  if(msg.type == msg.INFRARED_FRONT) {

    //If there is an object in the way
    if(msg.value < 20) {
      IrObject obj(msg, current);
    }
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "sensing");
  ros::NodeHandle handle;

  
  ros::Subscriber sub_ir = handle.subscribe("infrared_data", 100, irCallback);
  ros::Subscriber sub_current = handle.subscribe("update", 100, updateCallback);
  
  //ros::Publisher pub_obj = handle.advertise<


  std::cout<<"\nSpinning\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
