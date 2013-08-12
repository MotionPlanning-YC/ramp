#ifndef SUBSCRIBE_AND_PUBLISH_H
#define SUBSCRIBE_AND_PUBLISH_H
#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"

class SubscribeAndPublish {
  public:
    
    SubscribeAndPublish(ros::NodeHandle& h);
    ~SubscribeAndPublish();

    void callback(const ramp_msgs::ModificationRequest::ConstPtr& msg);
  
  private:  
    
    ros::NodeHandle handle_;
    ros::Publisher  pub_mod_paths_;
    ros::Subscriber sub_mod_reqs_;  
};

#endif
