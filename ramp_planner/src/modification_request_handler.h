#ifndef MODIFICATION_REQUEST_HANDLER_H
#define MODIFICATION_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/ModifiedPath.h"

class ModificationRequestHandler {
  public:
    ModificationRequestHandler();
    ModificationRequestHandler(const ros::NodeHandle& h);
    
    ramp_msgs::Path request(const ramp_msgs::ModificationRequest m);   
    void callback(const ramp_msgs::Path::ConstPtr& msg);

  private:
    ros::NodeHandle handle_;
    ros::Publisher  pub_request_;
    ros::Subscriber sub_traj_;

    unsigned int desiredId_;
    ramp_msgs::Path received_;
    bool mutex_;
};

#endif
