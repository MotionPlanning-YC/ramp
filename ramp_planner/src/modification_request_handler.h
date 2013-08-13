#ifndef MODIFICATION_REQUEST_HANDLER_H
#define MODIFICATION_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/ModifiedPath.h"

class ModificationRequestHandler {
  public:
    ModificationRequestHandler();
    ModificationRequestHandler(const ros::NodeHandle& h);
   
    
    const bool request(ramp_msgs::ModificationRequest& mr);   

  private:
    ros::NodeHandle handle_;
    ros::ServiceClient client_;
};

#endif