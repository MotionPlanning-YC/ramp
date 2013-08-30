#ifndef UPDATE_CONFIGURATION_HANDLER_H
#define UPDATE_CONFIGURATION_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/UpdateRequest.h"

class UpdateRequestHandler {
  public:
    UpdateRequestHandler(const ros::NodeHandle& h);

    const bool request(ramp_msgs::UpdateRequest& ur);

  private:
    ros::NodeHandle handle_;
    ros::ServiceClient client_;
};

#endif
