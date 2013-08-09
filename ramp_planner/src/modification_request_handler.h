#ifndef MODIFICATION_REQUEST_HANDLER_H
#define MODIFICATION_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"

class ModificationRequestHandler {
  public:
    ModificationRequestHandler();
    ModificationRequestHandler(const ros::NodeHandle& h);
    
    ramp_msgs::Trajectory request(const ramp_msgs::ModificationRequest m);   
    void callback(const ramp_msgs::Trajectory::ConstPtr& msg);

  private:
    ros::NodeHandle handle_;
    ros::Publisher pub_request_;
    ros::Subscriber sub_traj_;
};

#endif
