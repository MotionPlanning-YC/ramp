#ifndef TRAJECTORY_REQUEST_HANDLER_H
#define TRAJECTORY_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/TrajectoryRequest.h"

class TrajectoryRequestHandler {
  public:
    TrajectoryRequestHandler();
    TrajectoryRequestHandler(const ros::NodeHandle& h);

    //Cannot make r const because it has no serialize/deserialize
    const bool request(ramp_msgs::TrajectoryRequest& tr);

  private:
    ros::NodeHandle  handle_; 
    ros::ServiceClient client_;
};

#endif