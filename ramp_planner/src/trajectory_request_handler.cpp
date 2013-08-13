#include "trajectory_request_handler.h"

TrajectoryRequestHandler::TrajectoryRequestHandler() {}

TrajectoryRequestHandler::TrajectoryRequestHandler(const ros::NodeHandle& h) : handle_(h) {
  client_ = handle_.serviceClient<ramp_msgs::TrajectoryRequest>("trajectory_generator");
}


const bool TrajectoryRequestHandler::request(ramp_msgs::TrajectoryRequest& tr) {
  
  if(client_.call(tr)) 
    return true;

  return false;
}
