#include "trajectory_request_handler.h"


TrajectoryRequestHandler::TrajectoryRequestHandler(const ros::NodeHandle& h) : handle_(h) {
  client_ = handle_.serviceClient<ramp_msgs::TrajectoryRequest>("/trajectory_generator", true);
}


const bool TrajectoryRequestHandler::request(ramp_msgs::TrajectoryRequest& tr) {
  return client_.call(tr);
}
