#include "update_request_handler.h"

UpdateRequestHandler::UpdateRequestHandler(const ros::NodeHandle& h) : handle_(h) {
  client_ = handle_.serviceClient<ramp_msgs::UpdateRequest>("update_configuration");
}


const bool UpdateRequestHandler::request(ramp_msgs::UpdateRequest& ur) {
  return client_.call(ur);
}
