#include "evaluation_request_handler.h"


EvaluationRequestHandler::EvaluationRequestHandler(const ros::NodeHandle& h) : handle_(h) {
  client_ = handle_.serviceClient<ramp_msgs::EvaluationRequest>("trajectory_evaluation", true);
}


const bool EvaluationRequestHandler::request(ramp_msgs::EvaluationRequest& er) {
  if(client_.call(er)) 
    return true;

  return false;
}
