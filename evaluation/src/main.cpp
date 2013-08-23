#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"

bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  Evaluate ev(req);
  res.fitness = ev.perform();
  res.feasible = true;
  return true;
}

int main(int argc, char** argv) {


  ros::init(argc, argv, "evaluation");
  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("evaluation", handleRequest);

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
