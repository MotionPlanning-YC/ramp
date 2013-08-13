#include "ros/ros.h"
#include "ramp_msgs/ModificationRequest.h"

bool handleRequest(ramp_msgs::ModificationRequest::Request& req,
                   ramp_msgs::ModificationRequest::Response& res)
{
  
  //For now, just push back on the paths passed in
  for(unsigned int i=0;i<req.paths.size();i++) {
    res.mod_paths.push_back(req.paths.at(i));
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_modification");
  
  ros::NodeHandle handle;

  ros::ServiceServer service = handle.advertiseService("path_modification", handleRequest); 

  std::cout<<"\nSpinning...\n";
  ros::spin();
  std::cout<<"\nExiting Normally\n";
  return 0;
}
