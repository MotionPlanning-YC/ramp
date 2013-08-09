#ifndef SUBSCRIBE_AND_PUBLISH_H
#define SUBSCRIBE_AND_PUBLISH_H
#include "ros/ros.h"
#include "trajectory_request_handler.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "ramp_msgs/ModificationRequest.h"

class SubscribeAndPublish {
  public:
    
    SubscribeAndPublish(ros::NodeHandle& h);
    ~SubscribeAndPublish();

    void callback(const ramp_msgs::ModificationRequest::ConstPtr& msg);
  
  private:  
    ramp_msgs::Path extractPath(ramp_msgs::Trajectory traj);
    std::vector<float> extractTimes(ramp_msgs::Trajectory traj);
    
    ros::NodeHandle handle_;
    ros::Publisher  pub_mod_trajs_;
    ros::Subscriber sub_mod_reqs_;  
    TrajectoryRequestHandler* h_traj_req_;
};

#endif
