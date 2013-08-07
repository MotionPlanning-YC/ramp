#ifndef SUBSCRIBE_AND_PUBLISH_H
#define SUBSCRIBE_AND_PUBLISH_H
#include "ros/ros.h"
#include "trajectory.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/TrajectoryRequest.h"

class SubscribeAndPublish {
  public:
    
    SubscribeAndPublish(ros::NodeHandle& h);

    void callback(const ramp_msgs::TrajectoryRequest::ConstPtr& msg);
  
  private:
    ros::NodeHandle handle_;
    ros::Publisher  pub_trajs_;
    ros::Subscriber sub_traj_reqs_;  
};

#endif
