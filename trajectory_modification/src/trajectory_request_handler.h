#ifndef TRAJECTORY_REQUEST_HANDLER_H
#define TRAJECTORY_REQUEST_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/TrajectoryRequest.h"

class TrajectoryRequestHandler {
  public:
    TrajectoryRequestHandler();
    TrajectoryRequestHandler(const ros::NodeHandle& h);

    ramp_msgs::Trajectory request(const ramp_msgs::TrajectoryRequest r);
    void callback(const ramp_msgs::Trajectory::ConstPtr& msg); 
  private:
    ros::NodeHandle  handle_; 
    ros::Publisher   pub_request_;
    ros::Subscriber  sub_traj_;

    unsigned int desiredId;
    ramp_msgs::Trajectory  received_;
    bool                   mutex_;
};

#endif
