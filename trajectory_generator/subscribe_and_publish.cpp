#include "subscribe_and_publish.h"


/** Initialize the publisher and subscriber */
SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle& h) : handle_(h) {
  pub_trajs_ = handle_.advertise<ramp_msgs::Trajectory>("trajs", 1000); 
  sub_traj_reqs_ = handle_.subscribe("traj_requests", 1000, &SubscribeAndPublish::callback, this); 
}

/** Each time the package receives a TrajectoryRequst,
 *  it will generate a Trajectory based on the request and publish that Trajectory */
void SubscribeAndPublish::callback(const ramp_msgs::TrajectoryRequest::ConstPtr& msg) {
  
  std::cout<<"\nReceived a trajectory request!\n";

  
  //Initialize trajectory
  Trajectory traj(*msg);

  //Generate trajectory
  traj.generate();

  //Build msg to publish
  ramp_msgs::Trajectory msg_traj = traj.buildTrajectoryMsg();

  //Set ID
  msg_traj.id = msg->id;
  
  //Publish
  pub_trajs_.publish(msg_traj);
}
