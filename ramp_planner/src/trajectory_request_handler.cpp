#include "trajectory_request_handler.h"

TrajectoryRequestHandler::TrajectoryRequestHandler() : desiredId_(9999), mutex_(false) {}

TrajectoryRequestHandler::TrajectoryRequestHandler(const ros::NodeHandle& h) : desiredId_(9999), handle_(h), mutex_(false) {
  pub_request_ = handle_.advertise<ramp_msgs::TrajectoryRequest>("traj_requests", 1000);
  sub_traj_ = handle_.subscribe("trajs", 1000, &TrajectoryRequestHandler::callback, this); 
}



void TrajectoryRequestHandler::callback(const ramp_msgs::Trajectory::ConstPtr& msg) {

  //Check the id
  if(desiredId_ != 9999 && msg->id == desiredId_) {
    received_ = *msg;
    mutex_    = true;
  }

}



ramp_msgs::Trajectory TrajectoryRequestHandler::request(const ramp_msgs::TrajectoryRequest r) {
  
  //Set mutex
  mutex_ = false;
  
  //Set the ID
  desiredId_ = r.id;
  
  //Publish the request
  pub_request_.publish(r);
  
  //Wait for trajectory to be set
  while(!mutex_) {ros::spinOnce();}

  //reset ID and return
  desiredId_ = 9999;
  return received_; 
}
