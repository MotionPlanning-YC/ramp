#include "trajectory_request_handler.h"

TrajectoryRequestHandler::TrajectoryRequestHandler() : desiredId(9999), mutex_(false) {}

TrajectoryRequestHandler::TrajectoryRequestHandler(const ros::NodeHandle& h) : desiredId(9999), handle_(h), mutex_(false) {
  pub_request_ = handle_.advertise<ramp_msgs::TrajectoryRequest>("traj_requests", 1000);
  sub_traj_ = handle_.subscribe("trajs", 1000, &TrajectoryRequestHandler::callback, this); 
}



void TrajectoryRequestHandler::callback(const ramp_msgs::Trajectory::ConstPtr& msg) {
  
  std::cout<<"\nReceived a trajectory!\n";
  std::cout<<"\nmsg->id:"<<msg->id<<"\n";
  std::cout<<"\ndesiredId:"<<desiredId<<"\n";
  if(desiredId != 9999 && msg->id == desiredId) {
    received_ = *msg;
    mutex_ = true;
  }

}



ramp_msgs::Trajectory TrajectoryRequestHandler::request(const ramp_msgs::TrajectoryRequest r) {
  
  //Set mutex
  mutex_ = false;
  
  //Set the ID
  desiredId = r.id;
  
  std::cout<<"\ndesiredId:"<<desiredId;
  std::cout<<"\nPress enter to publish request\n";
  std::cin.get();
  
  //Publish the request
  pub_request_.publish(r);

  std::cout<<"\nPublished the request\n";
  
  //Wait for trajectory to be set
  while(!mutex_) {ros::spinOnce();}

  desiredId = 9999;
  return received_; 
}
