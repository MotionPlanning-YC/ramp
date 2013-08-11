#include "modification_request_handler.h"


ModificationRequestHandler::ModificationRequestHandler() : desiredId_(9999), mutex_(false) {} 

ModificationRequestHandler::ModificationRequestHandler(const ros::NodeHandle& h) : desiredId_(9999), handle_(h), mutex_(false) {
  
  pub_request_ = handle_.advertise<ramp_msgs::ModificationRequest>("modification_requests", 1000);
  sub_traj_ = handle_.subscribe("modified_trajs", 1000, &ModificationRequestHandler::callback, this);

}


void ModificationRequestHandler::callback(const ramp_msgs::Trajectory::ConstPtr& msg) {

  //Check the id
  if(desiredId_ != 9999 && msg->id == desiredId_) {
    received_ = *msg;
    mutex_    = true;
  }

}

ramp_msgs::Trajectory ModificationRequestHandler::request(const ramp_msgs::ModificationRequest mr) {
  
  //Set mutex
  mutex_ = false;

  //Set the ID
  desiredId_ = mr.id;

  //Publish the request
  pub_request_.publish(mr);

  //Wait for trajectory to be set
  while(!mutex_) {ros::spinOnce();}

  //reset ID and return
  desiredId_ = 9999;
  return received_;
}
