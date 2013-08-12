#include "subscribe_and_publish.h"


/** Initialize the publisher and subscriber */
SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle& h) : handle_(h) {

  pub_mod_paths_ = handle_.advertise<ramp_msgs::Path>("modified_paths", 1000); 
  sub_mod_reqs_  = handle_.subscribe("modification_requests", 1000, &SubscribeAndPublish::callback, this); 

}


SubscribeAndPublish::~SubscribeAndPublish() {}




/** Each time the package receives a TrajectoryRequst,
 *  it will generate a Trajectory based on the request and publish that Trajectory */
void SubscribeAndPublish::callback(const ramp_msgs::ModificationRequest::ConstPtr& msg) {

  //When a modification request on a path is received,
 
  ramp_msgs::Path p = msg->paths.at(0); 
  //***work to modify the trajectory***
  //p = modification_object.go(p);
  //For now we just leave it equal to the original
 
  ramp_msgs::Path mod_path = p;
  mod_path.id = p.id;

  //Publish the modified trajectory 
  pub_mod_paths_.publish(mod_path);
}
