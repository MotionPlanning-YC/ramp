#include "subscribe_and_publish.h"


/** Initialize the publisher and subscriber */
SubscribeAndPublish::SubscribeAndPublish(ros::NodeHandle& h) : handle_(h) {

  pub_mod_trajs_ = handle_.advertise<ramp_msgs::Trajectory>("modified_trajs", 1000); 
  sub_mod_reqs_  = handle_.subscribe("modification_requests", 1000, &SubscribeAndPublish::callback, this); 

  h_traj_req_ = new TrajectoryRequestHandler(handle_);
}


SubscribeAndPublish::~SubscribeAndPublish() {
  if(h_traj_req_ != 0) {
    delete h_traj_req_;
    h_traj_req_ = 0;
  }
}



ramp_msgs::Path SubscribeAndPublish::extractPath(ramp_msgs::Trajectory traj) {
  //std::cout<<"\nIn extractPath\n"; 

  //extract the Path
  ramp_msgs::Path result;
  
  //Go through each knot point... 
  for(unsigned int i=0;i<traj.index_knot_points.size();i++) {
    
    //Get the knot point at the index
    trajectory_msgs::JointTrajectoryPoint point = traj.trajectory.points.at(traj.index_knot_points.at(i));
    
    //For each position value, push the value onto c to create the configuration
    ramp_msgs::Configuration c;
    for(unsigned int j=0;j<point.positions.size();j++) {
      c.K.push_back(point.positions.at(j));
    }
   
    //push the configuration onto the path
    result.configurations.push_back(c);
  } 

  return result;
}





std::vector<float> SubscribeAndPublish::extractTimes(ramp_msgs::Trajectory traj) {
  //std::cout<<"\nIn extractTimes\n";

  std::vector<float> result;

  //For each knot point
  for(unsigned int i=1;i<traj.index_knot_points.size();i++) {
    
    //get the durations 
    ros::Duration d1 = traj.trajectory.points.at(traj.index_knot_points.at(i-1)).time_from_start;
    ros::Duration d2 = traj.trajectory.points.at(traj.index_knot_points.at(i)).time_from_start;

    result.push_back(d2.toSec() - d1.toSec());
  }
  
  

  return result;
}





/** Each time the package receives a TrajectoryRequst,
 *  it will generate a Trajectory based on the request and publish that Trajectory */
void SubscribeAndPublish::callback(const ramp_msgs::ModificationRequest::ConstPtr& msg) {

  //When a modification request on a trajectory is received,
  
  //Extract the path
  ramp_msgs::Path p = extractPath(msg->trajs.at(0));
  
  //***work to modify the trajectory***
  //p = modification_object.go(p);
  //For now we just leave it equal to the original
 


  //Now build the trajectory request
  ramp_msgs::TrajectoryRequest msg_traj_req;

  msg_traj_req.path = p;

  //Extract the times
  std::vector<float> times = extractTimes(msg->trajs.at(0));
  
  msg_traj_req.t = times;

  //The resolutionRate should be in the mod request msg
  msg_traj_req.resolutionRate = msg->resolutionRate;

  //The ID should be the same in the mod request msg
  msg_traj_req.id = msg->id;
  
   
  std::cout<<"\nSending request to trajectory_generator!\n";
  //Send the request
  ramp_msgs::Trajectory mod_traj = h_traj_req_->request(msg_traj_req);
  
  std::cout<<"\nPublishing to modified_trajs!\n";
  pub_mod_trajs_.publish(mod_traj);
}
