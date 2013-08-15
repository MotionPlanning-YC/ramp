#include "planner.h"


Planner::Planner() : resolutionRate_(5), populationSize_(7), h_traj_req_(0) {}

Planner::Planner(const int p) : resolutionRate_(5), populationSize_(p), h_traj_req_(0) {}

Planner::Planner(const unsigned int r) : resolutionRate_(r), populationSize_(7), h_traj_req_(0) {}

Planner::Planner(const unsigned int r, const int p) : resolutionRate_(r), populationSize_(p), h_traj_req_(0) {}



Planner::~Planner() {
  if(h_traj_req_ != 0) {
    delete h_traj_req_;  
    h_traj_req_ = 0;
  }
  
  if(h_mod_req_ != 0) {
    delete h_mod_req_;  
    h_mod_req_ = 0;
  }
}


void Planner::init_handlers(const ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_mod_req_  = new ModificationRequestHandler(h);
}


/** This function generates the initial population of trajectories */
void Planner::initialization() { 
  
  //Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    //Create the path with the start and goal
    Path temp_path(start_, goal_);

    //Each trajectory will have a random number of knot points
    //Put a max of 10 knot points for practicality...
    unsigned int num = rand() % 10;

    //For each knot point to be created...
    for(unsigned int j=0;j<num;j++) {

      //Create a random configuration
      Configuration temp_config;
      temp_config.ranges_ = ranges_;
      temp_config.random();
      
      //Add the random configuration to the path
      temp_path.Add(temp_config); 
    }

    //Add the path to the list of paths
    paths_.push_back(temp_path);
  }
  
  std::vector<float> v;
  //For each path
  for(unsigned int i=0;i<paths_.size();i++) {

    //Hardcode some times, 2s per segment
    for(unsigned int j=1;j<paths_.at(i).all_.size();j++) {
      v.push_back(10.0f);
    }
    
    //Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequestMsg(i,v);
    
    //Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      population_.push_back(msg_request.response.trajectory);
    }
    else {
      //some error handling
    }

    //Push the times vector onto times_
    velocities_.push_back(v);

    //Clear times vector
    v.clear();
  }

}


const bool Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr) const {
  return h_traj_req_->request(tr); 
}

const bool Planner::requestModification(ramp_msgs::ModificationRequest& mr) const {
  return h_mod_req_->request(mr);
}




const std::vector<ramp_msgs::Path> Planner::modifyPath(const unsigned int i1, const unsigned int i2) const {
  std::vector<ramp_msgs::Path> result;

  //Build a modification request
  ramp_msgs::ModificationRequest mr = buildModificationRequestMsg(i1, i2);
  
  //Request the modification handler to make the modification
  //It returns the modified Path
  if(requestModification(mr)) {
    result = mr.response.mod_paths;
  }
  else {
    //some error handling
  }

  return result;
}


const std::vector<ramp_msgs::Trajectory> Planner::modifyTraj(const unsigned int i1, const unsigned int i2) {
  std::vector<ramp_msgs::Trajectory> result;

  //The modification operators deal with paths
  //So send the path to be modified
  std::vector<ramp_msgs::Path> mp = modifyPath(i1, i2);

  for(unsigned int i=0;i<mp.size();i++) {
    
    Path mod_path(mp.at(i));
    std::cout<<"\nnew path:"<<mod_path.toString();

    paths_.push_back(mod_path); 

    std::vector<float> v = velocities_.at(i);
    v.push_back(10.0f);
    std::cout<<"v:";
    for(unsigned int i=0;i<v.size();i++) {
      std::cout<<" "<<v.at(i);
    }

    //Now build a TrajectoryRequestMsg
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequestMsg(mod_path, v);
   
    //Send the request and set the result to the returned trajectory 
    if(requestTrajectory(tr)) {
      result.push_back(tr.response.trajectory);
    }
    else {
      //some error handling
    }
  
  }
  
  return result;
}


/** Build a ModificationRequest msg */
const ramp_msgs::ModificationRequest Planner::buildModificationRequestMsg(const unsigned int i_path, const unsigned int i_path2) const {
  ramp_msgs::ModificationRequest result;

  result.request.paths.push_back(paths_.at(i_path).buildPathMsg() );

  if(i_path2 != -1)
    result.request.paths.push_back(paths_.at(i_path2).buildPathMsg() );

  return result;
}



/** Build a TrajectoryRequest msg */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequestMsg(const Path path, const std::vector<float> velocities ) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path = path.buildPathMsg();
  result.request.v    = velocities;
  result.request.resolutionRate = resolutionRate_;

  return result;
}


/** Build a TrajectoryRequest msg */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequestMsg(const unsigned int i_path, const std::vector<float> velocities) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path = paths_.at(i_path).buildPathMsg();
  result.request.v    = velocities;
  result.request.resolutionRate = resolutionRate_;
  
  return result; 
}
