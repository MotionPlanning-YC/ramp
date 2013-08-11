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
  
  std::vector<float> t;
  //For each path
  for(unsigned int i=0;i<paths_.size();i++) {

    //Hardcode some times, 2s per segment
    for(unsigned int j=1;j<paths_.at(i).all_.size();j++) {
      t.push_back(2);
    }
    
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequestMsg(i, t);
    
    population_.push_back(h_traj_req_->request(msg_request));   

    t.clear();
  }

}


const ramp_msgs::Trajectory Planner::modify(const unsigned int i_traj) const {
  ramp_msgs::Trajectory result;

  ramp_msgs::ModificationRequest mr = buildModificationRequestMsg(i_traj);
  
  result = h_mod_req_->request(mr);

  return result;
}

const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequestMsg(const unsigned int i_path, const std::vector<float> times) const {
  ramp_msgs::TrajectoryRequest result;

  result.id   = i_path;
  result.path = paths_.at(i_path).buildPathMsg();
  result.t    = times;
  result.resolutionRate = resolutionRate_;

  return result;
}

const ramp_msgs::ModificationRequest Planner::buildModificationRequestMsg(const unsigned int i_traj) const {
  ramp_msgs::ModificationRequest result;

  result.id = i_traj;
  result.trajs.push_back(population_.at(i_traj));
  result.resolutionRate = resolutionRate_;

  return result;
}

