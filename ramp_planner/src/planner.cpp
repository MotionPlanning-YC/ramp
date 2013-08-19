#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(5), populationSize_(7), generation_(0), h_traj_req_(0), modifier_(0) {}

Planner::Planner(const unsigned int r, const int p) : resolutionRate_(r), populationSize_(p), h_traj_req_(0), modifier_(0) {}

Planner::Planner(const ros::NodeHandle& h) : resolutionRate_(5), populationSize_(7), generation_(0) {
  init_handlers(h);
}

Planner::~Planner() {
  if(h_traj_req_!= 0) {
    delete h_traj_req_;  
    h_traj_req_= 0;
  }
  
  if(modifier_!= 0) {
    delete modifier_;  
    modifier_= 0;
  }
}






/****************************************************
 ************** Initialization Methods **************
 ****************************************************/


/** Initialize the handlers and allocate them on the heap */
void Planner::init_handlers(const ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler(h);
  modifier_ = new Modifier(h, paths_);
}


/** This function generates the initial population of trajectories */
void Planner::init_population() { 
  
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

    //Hardcode some velocities, 1m/s per segment
    for(unsigned int j=1;j<paths_.at(i).all_.size();j++) {
      v.push_back(1.0f);
    }
    
    //Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i,v,v);
    
    //Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp;
      
      //Set the Trajectory msg
      temp.trajec_ = msg_request.response.trajectory;
      population_.add(temp);
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





/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
const bool Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr) {
  return h_traj_req_->request(tr); 
}



/******************************************************
 ****************** Modifying Methods *****************
 ******************************************************/


/** Modify a Path */
const std::vector<Path> Planner::modifyPath() { 
  return modifier_->perform();
}


/** Modify a trajectory 
 *  Can accept 2 ids if the modification operator is binary */
const std::vector<RampTrajectory> Planner::modifyTrajec(const unsigned int i1, const unsigned int i2) {
  std::vector<RampTrajectory> result;

  //The modification operators deal with paths
  //So send the path to be modified
  std::vector<Path> mp = modifyPath();

  for(unsigned int i=0;i<mp.size();i++) {
    
    Path mod_path = mp.at(i);
    std::cout<<"\nnew path:"<<mod_path.toString();

    paths_.push_back(mod_path); 

    std::vector<float> v = velocities_.at(i);
    v.push_back(10.0f);
    std::cout<<"v:";
    for(unsigned int i=0;i<v.size();i++) {
      std::cout<<" "<<v.at(i);
    }

    //Now build a TrajectoryRequestMsg
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(mod_path, v, v);
   
    //Send the request and set the result to the returned trajectory 
    if(requestTrajectory(tr)) {
      RampTrajectory temp;
      temp.trajec_ = tr.response.trajectory;
      result.push_back(temp);
    }
    else {
      //some error handling
    }
  
  }
  
  return result;
}





/******************************************************
 **************** Srv Building Methods ****************
 ******************************************************/



/** Build a TrajectoryRequest msg */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e ) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path = path.buildPathMsg();
  result.request.v_start    = v_s;
  result.request.v_end      = v_e;
  result.request.resolutionRate = resolutionRate_;

  return result;
}


/** Build a TrajectoryRequest msg */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path = paths_.at(i_path).buildPathMsg();
  result.request.v_start    = v_s;
  result.request.v_end      = v_e;
  result.request.resolutionRate = resolutionRate_;
  
  return result; 
}






/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


 void Planner::go() {

  //t=0
  generation_ = 0;
  
  //initialize population
  init_population();

  RampTrajectory T_move = population_.evaluateAndObtainBest();

  //createSubpopulations();
  
  while(!current_.equals(goal_)) {
    generation_++;

    //Call modification
    std::vector<Path> mod_path = modifier_->perform();
    //Get trajectories for the modifications
    //std::vector<RampTrajectory> mod_trajec = modifyTrajec(); 
    
    //Replace 1-2 in population with mod_path
    //for(unsigned int i=0;i<mod_path.size();i++) {
      //population_.add(mod_
    //}
  
  
  }

}
