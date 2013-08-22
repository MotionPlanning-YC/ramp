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
  h_control_  = new ControlHandler(h);
  modifier_   = new Modifier(h, paths_, velocities_);
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
      
      //Add the trajectory to the population
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






/** This method returns the new velocity vector(s) for the modified path(s) */
const std::vector< std::vector<float> > Planner::getNewVelocities(std::vector<Path> new_paths, std::vector<int> i_old) {
  std::vector< std::vector<float> > result; 

  //If the modification only changed one path, i.e., not a crossover 
  if(new_paths.size() == 1) {

    //Get the original velocity values
    std::vector<float> v = velocities_.at(i_old.at(0));

    //Get the old and new paths
    Path old = paths_.at(i_old.at(0));
    Path new_path = new_paths.at(0);

    //Check the difference in path sizes
    int diff = new_path.all_.size() - old.all_.size();

    //Insertion
    if(diff == 1) {
    
      //Find the insertion
      for(unsigned int i=0;i<new_path.all_.size();i++) {
        if(!new_path.all_.at(i).equals(old.all_.at(i))) {
          v.push_back(v.at(i-1));
          i=new_path.all_.size();
        }
      } //end for
    } //end if insertion 

    //Deletion
    else if(diff == -1) {
      
      //Find the deletion
      for(unsigned int i=0;i<new_path.all_.size();i++) {
        if(!new_path.all_.at(i).equals(old.all_.at(i))) {
          v.erase(v.begin()+i);
          i=new_path.all_.size();
        }
      } //end for
    } //end if deletion

    //Push onto the result
    result.push_back(v);
  } //end if not crossover


  //Crossover
  else {
    
    //For each path,
    for(unsigned int i=0;i<new_paths.size();i++) {
    
      //Get the old and new paths
      Path old = paths_.at(i_old.at(i)); 
      Path new_path = new_paths.at(i);

      //Get the id of the path it was crossed with 
      unsigned int i_crossed = (i==0) ? i_old.at(1) : i_old.at(0);

      //Make a Path object for it
      Path crossed_path = paths_.at(i_crossed);

      //This will hold the velocities
      std::vector<float> v;

      //Find where the crossover occurred
      unsigned int i_cross=0;
      for(unsigned int c=0;c<old.all_.size();c++) {
        
        //If the two configurations are different, that is where the crossover occurred
        if(!old.all_.at(c).equals(new_path.all_.at(c))) {
          i_cross = c;
          c = old.all_.size();
        }

        //If the two configurations are the same, push on the velocity value for the segment
        else if(c<old.all_.size()-1) {
          v.push_back(velocities_.at(i_old.at(i)).at(c));
        }

        //If we reach the end, the crossed path is too small
        else {
          i_cross = old.all_.size();
        }
      } //end for c
      
      //Push all the velocity values from the crossed path (if not too small)
      for(unsigned int c=i_cross;c<crossed_path.all_.size()-1;c++) {
        v.push_back(velocities_.at(i_crossed).at(c));
      }
      
      //Push the vector of velocities onto the result
      result.push_back(v); 
    } //end for
  }
  
  //Swap and Change do not change the size

  return result;
} //End getNewVelocities





/** Modify a trajectory 
 *  Can accept 2 ids if the modification operator is binary */
const std::vector<RampTrajectory> Planner::modifyTrajec() {
  std::vector<RampTrajectory> result;


  //The modification operators deal with paths
  //So send the path to be modified
  std::vector<Path> mp = modifyPath();

  //Hold the ids of the path(s) modified
  std::vector<int> olds;
  olds.push_back(modifier_->i_changed1);

  //If a crossover was performed, push on the 2nd path 
  if(mp.size()>1) {
    olds.push_back(modifier_->i_changed2);
  }

  //Hold the new velocity vector(s)
  std::vector< std::vector<float> > vs = getNewVelocities(mp, olds); 
  
  //For each modified path,
  for(unsigned int i=0;i<mp.size();i++) {
    
    //Build a TrajectoryRequestMsg
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(mp.at(i), vs.at(i), vs.at(i));
    
    //Send the request and set the result to the returned trajectory 
    if(requestTrajectory(tr)) {
      RampTrajectory temp;
      temp.trajec_ = tr.response.trajectory;
      result.push_back(temp);
    }
  }
  
  return result;
}


/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  and set tau to the new best, */
void Planner::modification() {
  std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  
  for(unsigned int i=0;i<mod_trajec.size();i++) {
    population_.add(mod_trajec.at(i));
  }

  //Evaluate and obtain the best trajectory
  //****we need a way to only evaluate the new trajectories****
  //****once evaluation package exists, make evaluation there****
  //evaluate mod_trajec

  //Obtain best trajectory
  bestTrajec_ = population_.getBest();
  
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
 ******************** Miscellaneous ********************
 *******************************************************/
void Planner::sendBest() {
  h_control_->send(bestTrajec_.trajec_);
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
    
    //t=t+1
    generation_++;

    //Call modification
    modification();

    //If end of current control cycle
    if(generation_ % 3 == 0) {
      //Update starting configuration and velocity of P(t)
      //

      //Create subpopulations in P(t)
      //

      //Evaluate P(t) and obtain best T, T_move=T_best
      T_move = population_.evaluateAndObtainBest();
      
      //Send the best trajectory 
      sendBest();
    }

  
  
  }

}
