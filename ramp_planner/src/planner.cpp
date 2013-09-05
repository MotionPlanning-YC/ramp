#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(5), populationSize_(7), generation_(0), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), mutex_start_(true) 
{
  controlCycle_ = ros::Duration(3);
}

Planner::Planner(const unsigned int r, const int p) : resolutionRate_(r), populationSize_(p), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), mutex_start_(true) 
{
  controlCycle_ = ros::Duration(3);
}

Planner::Planner(const ros::NodeHandle& h) : resolutionRate_(5), populationSize_(7), generation_(0), mutex_start_(true) 
{
  init_handlers(h); 
  controlCycle_ = ros::Duration(3);
}

Planner::~Planner() {
  if(h_traj_req_!= 0) {
    delete h_traj_req_;  
    h_traj_req_= 0;
  }

  if(h_control_ != 0) {
    delete h_control_;
    h_control_ = 0;
  }

  if(h_eval_req_ != 0) {
    delete h_eval_req_;
    h_eval_req_ = 0;
  }
  
  if(modifier_!= 0) {
    delete modifier_;  
    modifier_= 0;
  }
}


/** Getter method for start_. It waits for mutex_start_ to be true before returning. */
Configuration Planner::getStartConfiguration() {
  while(!mutex_start_) {}
  return start_;
}

/** Sets start_ */
void Planner::updateCallback(const ramp_msgs::Update::ConstPtr& msg) {
  
  //Wait for mutex to be true
  while(!mutex_start_) {}

  mutex_start_ = false;

  Configuration temp(msg->configuration);
  start_ = temp;

  mutex_start_ = true;
}


/****************************************************
 ************** Initialization Methods **************
 ****************************************************/


/** Initialize the handlers and allocate them on the heap */
void Planner::init_handlers(const ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_control_  = new ControlHandler(h);
  h_eval_req_ = new EvaluationRequestHandler(h);
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

    //Hardcode some velocities, 0.5m/s per segment
    for(unsigned int j=1;j<paths_.at(i).all_.size();j++) {
      v.push_back(0.5f);
    }
    
    //Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i,v,v);
    
    //Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp;
      
      //Set the Trajectory msg
      temp.msg_trajec_ = msg_request.response.trajectory;
      
      //Add the trajectory to the population
      population_.add(temp);
    }
    else {
      //some error handling
    }

    //Push the times vector onto velocities_
    velocities_.push_back(v);

    //Clear times vector
    v.clear();
  } //end for


  //Update the modifier
  modifier_->updateAll(paths_, velocities_);
} //End init_population





/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
const bool Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr) {
  return h_traj_req_->request(tr); 
}

/** Request an evaluation */
const bool Planner::requestEvaluation(ramp_msgs::EvaluationRequest& er) {
  return h_eval_req_->request(er);
}



/******************************************************
 ****************** Modifying Methods *****************
 ******************************************************/


/** Modify a Path */
const std::vector<Path> Planner::modifyPath() { 
  return modifier_->perform();
}

/** Modify a trajectory */ 
const std::vector<ModifiedTrajectory> Planner::modifyTrajec() {
  std::vector<ModifiedTrajectory> result;

  //The modification operators deal with paths
  //So send the path to be modified
  std::vector<Path> mp = modifyPath();
  //std::cout<<"\nDone modifying path!\n";
  
  //Hold the ids of the path(s) modified
  std::vector<int> olds;
  olds.push_back(modifier_->i_changed1);

  //If a crossover was performed, push on the 2nd path 
  if(mp.size()>1) {
    olds.push_back(modifier_->i_changed2);
  }

  /*std::cout<<"\nBefore getNewVelocities\n";
  for(unsigned int i=0;i<olds.size();i++) {
    std::cout<<"\nolds["<<i<<"]: "<<olds.at(i);
    std::cout<<"\nPath "<<olds.at(i)<<": "<<paths_.at(olds.at(i)).toString();
    std::cout<<"\nvelocities: (";
    for(unsigned int v=0;v<velocities_.at(olds.at(i)).size();v++) {
      std::cout<<velocities_.at(olds.at(i)).at(v)<<", ";
    }
    std::cout<<")";
  }*/

  /*for(unsigned int i=0;i<mp.size();i++) 
    std::cout<<"\nmp["<<i<<"]: "<<mp.at(i).toString();*/
  
  //Hold the new velocity vector(s)
  std::vector< std::vector<float> > vs = getNewVelocities(mp, olds); 
  //std::cout<<"\nDone getNewVelocities!\n";
  
  //For each modified path,
  for(unsigned int i=0;i<mp.size();i++) {
    
    //Build a TrajectoryRequestMsg
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(mp.at(i), vs.at(i), vs.at(i));
    
    //Send the request and set the result to the returned trajectory 
    if(requestTrajectory(tr)) {
      RampTrajectory temp;
      temp.msg_trajec_ = tr.response.trajectory;

      ModifiedTrajectory mt;
      mt.trajec_ = temp;
      mt.velocities_ = vs.at(i); 

      result.push_back(mt);
    }
  }
  
  //std::cout<<"\nReturning result size:"<<result.size();
  return result;
} //End modifyTrajectory


/******************************************************
 **************** Srv Building Methods ****************
 ******************************************************/



/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<float> v_s, const std::vector<float> v_e ) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path = path.buildPathMsg();
  result.request.v_start    = v_s;
  result.request.v_end      = v_e;
  result.request.resolutionRate = resolutionRate_;

  return result;
}


/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const unsigned int i_path, const std::vector<float> v_s, const std::vector<float> v_e) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path = paths_.at(i_path).buildPathMsg();
  result.request.v_start    = v_s;
  result.request.v_end      = v_e;
  result.request.resolutionRate = resolutionRate_;
  
  return result; 
}


/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const unsigned int i_trajec, const std::vector<unsigned int> i_segments) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = population_.population_.at(i_trajec).msg_trajec_;

  for(unsigned int i=0;i<i_segments.size();i++) {
    result.request.i_segments.push_back(i_segments.at(i));
  }

  return result;
}

/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec, const std::vector<unsigned int> i_segments) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = trajec.msg_trajec_;

  for(unsigned int i=0;i<i_segments.size();i++) {
    result.request.i_segments.push_back(i_segments.at(i));
  }

  return result;
}



/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/

/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {
  h_control_->send(bestTrajec_.msg_trajec_);
}


/** This method returns the new velocity vector(s) for the modified path(s) */
const std::vector< std::vector<float> > Planner::getNewVelocities(std::vector<Path> new_paths, std::vector<int> i_old) {
  std::vector< std::vector<float> > result; 

  //If the modification only changed one path, i.e., not a crossover 
  if(new_paths.size() == 1) {
    //std::cout<<"\nOnly changed one path!\n";
    //std::cout<<"\ni_old.at(0):"<<i_old.at(0);

    //Get the original velocity values
    std::vector<float> v = velocities_.at(i_old.at(0));

    //Get the old and new paths
    Path old = paths_.at(i_old.at(0));
    Path new_path = new_paths.at(0);

    //Check the difference in path sizes
    int diff = new_path.all_.size() - old.all_.size();
    //std::cout<<"\ndiff:"<<diff<<"\n";

    //Insertion
    if(diff == 1) {
     
      //std::cout<<"\nv.size():"<<v.size()<<"\n";
      //std::cout<<"\nnew_path.all_.size():"<<new_path.all_.size();
      //Find the insertion
      for(unsigned int i=0;i<new_path.all_.size();i++) {
        if(!new_path.all_.at(i).equals(old.all_.at(i))) {
          //std::cout<<"\ni:"<<i;
          v.push_back(v.at(i-1));
          i=new_path.all_.size();
        }
      } //end for
    } //end if insertion 

    //Deletion
    else if(diff == -1) {
      
      //std::cout<<"\nv.size():"<<v.size()<<"\n";
      //std::cout<<"\nnew_path.all_.size():"<<new_path.all_.size();
      //Find the deletion
      for(unsigned int i=0;i<new_path.all_.size();i++) {
        if(!new_path.all_.at(i).equals(old.all_.at(i))) {
          //std::cout<<"\ni:"<<i;
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
      //std::cout<<"\nold path "<<i<<":"<<old.toString();
      //std::cout<<"\nnew path "<<i<<":"<<new_path.toString();

      //Get the id of the path it was crossed with 
      unsigned int i_crossed = (i==0) ? i_old.at(1) : i_old.at(0);
      //std::cout<<"\ni_crossed:"<<i_crossed<<"\n";

      //Make a Path object for it
      Path crossed_path = paths_.at(i_crossed);
      //std::cout<<"\ncrossed_path:"<<crossed_path.toString()<<"\n";

      //This will hold the velocities
      std::vector<float> v;

      //Find where the crossover occurred
      //std::cout<<"\nLooking for crossover!\n";
      unsigned int i_cross=0;
      for(unsigned int c=0;c<old.all_.size();c++) {
        //std::cout<<"\nc:"<<c<<"\n";
        
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
      //std::cout<<"\nDone looking for crossover!\n";
      
      //std::cout<<"\ni_cross:"<<i_cross;
      //std::cout<<"\ncrossed_path.all_.size()-1:"<<crossed_path.all_.size()-1;
      //std::cout<<"\nvelocities_.at("<<i_crossed<<").size():"<<velocities_.at(i_crossed).size();
      //Push all the velocity values from the crossed path (if not too small)
      for(unsigned int c=i_cross;c<crossed_path.all_.size()-1;c++) {
        //std::cout<<"\nc:"<<c<<"\n";
        v.push_back(velocities_.at(i_crossed).at(c));
      }
      
      /*for(unsigned int k=0;k<v.size();k++) {
        std::cout<<"\nv["<<k<<"]: "<<v.at(k);
      }*/

      //Push the vector of velocities onto the result
      result.push_back(v); 
    } //end for
  }
  
  //Swap and Change do not change the size

  return result;
} //End getNewVelocities



/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  and set tau to the new best, */
void Planner::modification() {
  std::vector<ModifiedTrajectory> mod_trajec = modifyTrajec();
  
  //Add the modified trajectories to the population
  //and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {

    //Index is where the trajectory was added in the population (may replace another)
    int index = population_.add(mod_trajec.at(i).trajec_);

    //Update the path and velocities in the planner and modifier
    paths_.at(index)      = mod_trajec.at(i).trajec_.getPath();
    velocities_.at(index) = mod_trajec.at(i).velocities_;
    modifier_->update(paths_.at(index), index);
  }

  //Evaluate and obtain the best trajectory
  //****we need a way to only evaluate the new trajectories****
  //evaluate mod_trajec
  evaluatePopulation();

  //Obtain best trajectory
  bestTrajec_ = population_.findBest();
}

void Planner::evaluateTrajectory(RampTrajectory& trajec, const std::vector<unsigned int> i_segments) {

  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(trajec, i_segments);
  
  //Do the evaluation and set the fitness and feasibility members
  if(requestEvaluation(er)) {
    trajec.fitness_   = er.response.fitness;
    trajec.feasible_  = er.response.feasible;
  }
  else {
    //some error handling
  }

}

void Planner::evaluatePopulation() {
  
  //i_segments is the parameter we pass if we only want to evaluate specific segments
  //in this method, it should be empty because we are evaluating the entire trajectory
  std::vector<unsigned int> i_segments;

  //Go through each trajectory in the population and evaluate it
  for(unsigned int i=0;i<population_.population_.size();i++) {
    evaluateTrajectory(population_.population_.at(i), i_segments);
  } //end for   
} //End evaluatePopulation



/** This method updates all the paths with the current configuration */
void Planner::updatePaths(Configuration start, ros::Duration dur) {
  //std::cout<<"\nUpdating start to: "<<start.toString();
  //std::cout<<"\ndur: "<<dur<<"\n";


  //For each trajectory
  for(unsigned int i=0;i<population_.population_.size();i++) {

    //Track how many knot points we get rid of
    unsigned int throwaway=0;

    //For each knot point,
    for(unsigned int i_kp=0;i_kp<population_.population_.at(i).msg_trajec_.index_knot_points.size();i_kp++) {
     
      //Get the point 
      trajectory_msgs::JointTrajectoryPoint point = population_.population_.at(i).msg_trajec_.trajectory.points.at( population_.population_.at(i).msg_trajec_.index_knot_points.at(i_kp));
      //std::cout<<"\npoint["<<i<<"].time_from_start:"<<point.time_from_start;

      //Compare the durations
      if( dur > point.time_from_start) {
        throwaway++;
      }
    }

    //If the whole path has been passed, adjust throwaway so that 
    // we are left with a path that is: {new_start_, goal_}
    if( throwaway >= paths_.at(i).size() ) { 
      throwaway = paths_.at(i).size()-1;
    } 
    
    //Erase the amount of throwaway
    paths_.at(i).all_.erase( paths_.at(i).all_.begin(), paths_.at(i).all_.begin()+throwaway );
    
    //Insert the new starting configuration
    paths_.at(i).all_.insert( paths_.at(i).all_.begin(), start_);

    //Set start_ to be the new starting configuration of the path
    paths_.at(i).start_ = start;

    //Also erase velocities, throwaway-1
    if(throwaway > 0) {
      velocities_.at(i).erase( velocities_.at(i).begin(), velocities_.at(i).begin()+throwaway-1);
    }
  } //end for
} //End updatePaths


/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
void Planner::updatePopulation(ros::Duration d) {
  
  //First, get the updated current configuration
  start_ = getStartConfiguration();
  
  //Update the paths with the new starting configuration 
  updatePaths(getStartConfiguration(), d);

  //Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  //For each path, get a trajectory
  for(unsigned int i=0;i<paths_.size();i++) {

    //Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i, velocities_.at(i), velocities_.at(i));
    
    //Send the request 
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp;
      
      //Set the Trajectory msg
      temp.msg_trajec_ = msg_request.response.trajectory;
      
      //Push onto updatedTrajecs
      updatedTrajecs.push_back(temp);
    } //end if
  } //end for

  //Replace the population's trajectories_ with the updated trajectories
  population_.replaceAll(updatedTrajecs);
} //End updatePopulation




/** This method calls evaluatePopulation and population_.getBest() */
const RampTrajectory Planner::evaluateAndObtainBest() {
  evaluatePopulation();
  return population_.findBest();
}

/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


 void Planner::go() {

  //t=0
  generation_ = 0;
  
  //initialize population
  init_population();
  
  /*std::cout<<"\nPopulation initialized!\n";
  for(unsigned int i=0;i<paths_.size();i++) {
    std::cout<<"\nPath "<<i<<": "<<paths_.at(i).toString();
  }
  std::cout<<"\nPress enter to evaluate P(0)\n";
  std::cin.get();*/

  //Initialize the modifier
  modifier_->paths_ = paths_;

  //***Adjust***
  RampTrajectory T_move = evaluateAndObtainBest();
  /*std::cout<<"\nPopulation evaluated!\n"<<population_.fitnessFeasibleToString(); 
  std::cout<<"\nPress enter to start the loop!\n";
  std::cin.get();*/
  


  //createSubpopulations();
  
  while(!start_.equals(goal_)) {
    
    //t=t+1
    generation_++;
    //std::cout<<"\ngeneration_ = "<<generation_;
    
    /*std::cout<<"\nPress Enter to start modification!\n";
    std::cin.get();*/
    //Call modification
    modification();
    /*std::cout<<"\nmodification completed!\n";
    std::cout<<"\nPaths are now: ";
    for(unsigned int i=0;i<paths_.size();i++) {
      std::cout<<"\nPath "<<i<<": "<<paths_.at(i).toString();
    }*/

    /*std::cout<<"\n\n\nPress enter to check for end of control cycle!\n";
    std::cin.get();*/
    //If end of current control cycle
    if(ros::Time::now() - lastUpdate_ >= controlCycle_) {
      //std::cout<<"\nIn end of control cycle!\n";
      /*//start_ should already be updated
      //Update starting configuration and velocity of P(t)
      std::cout<<"\nPress enter to update population!\n";
      std::cin.get();
      updatePopulation(controlCycle_);

      //Create subpopulations in P(t)
      //

      std::cout<<"\nPress enter to evaluate and get best!\n";
      std::cin.get();
      //Evaluate P(t) and obtain best T, T_move=T_best
      T_move = evaluateAndObtainBest();
      
      std::cout<<"\nPress enter to send the best trajectory!\n";
      std::cin.get();
      //T_move = T
      //Send the best trajectory 
      sendBest();*/
    }

    //If new sensing cycle...
  
    ros::spinOnce(); 
  } //end while
} //End go
