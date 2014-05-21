#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), populationSize_(3), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(6), D_(2.f), init_evaluation_(false), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), stop_(false) 
{
  controlCycle_ = ros::Duration(1.f / 10.f);
  planningCycle_ = ros::Duration(1.f / 25.f);
  imminentCollisionCycle_ = ros::Duration(1.f / 50.f);
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



/*****************************************************
 ********************** Methods **********************
 *****************************************************/

/** This method initializes the T_base_w_ transform object */
void Planner::setT_base_w(std::vector<float> base_pos) {
  T_base_w_.setRotation(tf::createQuaternionFromYaw(base_pos.at(2)));
  T_base_w_.setOrigin(  tf::Vector3(base_pos.at(0), base_pos.at(1), 0));
} // End setT_base_w



/** Returns an id for RampTrajectory objects */
unsigned int Planner::getIRT() { return i_rt++; }



/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) {
  //std::cout<<"\nIn imminentCollisionCycle_\n";

  if(!bestTrajec_.feasible_ && (bestTrajec_.time_until_collision_ < D_)) {
    h_parameters_.setImminentCollision(true); 
  } 
  else {
    h_parameters_.setImminentCollision(false);
  }

  //std::cout<<"\nAfter imminentCollisionCycle_\n";
}




/** 
 * Sets the latest update member
 * and transformes it by T_base_w because 
 * updates are relative to odometry frame
 * */
void Planner::updateCallback(const ramp_msgs::MotionState& msg) {
  //std::cout<<"\nReceived update!\n";

  latestUpdate_ = msg;
  
  if(msg.positions.size() < 3 ||
     msg.velocities.size() < 3 ||
     msg.accelerations.size() < 3 )
  { 
    // Log Error
  }
  if(latestUpdate_.positions_.size() < 3 ||
     latestUpdate_.velocities_.size() < 3 ||
     latestUpdate_.accelerations_.size() < 3)
  { 
    // Log Error
  }

  // Transform configuration from odometry to world coordinates
  latestUpdate_.transformBase(T_base_w_);

  // Set proper velocity values
  latestUpdate_.velocities_.at(0) = msg.velocities.at(0) * cos(latestUpdate_.positions_.at(2));
  latestUpdate_.velocities_.at(1) = msg.velocities.at(0) * sin(latestUpdate_.positions_.at(2));

  // Set proper acceleration values
  latestUpdate_.accelerations_.at(0) = msg.accelerations.at(0) * cos(latestUpdate_.positions_.at(2));
  latestUpdate_.accelerations_.at(1) = msg.accelerations.at(0) * sin(latestUpdate_.positions_.at(2));

  //std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString()<<"\n";
} // End updateCallback






/** This method sets random values for the position vector of ms
 *  ONLY RANDOMIZES POSITIONS */
const void Planner::randomizeMSPositions(MotionState& ms) const {
  ms.positions_.clear();

  for(unsigned int i=0;i<ranges_.size();i++) {
    ms.positions_.push_back(ranges_.at(i).random());
  }
} // End randomizeMotionState


/****************************************************
 ************** Initialization Methods **************
 ****************************************************/



void Planner::initStartGoal(const MotionState s, const MotionState g) {
  start_  = s;
  goal_   = g; 
}


/** Initialize the handlers and allocate them on the heap */
void Planner::init(const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r) {

  // Initialize the handlers
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_control_  = new ControlHandler(h);
  h_eval_req_ = new EvaluationRequestHandler(h);
  modifier_   = new Modifier(h, paths_, num_ops_);

  // Initialize the timers, but don't start them yet
  controlCycleTimer_ = h.createTimer(ros::Duration(controlCycle_), 
                                     &Planner::controlCycleCallback, this);
  controlCycleTimer_.stop();

  planningCycleTimer_ = h.createTimer(ros::Duration(planningCycle_), 
                                      &Planner::planningCycleCallback, this);
  planningCycleTimer_.stop();

  imminentCollisionTimer_ = h.createTimer(ros::Duration(imminentCollisionCycle_), 
                                          &Planner::imminentCollisionCallback, this);
  imminentCollisionTimer_.stop();



  // Set the ranges vector
  ranges_ = r;

  // Initialize the start and goal
  initStartGoal(s, g);

  // Set the base transformation
  setT_base_w(start_.positions_);

} // End init


void Planner::seedPopulation() {
  paths_.clear();
  population_.clear();
  
  Path p_one(start_, goal_);
  MotionState ms_one;
  
  ms_one.positions_.push_back(0.5);
  ms_one.positions_.push_back(2);
  ms_one.positions_.push_back(1.3089);  //75 degrees

  for(int i=0;i<3;i++) {
    ms_one.velocities_.push_back(0);
    ms_one.accelerations_.push_back(0);
  }

  p_one.Add(ms_one);

  Path p_two(start_, goal_);
  MotionState ms_two;
  
  ms_two.positions_.push_back(3);
  ms_two.positions_.push_back(0.5);
  ms_two.positions_.push_back(0.1624);  //9 degrees

  for(int i=0;i<3;i++) {
    ms_two.velocities_.push_back(0);
    ms_two.accelerations_.push_back(0);
  }

  p_two.Add(ms_two);

  paths_.push_back(p_one);
  paths_.push_back(p_two);
  
  // For each path
  for(unsigned int i=0;i<paths_.size();i++) {

    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i);
    
    // Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(resolutionRate_, getIRT());
      
      // Set the Trajectory msg
      temp.msg_trajec_ = msg_request.response.trajectory;

      // Set trajectory path
      temp.path_ = paths_.at(i);

      
      // Add the trajectory to the population
      population_.add(temp);
    }
    else {
      // some error handling
    }
  } // end for

  evaluatePopulation();
  std::cout<<"\nPopulation after seed: "<<population_.fitnessFeasibleToString();

  // Update the modifier
  modifier_->updateAll(paths_);
} // End seedPopulation




/** This method updates all the paths with the current configuration */
void Planner::adaptPaths(MotionState start, ros::Duration dur) {
   //std::cout<<"\nUpdating start to: "<<start.toString();
   //std::cout<<"\ndur: "<<dur<<"\n";

  if(dur.toSec() > 0) {

    // For each trajectory
    for(unsigned int i=0;i<population_.size();i++) {

      // Track how many knot points we get rid of
      unsigned int throwaway=0;

      // For each knot point,
      for(unsigned int i_kp=0;i_kp<population_.get(i).msg_trajec_.index_knot_points.size();i_kp++) {
       
        // Get the knot point 
        trajectory_msgs::JointTrajectoryPoint point = population_.get(i).msg_trajec_.trajectory.points.at( population_.get(i).msg_trajec_.index_knot_points.at(i_kp));
        // std::cout<<"\npoint["<<i<<"].time_from_start:"<<point.time_from_start;

        // Compare the durations
        if( dur > point.time_from_start) {
          throwaway++;
        }
        else {
          break;
        }
      } // end inner for

      //std::cout<<"\nthrowaway: "<<throwaway;

      // If the whole path has been passed, adjust throwaway so that 
      //  we are left with a path that is: {new_start_, goal_}
      if( throwaway >= paths_.at(i).size() ) { 
        throwaway = paths_.at(i).size()-1;
      } 
      
      // Erase the amount of throwaway points (points we have already passed)
      paths_.at(i).all_.erase( paths_.at(i).all_.begin(), paths_.at(i).all_.begin()+throwaway );
      
      // Insert the new starting configuration
      paths_.at(i).all_.insert( paths_.at(i).all_.begin(), start);

      // Set start_ to be the new starting configuration of the path
      paths_.at(i).start_ = start;
    } // end outer for

    // Update the modifier's paths
    modifier_->updateAll(paths_);

  } // end if
} // End updatePaths





const bool Planner::checkOrientation() const {
  //std::cout<<"\nEntering checkOrientation\n";
  
  float actual_theta = start_.positions_.at(2);
  //std::cout<<"\nactual_theta: "<<actual_theta;
  //std::cout<<"\norientations_.at("<<i<<"): "<<orientations_.at(i)<<"\n";
  
  int i2 = bestTrajec_.msg_trajec_.index_knot_points.at(1);
  float t = utility_.findAngleFromAToB(bestTrajec_.msg_trajec_.trajectory.points.at(0),
                                       bestTrajec_.msg_trajec_.trajectory.points.at(i2));

  if(t == 0 && utility_.getEuclideanDist(bestTrajec_.msg_trajec_.trajectory.points.at(0).positions, 
        bestTrajec_.msg_trajec_.trajectory.points.at(i2).positions) < 0.0001f) 
  {
    return true;
  }

  float diff = fabs(utility_.findDistanceBetweenAngles(actual_theta, t));
  //std::cout<<"\ndiff: "<<diff<<" actual_theta: "<<actual_theta<<" t: "<<t;

  return (diff <= PI/36.f);
}



/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
void Planner::adaptPopulation(ros::Duration d) {
  
  /** First, get the updated current configuration */
  // We want to plan ahead of time
  // Check if the robot will be moving ahead
  // if not satisfied, the robot won't be moving
  if(!checkOrientation()) {
    startPlanning_ = start_;
    d = ros::Duration(0);
  }
  else {
    MotionState ms(bestTrajec_.getPointAtTime(controlCycle_.toSec()));
    startPlanning_ = ms;
  }

  
  // Update the paths with the new starting configuration 
  adaptPaths(startPlanning_, d*2);

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  // For each path, get a trajectory
  for(unsigned int i=0;i<paths_.size();i++) {

    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i);
    
    // Send the request 
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(resolutionRate_, population_.get(i).id_);
      
      // Set the Trajectory msg and the path
      temp.msg_trajec_  = msg_request.response.trajectory;
      temp.path_        = paths_.at(i);
      
      // Push onto updatedTrajecs
      updatedTrajecs.push_back(temp);
    } // end if
  } // end for

  // Replace the population's trajectories_ with the updated trajectories
  population_.replaceAll(updatedTrajecs);

  //std::cout<<"\nLeaving updatePopulation\n";
} // End adaptPopulation







/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nControl cycle occurring\n";
  
  
  /** TODO **/
  // Create subpopulations in P(t)


  if(!stop_) {
    
    // Update starting state of motion
    start_ = latestUpdate_;

    // Update the population 
    adaptPopulation(controlCycle_);


    // Evaluate P(t) and obtain best T, T_move=T_best
    bestTrajec_ = evaluateAndObtainBest();



    // Send the best trajectory 
    sendBest();
  } 
  
  //std::cout<<"\nControl cycle complete\n";
} // End controlCycleCallback







/** 
 * This function generates the initial population of trajectories,
 *  sets the paths in the Modifier class
 *  and evaluates the population
 **/
void Planner::initPopulation() { 
  
  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path(start_, goal_);

    // Each trajectory will have a random number of knot points
    // Put a max of 5 knot points for practicality...
    //unsigned int num = rand() % 5;
    unsigned int num = 3;
  

    // For each knot point to be created
    for(unsigned int j=0;j<num;j++) {

      // Create a random configuration
      MotionState temp_ms;
      randomizeMSPositions(temp_ms);
      
      // Push on velocity values (for target v when getting trajectory)
      // //TODO: Not 0 for all knot points
      for(unsigned int i=0;i<temp_ms.positions_.size();i++) {
        temp_ms.velocities_.push_back(0);
      }
      
      // Add the random configuration to the path
      temp_path.Add(temp_ms); 
    }

    // Add the path to the list of paths
    paths_.push_back(temp_path);
  } // end for create n paths
  
  // For each path
  for(unsigned int i=0;i<paths_.size();i++) {

    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(i);
    
    // Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(resolutionRate_, getIRT());
      
      // Set the Trajectory msg
      temp.msg_trajec_ = msg_request.response.trajectory;

      // Set trajectory path
      temp.path_ = paths_.at(i);
      
      // Add the trajectory to the population
      population_.add(temp);
    }
    else {
      // some error handling
    }
  } // end for


  // Update the modifier
  modifier_->updateAll(paths_);

  // Evaluate the population 
  bestTrajec_ = evaluateAndObtainBest();
} // End init_population




const RampTrajectory Planner::getChangingTrajectory() const {
    // Make a new Trajectory to stop the robot
    // Current state of motion
    // startPlanning position, 0 velocity
    MotionState ms = startPlanning_;

    /*std::cout<<"\nms: "<<ms.toString();
    std::cout<<"\nstart: "<<start_.toString();
    std::cout<<"\nstartPlanning_: "<<startPlanning_.toString();
    std::cout<<"\nlatestUpdate_: "<<latestUpdate_.toString();*/
    
    // Set v and a
    for(int i=0;i<ms.positions_.size();i++) {
      ms.velocities_.at(i) = 0;
      ms.accelerations_.at(i) = 0;
    }

    // Should we use latestUpdate_?
    Path p(start_, ms);
    //std::cout<<"\nPath p: "<<p.toString();

    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);

    h_traj_req_->request(tr);

    RampTrajectory toSend;
    toSend.msg_trajec_ = tr.response.trajectory;


    return toSend;
} 




/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
bool Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr) {
  return h_traj_req_->request(tr); 
}

/** Request an evaluation */
bool Planner::requestEvaluation(ramp_msgs::EvaluationRequest& er) {
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
const std::vector<RampTrajectory> Planner::modifyTrajec() {
  //std::cout<<"\nIn modifyTrajec\n";
  std::vector<RampTrajectory> result;
  

  // The process begins by modifying one or more paths
  std::vector<Path> modded_paths = modifyPath();
  //std::cout<<"\nNumber of modified paths returned: "<<modded_paths.size()<<"\n";


  // For each targeted path,
  for(unsigned int i=0;i<modded_paths.size();i++) {
    
    // Build a TrajectoryRequestMsg
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(modded_paths.at(i));

    // Send the request and set the result to the returned trajectory 
    if(requestTrajectory(tr)) {
  
      // Build RampTrajectory
      RampTrajectory temp(resolutionRate_, getIRT());
      temp.msg_trajec_ = tr.response.trajectory;
      temp.path_ = modded_paths.at(i);

      result.push_back(temp);
  
    } // end if
  } // end for
  
  return result;
} // End modifyTrajectory




/** 
 * This method updates the paths vector for the planner
 * and the modifier 
 **/
void Planner::updateWithModifier(const int index, const Path path) {

    // Update the path in the planner 
    paths_.at(index) = path;
    
    // Update the path in the modifier 
    modifier_->update(paths_.at(index), index);
} // End updateWithModifier




/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  and set tau to the new best */
void Planner::modification() {

  // Modify 1 or more trajectories
  std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  
  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {

    // Evaluate the new trajectory
    evaluateTrajectory(mod_trajec.at(i));
    
    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    int index = population_.add(mod_trajec.at(i));

    // Update the path in the planner and the modifier
    updateWithModifier(index, mod_trajec.at(i).path_);
  } // end for*/
  
  // After adding new trajectories to population,
  // get the best trajectory
  int index = population_.findBest();
  bestTrajec_ = population_.get(index);


  // If the best trajectory has changed and the control cycles have started
  if(index != i_best_prev_ && startPlanning_.positions_.size() > 0) {
  
    // Set index of previous best
    i_best_prev_ = index;
  
    RampTrajectory toSend = getChangingTrajectory();

    //std::cout<<"\nSending Trajectory: "<<toSend.toString();
    h_control_->send(toSend.msg_trajec_); 

     // Make population only contain the preparation trajectory 
    /*Population temp_pop = population_;
    population_.clear();
    population_.add(toSend);
    sendPopulation();
    
    stop_=true;

    // Make an empty trajectory to stop the robot
    ramp_msgs::Trajectory empty;
    h_control_->send(empty);

    std::cout<<"\nPress Enter to continue execution\n";
    std::cin.get();
    stop_=false;

    population_ = temp_pop;*/
  } // end if

  
  // Send the new population to the trajectory viewer
  sendPopulation();
} // End modification



void Planner::planningCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nPlanning cycle occurring, generation = "<<generation_<<"\n";
  
  //std::cout<<"\nAfter generation "<<generation_<<", population: "<<population_.fitnessFeasibleToString();
  //std::cout<<"\nBest: "<<bestTrajec_.toString();


  // At generation 250, create a straight-line trajectory
  // and add it to the population 
  /*if(generation_ == 200) {
    std::cout<<"\nstartPlanning at time of creating new trajectory: "<<startPlanning_.toString();
    MotionState s = startPlanning_;
    for(int i=0;i<s.velocities_.size();i++) {
      s.velocities_.at(i) = 0;
    }
    
    Path p(startPlanning_, goal_); 
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);

    h_traj_req_->request(tr);

    RampTrajectory toAdd(resolutionRate_, getIRT());
    toAdd.msg_trajec_ = tr.response.trajectory;
    toAdd.path_ = p;

    int index = population_.add(toAdd);
    //std::cout<<"\nAdding new trajectory: "<<toAdd.toString();

   // std::cout<<"\nPopulation: "<<population_.fitnessFeasibleToString();

    evaluateTrajectory(toAdd);
    //sendPopulation();
    updateWithModifier(index, toAdd.path_);
    
  }*/

  
  //std::cout<<"\nBefore modification\n";
  // Call modification
  modification();
  //std::cout<<"\nAfter modification\n";
  
  // t=t+1
  generation_++;
  
  if( (generation_-1) % 50 == 0) {
    std::cout<<"\nPlanning cycle "<<generation_-1<<" complete\n";
  }
} // End planningCycleCallback






/******************************************************
 **************** Srv Building Methods ****************
 ******************************************************/



/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = path.buildPathMsg();
  result.request.resolutionRate = resolutionRate_;

  return result;
} // End buildTrajectoryRequest


/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const unsigned int i_path) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = paths_.at(i_path).buildPathMsg();
  result.request.resolutionRate = resolutionRate_;

  return result; 
} // End buildTrajectoryRequest


/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const unsigned int i_trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = population_.get(i_trajec).msg_trajec_;

  return result;
} // End buildEvaluationRequest

/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = trajec.msg_trajec_;

  return result;
} // End buildEvaluationRequest





/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/




/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {


  // If infeasible and too close to obstacle, 
  // Stop the robot by sending a blank trajectory
  if(!bestTrajec_.feasible_ && (bestTrajec_.time_until_collision_ < 2.f)) {
    std::cout<<"\nCollision within 2 seconds! Stopping robot!\n";
  }
  else if(!bestTrajec_.feasible_) {
    std::cout<<"\nBest trajectory is not feasible! Time until collision: "<<bestTrajec_.time_until_collision_;
  }
  
  h_control_->send(bestTrajec_.msg_trajec_);
} // End sendBest







/** Send the whole population of trajectories to the trajectory viewer */
void Planner::sendPopulation() {

  // Need to set robot id
  ramp_msgs::Population msg = population_.populationMsg();
  msg.robot_id = id_;

  h_control_->sendPopulation(msg);
}




/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
void Planner::evaluateTrajectory(RampTrajectory& trajec) {

  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(trajec);
  
  // Do the evaluation and set the fitness and feasibility members
  if(requestEvaluation(er)) {
    trajec.fitness_   = er.response.fitness;
    trajec.feasible_  = er.response.feasible;
    trajec.msg_trajec_.fitness    = trajec.fitness_;
    trajec.msg_trajec_.feasible   = trajec.feasible_;
    trajec.time_until_collision_  = er.response.time_until_collision;
  }
  else {
    // TODO: some error handling
  }
} // End evaluateTrajectory



/** 
 * This method evaluates each trajectory in the population
 * It also sete i_best_prev_
 **/
void Planner::evaluatePopulation() {
  
  // Go through each trajectory in the population and evaluate it
  for(unsigned int i=0;i<population_.size();i++) {
    evaluateTrajectory(population_.get(i));
  } // end for   

  i_best_prev_ = population_.findBest();
} // End evaluatePopulation




/** This method calls evaluatePopulation and population_.getBest() */
const RampTrajectory Planner::evaluateAndObtainBest() {
  // Evaluate population
  evaluatePopulation();
  
  // Find the best trajectory
  int index = population_.findBest();
  i_best_prev_ = index;

  // Return the trajectory
  return population_.get(index);
}


const std::string Planner::pathsToString() const {
  std::ostringstream result;

  result<<"\nPaths:";
  for(unsigned int i=0;i<paths_.size();i++) {
    result<<"\n  "<<paths_.at(i).toString();
  }
  result<<"\n";
  return result.str();
}





/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


 void Planner::go() {

  // t=0
  generation_ = 0;
  
  // initialize population
  initPopulation();
  //seedPopulation();

  /*std::cout<<"\nPopulation initialized!\n";
  std::cout<<"\npaths_.size(): "<<paths_.size()<<"\n";
  for(unsigned int i=0;i<paths_.size();i++) {
    std::cout<<"\nPath "<<i<<": "<<paths_.at(i).toString();
  }
  std::cout<<"\n";*/
  // std::cout<<"\nPress enter to continue\n";
  // std::cin.get();

  sendPopulation();
  //std::cout<<"\nPopulation evaluated!\n"<<population_.fitnessFeasibleToString()<<"\n\n"; 
  // std::cout<<"\nPress enter to start the loop!\n";
  // std::cin.get();
  

  /** TODO */
  // createSubpopulations();
  
  // Start the planning cycle timer
  planningCycleTimer_.start();

  // Wait for 100 generations before starting 
  while(generation_ < 100) {ros::spinOnce();}

  // Start the control cycle timer
  std::cout<<"\n********Robot "<<id_<<": Starting Control Cycle********\n";
  controlCycleTimer_.start();
  imminentCollisionTimer_.start();
  
  // Do planning until robot has reached goal
  // D = 0.4 if considering mobile base, 0.2 otherwise
  goalThreshold_ = 0.15;
  while( (latestUpdate_.comparePosition(goal_, false) > goalThreshold_) && ros::ok()) {
    ros::spinOnce(); 
  } // end while
  std::cout<<"\nOutside while\n";
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();

  
  // Send an empty trajectory
  ramp_msgs::Trajectory empty;
  h_control_->send(empty);
  
  std::cout<<"\nPlanning complete!";
  std::cout<<"\nLatest update: "<<latestUpdate_.toString();
  std::cout<<"\nLatest start: "<<start_.toString();
  std::cout<<"\nLatest startPlanning_: "<<startPlanning_.toString();
  //std::cout<<"\nbestTrajec: "<<bestTrajec_.toString()<<"\n";
  

  //std::cout<<"\nFinal population: ";
  //std::cout<<"\n"<<pathsToString(); 
} // End go
