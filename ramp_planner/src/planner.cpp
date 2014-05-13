#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 5.f), populationSize_(5), generation_(0), mutex_start_(true), mutex_pop_(true), i_rt(1), goalThreshold_(0.4), num_ops_(6), D_(2.f), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0) 
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
  std::cout<<"\nid: "<<id_<<" base_pos.size(): "<<base_pos.size()<<"\n";
  for(int i=0;i<base_pos.size();i++) 
    std::cout<<"\n "<<base_pos.at(i);
 
  T_base_w_.setRotation(tf::createQuaternionFromYaw(base_pos.at(2)));
  T_base_w_.setOrigin(tf::Vector3(base_pos.at(0), base_pos.at(1), 0));
} // End setT_base_w



/** Returns an id for RampTrajectory objects */
unsigned int Planner::getIRT() { return i_rt++; }


/** Getter method for start_. It waits for mutex_start_ to be true before returning. */
MotionState Planner::getStartConfiguration() {
  while(!mutex_start_) {}
  return start_;
} // End getStartConfiguration


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
 * Sets start_ member by taking in the latest update 
 * and transforming it by T_base_w because 
 * updates are relative to odometry frame
 * */
void Planner::updateCallback(const ramp_msgs::MotionState& msg) {
  //std::cout<<"\nReceived update!\n";
  
  // Wait for mutex to be true
  while(!mutex_start_) {}

  mutex_start_ = false;

  start_ = msg;

  // Transform configuration from odometry to world coordinates
  start_.transformBase(T_base_w_);
  //std::cout<<"\nNew starting configuration: "<<start_.toString();
  

  // Set proper velocity values
  start_.velocities_.at(0) = msg.velocities.at(0) * cos(start_.positions_.at(2));
  start_.velocities_.at(1) = msg.velocities.at(0) * sin(start_.positions_.at(2));

  // Set proper acceleration values
  start_.accelerations_.at(0) = msg.accelerations.at(0) * cos(start_.positions_.at(2));
  start_.accelerations_.at(1) = msg.accelerations.at(0) * sin(start_.positions_.at(2));


  //std::cout<<"\nstart_: "<<start_.toString();
  //std::cout<<"\nstartPlanning_: "<<startPlanning_.toString();

  mutex_start_ = true;
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



void Planner::planningCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nPlanning cycle occurring, generation = "<<generation_<<"\n";
  
  // Wait until mutex can be obtained
  while(!mutex_pop_) {}
    
  mutex_pop_ = false;
  
  // Call modification
  modification();
  
  mutex_pop_ = true;
  
  //std::cout<<"\nAfter generation "<<generation_<<", population: "<<population_.fitnessFeasibleToString();
  //std::cout<<"\nBest: "<<bestTrajec_.toString();
  
  // t=t+1
  generation_++;

  //std::cout<<"\nPlanning cycle "<<generation_-1<<" complete\n";
} // End planningCycleCallback






/* This method returns a trajectory to gradually move into a new trajectory */
void Planner::gradualTrajectory(RampTrajectory& t) {

  // Find where linear velocity starts
  int i=0;
  while(t.msg_trajec_.trajectory.points.at(i).velocities.at(0) == 0 &&
        t.msg_trajec_.trajectory.points.at(i).velocities.at(1) == 0 &&
        i < t.msg_trajec_.trajectory.points.size()) {i++;}

  // If i is in the trajectory (it won't be if rotation-only) 
  if( i < t.msg_trajec_.trajectory.points.size()) {

    // Get the point
    trajectory_msgs::JointTrajectoryPoint p = t.msg_trajec_.trajectory.points.at(i);

    // Set the linear velocities to be the same as p
    for(unsigned int j=0;j<i;j++) {
      t.msg_trajec_.trajectory.points.at(j).velocities.at(0) = p.velocities.at(0);
      t.msg_trajec_.trajectory.points.at(j).velocities.at(1) = p.velocities.at(1);
    } //end for
  } //end if i in range
} // End gradualTrajectory



/** This method updates all the paths with the current configuration */
void Planner::adaptPaths(MotionState start, ros::Duration dur) {
   //std::cout<<"\nUpdating start to: "<<start.toString();
   //std::cout<<"\ndur: "<<dur<<"\n";

  if(dur.toSec() > 0) {

    // For each trajectory
    for(unsigned int i=0;i<population_.population_.size();i++) {

      // Track how many knot points we get rid of
      unsigned int throwaway=0;

      // For each knot point,
      for(unsigned int i_kp=0;i_kp<population_.population_.at(i).msg_trajec_.index_knot_points.size();i_kp++) {
       
        // Get the knot point 
        trajectory_msgs::JointTrajectoryPoint point = population_.population_.at(i).msg_trajec_.trajectory.points.at( population_.population_.at(i).msg_trajec_.index_knot_points.at(i_kp));
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
  float actual_theta = start_.positions_.at(2);
  //std::cout<<"\nactual_theta: "<<actual_theta;
  //std::cout<<"\norientations_.at("<<i<<"): "<<orientations_.at(i)<<"\n";
  
  int i2 = bestTrajec_.msg_trajec_.index_knot_points.at(1);
  float t = utility_.findAngleFromAToB(bestTrajec_.msg_trajec_.trajectory.points.at(0),
                                       bestTrajec_.msg_trajec_.trajectory.points.at(i2));

  float diff = fabs(utility_.findDistanceBetweenAngles(actual_theta, t));
  /*std::cout<<"\ndiff: "<<diff<<" actual_theta: "<<actual_theta<<" t: "<<t;
  std::cout<<"\nbestTrajec_.msg_trajec_.trajectory.points.at(0): "<<utility_.toString(bestTrajec_.msg_trajec_.trajectory.points.at(0));
  std::cout<<"\nbestTrajec_.msg_trajec_.trajectory.points.at("<<i2<<"): "<<utility_.toString(bestTrajec_.msg_trajec_.trajectory.points.at(i2))*/;
  
  return (diff > PI/12.);
}



/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
void Planner::adaptPopulation(ros::Duration d) {
  
  // First, get the updated current configuration
  //start_ = getStartConfiguration();

  // We want to plan ahead of time
  // Check if the robot will be moving ahead
  if(checkOrientation()) {
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
      RampTrajectory temp(resolutionRate_, population_.population_.at(i).id_);
      
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
} // End updatePopulation






/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nControl cycle occurring\n";
  
  // Obtain mutex on population and update it
  while(!mutex_pop_) {}
  mutex_pop_ = false;
  adaptPopulation(controlCycle_);
  mutex_pop_ = true;

  /** TODO **/
  // Create subpopulations in P(t)


  // Evaluate P(t) and obtain best T, T_move=T_best
  bestTrajec_ = evaluateAndObtainBest();


  // Find orientation difference between the old and new trajectory 
  float b = utility_.findAngleFromAToB(start_.positions_, 
                bestTrajec_.path_.all_.at(1).motionState_.positions_);
  float diff = utility_.findDistanceBetweenAngles(
                start_.positions_.at(2), b);
  

  // If difference < 30 degrees
  // Make the trajectory a gradual change
  if(fabs(diff) <= 0.52356f) {
    gradualTrajectory(bestTrajec_);
  }
  

  // Send the best trajectory 
  sendBest(); 
  
  //std::cout<<"\nControl cycle complete\n";
} // End controlCycleCallback







/** This function generates the initial population of trajectories */
void Planner::initPopulation() { 
  
  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path(start_, goal_);

    // Each trajectory will have a random number of knot points
    // Put a max of 5 knot points for practicality...
    unsigned int num = rand() % 5;
  

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
} // End init_population





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
const std::vector<ModifiedTrajectory> Planner::modifyTrajec() {
  //std::cout<<"\nIn modifyTrajec\n";
  std::vector<ModifiedTrajectory> result;
  

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

      // Build ModifiedTrajectory
      ModifiedTrajectory mt;
      mt.trajec_ = temp;

      result.push_back(mt);
  
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
  std::vector<ModifiedTrajectory> mod_trajec = modifyTrajec();
  
  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {

    // Evaluate the new trajectory
    evaluateTrajectory(mod_trajec.at(i).trajec_);
    
    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    int index = population_.add(mod_trajec.at(i).trajec_);

    // Update the path in the planner and the modifier
    updateWithModifier(index, mod_trajec.at(i).trajec_.path_);
  } // end for
  
  // Obtain and set best trajectory
  bestTrajec_ = population_.findBest();

  // Check if the best trajectory changed
  if(generation_ >= 100 && population_.checkIfChange()) {
    std::cout<<"\nBest has changed!";
  
    // Make a new Trajectory to stop the robot
    // Current state of motion
    // startPlanning position, 0 velocity
    MotionState temp;
    temp.positions_ = startPlanning_.positions_; 

    // Set v and a
    for(int i=0;i<temp.positions_.size();i++) {
      temp.velocities_.push_back(0);
      temp.accelerations_.push_back(0);
    }

    Path p(start_, temp);
    std::cout<<"\nPath p: "<<p.toString();

    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);

    h_traj_req_->request(tr);

    RampTrajectory toSend(getIRT(), resolutionRate_);
    toSend.msg_trajec_ = tr.response.trajectory;

    std::cout<<"\nSending Trajectory: "<<toSend.toString();
    h_control_->send(toSend.msg_trajec_); 

  } // end if

  
  // Send the new population to the trajectory viewer
  sendPopulation();
} // End modification



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

  result.request.trajectory = population_.population_.at(i_trajec).msg_trajec_;

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
    ramp_msgs::Trajectory blank;
    h_control_->send(blank); 
  }
  else if(!bestTrajec_.feasible_) {
    std::cout<<"\nBest trajectory is not feasible! Time until collision: "<<bestTrajec_.time_until_collision_;
    h_control_->send(bestTrajec_.msg_trajec_);
  }
  else {
    //std::cout<<"\nSending Trajectory: \n"<<bestTrajec_.toString();
    h_control_->send(bestTrajec_.msg_trajec_);
  }
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


void Planner::evaluatePopulation() {
  
  // Go through each trajectory in the population and evaluate it
  for(unsigned int i=0;i<population_.population_.size();i++) {
    evaluateTrajectory(population_.population_.at(i));
  } // end for   
} // End evaluatePopulation




/** This method calls evaluatePopulation and population_.getBest() */
const RampTrajectory Planner::evaluateAndObtainBest() {
  evaluatePopulation();
  return population_.findBest();
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

  /*std::cout<<"\nPopulation initialized!\n";
  std::cout<<"\npaths_.size(): "<<paths_.size()<<"\n";
  for(unsigned int i=0;i<paths_.size();i++) {
    std::cout<<"\nPath "<<i<<": "<<paths_.at(i).toString();
  }
  std::cout<<"\n";*/
  // std::cout<<"\nPress enter to continue\n";
  // std::cin.get();


  // Initialize the modifier
  modifier_->paths_ = paths_;


  // Evaluate the population and get the initial trajectory to move on
  bestTrajec_ = evaluateAndObtainBest();
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
  goalThreshold_ = 0.1;
  while( (start_.comparePosition(goal_, false) > goalThreshold_) && ros::ok()) {
    ros::spinOnce(); 
  } // end while
  
  std::cout<<"\nPlanning complete!";
  std::cout<<"\nLatest pose: "<<start_.toString()<<"\n";
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();

  // Send an empty trajectory
  ramp_msgs::Trajectory empty;
  h_control_->send(empty);

  //std::cout<<"\nFinal population: ";
  //std::cout<<"\n"<<pathsToString(); 
} // End go
