#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), populationSize_(6), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(5), D_(2.f), generationsBeforeCC_(30), cc_started_(false), subPopulations_(false), c_pc_(0), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), stop_(false) 
{
  controlCycle_ = ros::Duration(1.f / 2.f);
  planningCycle_ = ros::Duration(1.f / 20.f);
  imminentCollisionCycle_ = ros::Duration(1.f / 25.f);
  generationsPerCC_ = controlCycle_.toSec() / planningCycle_.toSec();
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
void Planner::setT_base_w(std::vector<double> base_pos) {
  T_base_w_.setRotation(tf::createQuaternionFromYaw(base_pos.at(2)));
  T_base_w_.setOrigin(  tf::Vector3(base_pos.at(0), base_pos.at(1), 0));
} // End setT_base_w



/** Returns an id for RampTrajectory objects */
unsigned int Planner::getIRT() { return i_rt++; }




/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) {
  //std::cout<<"\nIn imminentCollisionCycle_\n";

  if(!bestTrajec_.msg_.feasible && (bestTrajec_.timeUntilCollision_ < D_)) {
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

  m_cc_ = start_;
  startPlanning_ = start_;

  latestUpdate_ = start_;
}


/** Initialize the handlers and allocate them on the heap */
void Planner::init(const uint8_t i, const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r, const int population_size, const bool sub_populations) {

  // Set ID
  id_ = i;

  // Initialize the handlers
  h_traj_req_ = new TrajectoryRequestHandler(h);
  h_control_  = new ControlHandler(h);
  h_eval_req_ = new EvaluationRequestHandler(h);
  modifier_   = new Modifier(h, num_ops_);

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

  // Set misc members
  populationSize_ = population_size;
  subPopulations_ = sub_populations;
} // End init






/** Place code to seed population here */
void Planner::seedPopulation() {

  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(0.5);
  kp.motionState.positions.push_back(3);
  kp.motionState.positions.push_back(1.396263); // 80 degrees 
  
  kp.motionState.velocities.push_back(0);
  kp.motionState.velocities.push_back(0);
  kp.motionState.velocities.push_back(0);
  
  ramp_msgs::KnotPoint kp1;
  
  kp1.motionState.positions.push_back(2.);
  kp1.motionState.positions.push_back(0.);
  kp1.motionState.positions.push_back(PI/4);
  
  kp1.motionState.velocities.push_back(0);
  kp1.motionState.velocities.push_back(0);
  kp1.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all;
  all.push_back(start_);
  all.push_back(kp);
  all.push_back(kp1);
  all.push_back(goal_);

  Path p1(all);

  /*************************/

  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(0.5);
  kp2.motionState.positions.push_back(2);
  kp2.motionState.positions.push_back(PI/4);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  
  ramp_msgs::KnotPoint kp3;
  
  kp3.motionState.positions.push_back(1);
  kp3.motionState.positions.push_back(1);
  kp3.motionState.positions.push_back(PI/4);
  
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(start_);
  all2.push_back(kp2);
  all2.push_back(kp3);
  all2.push_back(goal_);

  Path p2(all2);
  /****************************/

  /**** Create the vector of Paths ****/

  std::vector<Path> paths;
  paths.push_back(p1);
  paths.push_back(p1);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(paths.at(i));
    
    // Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(resolutionRate_, getIRT());
      
      // Set the Trajectory msg
      temp.msg_ = msg_request.response.trajectory;

      // Set trajectory path
      temp.path_        = paths.at(i);
      temp.bezierPath_  = msg_request.response.newPath;

      // Evaluate and add the trajectory to the population
      new_pop.push_back(evaluateTrajectory(temp));
    }
  } // end for
  /************************************/

  population_.replaceAll(new_pop);  
} // End seedPopulation



/** Will seed population with a straight-line trajectory to the goal */
void Planner::seedPopulationLine() {

  //Path p_one(startPlanning_, goal_);
  
  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(0.5);
  kp2.motionState.positions.push_back(2);
  kp2.motionState.positions.push_back(PI/4);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  
  ramp_msgs::KnotPoint kp3;
  
  kp3.motionState.positions.push_back(1);
  kp3.motionState.positions.push_back(1);
  kp3.motionState.positions.push_back(PI/4);
  
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);
  kp3.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(start_);
  all2.push_back(kp2);
  all2.push_back(kp3);
  all2.push_back(goal_);

  Path p2(all2);

  ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(p2);
  //ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(p_one);
  
  // Send the request and push the returned Trajectory onto population_
  if(requestTrajectory(msg_request)) {
    RampTrajectory temp(resolutionRate_, getIRT());
    
    // Set the Trajectory msg
    temp.msg_ = msg_request.response.trajectory;

    // Set trajectory path
    temp.path_        = msg_request.request.path;
    temp.bezierPath_  = msg_request.response.newPath;

    // Evaluati and add the trajectory to the population
    population_.replace(1, evaluateTrajectory(temp));
  }
} // End seedPopulationLine





/** 
 * This method updates all the paths with the current configuration 
 * For each knot point in a path, it will remove the knot point if
 * its time_from_start is <= the Duration argument
 * */
void Planner::adaptPaths(MotionState start, ros::Duration dur) {
   //std::cout<<"\nUpdating start to: "<<start.toString();
   //std::cout<<"\ndur: "<<dur<<"\n";

  if(dur.toSec() > 0) {

    // For each trajectory
    for(unsigned int i=0;i<population_.size();i++) {

      // Track how many knot points we get rid of
      // Set to 1 to always remove starting position
      unsigned int throwaway=1;

      // For each knot point,
      for(unsigned int i_kp=1;i_kp<population_.get(i).msg_.i_knotPoints.size();i_kp++) {
       
        // Get the knot point 
        trajectory_msgs::JointTrajectoryPoint point = population_.get(i).msg_.trajectory.points.at( population_.get(i).msg_.i_knotPoints.at(i_kp));
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
      if( throwaway >= population_.paths_.at(i).size() ) { 
        throwaway = population_.paths_.at(i).size()-1;
      } 
      
      // Erase the amount of throwaway points (points we have already passed)
      population_.paths_.at(i).all_.erase( population_.paths_.at(i).all_.begin(), population_.paths_.at(i).all_.begin()+throwaway );
      
      // Insert the new starting configuration
      population_.paths_.at(i).all_.insert( population_.paths_.at(i).all_.begin(), start);

      // Set start_ to be the new starting configuration of the path
      population_.paths_.at(i).start_ = start;
    } // end outer for


  } // end if
} // End adaptPaths




/** This method returns true if the robot has orientation to move on the best trajectory */
const bool Planner::checkOrientation() const {
  //std::cout<<"\nEntering checkOrientation\n";
  
  float actual_theta = start_.positions_.at(2);
  //std::cout<<"\nactual_theta: "<<actual_theta;
  //std::cout<<"\norientations_.at("<<i<<"): "<<orientations_.at(i)<<"\n";
  
  int i2 = bestTrajec_.msg_.i_knotPoints.at(1);
  float t = utility_.findAngleFromAToB(bestTrajec_.msg_.trajectory.points.at(0),
                                       bestTrajec_.msg_.trajectory.points.at(i2));

  if(t == 0 && utility_.getEuclideanDist(bestTrajec_.msg_.trajectory.points.at(0).positions, 
        bestTrajec_.msg_.trajectory.points.at(i2).positions) < 0.0001f) 
  {
    return true;
  }

  float diff = fabs(utility_.findDistanceBetweenAngles(actual_theta, t));

  return (diff <= PI/12.f);
} // End checkOrientation




/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
void Planner::adaptPopulation(ros::Duration d) {
  

  
  // ***** TODO: PREDICT DURATION *****
  // Update the paths with the new starting configuration
  adaptPaths(m_cc_, d);

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  // For each path, get a trajectory
  for(unsigned int i=0;i<population_.paths_.size();i++) {

    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(population_.paths_.at(i));
    
    // Send the request 
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(resolutionRate_, population_.get(i).id_);
      
      // Set the Trajectory msg and the path
      temp.msg_  = msg_request.response.trajectory;
      temp.path_        = population_.paths_.at(i);
      temp.bezierPath_  = msg_request.response.newPath;
      
      // Push onto updatedTrajecs
      updatedTrajecs.push_back(temp);
    } // end if
  } // end for

  // Replace the population's trajectories_ with the updated trajectories
  population_.replaceAll(updatedTrajecs);

  //std::cout<<"\nLeaving updatePopulation\n";
} // End adaptPopulation





void Planner::setMi() {

  // Clear m_i
  m_i.clear();
  
  // Need to set m_delta
  // motion difference from previous CC to next CC
  MotionState delta_m = m_cc_.subtract(start_);
  //std::cout<<"\nDelta_m: "<<delta_m.toString();

  // Divide delta_m by num_pc to get the motion difference for each PC
  MotionState delta_m_inc = delta_m.divide(generationsPerCC_);
  //std::cout<<"\nDelta_m / num_pc: "<<delta_m_inc.toString();
  
  // Set m_i
  // Each m_i will be start + (delta_m_inc * i)
  for(int i=0;i<generationsPerCC_;i++) {
    MotionState temp = delta_m_inc.multiply(i+1);
    MotionState m = start_.add(temp);

    m_i.push_back(m);

    //std::cout<<"\n\nPC: "<<i<<": Pushing on "<<m.toString();
  }
}




const std::vector<RampTrajectory> Planner::getTrajectories(const std::vector<Path> p) {
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<p.size();i++) {

    // Build a TrajectoryRequest 
    ramp_msgs::TrajectoryRequest msg_request = buildTrajectoryRequest(p.at(i));
    
    // Send the request and push the returned Trajectory onto population_
    if(requestTrajectory(msg_request)) {
      RampTrajectory temp(resolutionRate_, getIRT());
      
      // Set the Trajectory msg
      temp.msg_ = msg_request.response.trajectory;

      // Set trajectory paths 
      temp.path_       = p.at(i);
      temp.bezierPath_ = msg_request.response.newPath;
      
      // Add the trajectory to the population
      result.push_back(temp);
    }
    else {
      // some error handling
    }
  } // end for

  return result;
}






/** 
 * This function generates the initial population of trajectories,
 *  sets the paths in the Modifier class
 *  and evaluates the population
 **/
void Planner::initPopulation() { 
  
  // Set the max size
  population_.maxSize_ = populationSize_;
  
  std::vector<Path> randomPaths;
  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path(start_, goal_);

    // Each trajectory will have a random number of knot points
    // Put a max of 3 knot points for practicality...
    unsigned int num = rand() % 3;
  

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
    randomPaths.push_back(temp_path);
  } // end for create n paths
  

  // Get trajectories for the paths
  std::vector<RampTrajectory> trajecs = getTrajectories(randomPaths);

  // Add each trajectory to the population
  for(unsigned int i=0;i<trajecs.size();i++) {
    population_.add(trajecs.at(i));
  }

  // Evaluate the population 
  bestTrajec_ = evaluateAndObtainBest();
} // End init_population






// **** TODO: Adapt population before continuing ****
const RampTrajectory Planner::getTransitionTrajectory(const RampTrajectory trgt_traj) {
  std::cout<<"\nIn getTransitionTrajectory\n";
  h_parameters_.setImminentCollision(true); 

  std::vector<MotionState> segment_points;
  segment_points.push_back(start_);
  segment_points.push_back(startPlanning_);
  // 2nd knot point should be the initial point on that trajectory's bezier 
  segment_points.push_back(bestTrajec_.bezierPath_.all_.at(1).motionState_);

  std::cout<<"\nstart: "<<start_.toString();
  std::cout<<"\nstartPlaning: "<<startPlanning_.toString();
  //std::cout<<"\nbestTrajec: "<<bestTrajec_.toString();


  Path p(segment_points);
  std::cout<<"\nPath to change trajectories: "<<p.toString();

  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  tr.request.type = TRANSITION;
  tr.request.print = false;

  h_traj_req_->request(tr);

  RampTrajectory transition;
  transition.msg_  = tr.response.trajectory;
  transition.path_ = tr.response.newPath;

  
  std::cout<<"\nTrajectory to change: "<<transition.toString()<<"\n";

  std::cout<<"\nFirst point of target curve: ";
  std::cout<<"\n"<<bestTrajec_.path_.all_.at(1).toString()<<"\n";


  return transition;
}


const RampTrajectory Planner::getTrajectoryWithCurve(const RampTrajectory trgt_traj) {
  RampTrajectory result = getTransitionTrajectory(trgt_traj);

  // Set the cycle time and latest point's time
  ros::Duration t_cycle   = result.msg_.trajectory.points.at(1).time_from_start - 
                            result.msg_.trajectory.points.at(0).time_from_start;
  ros::Duration t_latest  = result.msg_.trajectory.points.at(
                            result.msg_.trajectory.points.size()-1).time_from_start 
                            + t_cycle;

  // Keep a counter for the knot points
  int c_kp=2;

  // Start at the bezier curve in trgt_traj and push on the rest of the trajectory to result
  // todo: i should be + 1
  for(uint16_t i=trgt_traj.msg_.i_knotPoints.at(1); i<trgt_traj.msg_.trajectory.points.size(); i++) {
    trajectory_msgs::JointTrajectoryPoint temp = trgt_traj.msg_.trajectory.points.at(i);

    // Set proper time
    temp.time_from_start = t_latest;
    t_latest += t_cycle;
    
    // Push on new point
    result.msg_.trajectory.points.push_back( temp );
   
    // If knot point, push on the index
    if( i == trgt_traj.msg_.i_knotPoints.at(c_kp) ) {
      result.msg_.i_knotPoints.push_back(result.msg_.trajectory.points.size()-1);
      c_kp++;
    }
  } // end for

  return result;
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
  return modifier_->perform(population_);
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
      temp.msg_ = tr.response.trajectory;
      temp.path_ = modded_paths.at(i);
      temp.bezierPath_ = tr.response.newPath;

      result.push_back(temp);
  
    } // end if
  } // end for
  
  return result;
} // End modifyTrajectory







/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  set the new best trajectory,
 *  and return the index of the new best trajectory */
void Planner::modification() {

  // Modify 1 or more trajectories
  /*std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  

  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {

    // Evaluate the new trajectory
    mod_trajec.at(i) = evaluateTrajectory(mod_trajec.at(i));

    
    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    int index = population_.add(mod_trajec.at(i));

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) {
       population_.createSubPopulations();
    }
  } // end for*/

    
  // Evaluate entire population
  int index = population_.getBestID();
  bestTrajec_ = population_.get(index);
  
  // If the best trajectory has changed and the control cycles have started
  if(index != i_best_prev_ && cc_started_) {
    std::cout<<"\nBest trajectory now index: "<<index;
  
    // Set index of previous best
    i_best_prev_ = index;
  
    // Get the transition trajectory
    //RampTrajectory transition = getTransitionTrajectory(population_.get(index));

    /** Test Trajectory **/
    /*ramp_msgs::Population pop;
    pop.population.push_back(transition.msg_);
    h_control_->sendPopulation(pop);

    std::cout<<"\nTransition trajectory published, Press Enter to move on transition trajectory\n";
    std::cin.get();

    h_parameters_.setImminentCollision(false); 
    h_control_->send(transition.msg_);

    controlCycleTimer_.stop();
    planningCycleTimer_.stop();
    imminentCollisionTimer_.stop();
 
    std::cout<<"\nPress Enter to continue with planner\n";
    std::cin.get();


    controlCycleTimer_.start();
    planningCycleTimer_.start();
    imminentCollisionTimer_.start();*/
    /*************************/


    /*std::cout<<"\nSending Trajectory: \n";
    h_control_->send(transition.msg_); 
    std::cout<<"\nAfter Sending Trajectory:\n";

    population_.replace(index, transition);*/
  } // end if

} // End modification






void Planner::stopForDebugging() {

  h_parameters_.setImminentCollision(true); 

  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();
}

void Planner::restartAfterDebugging() {
  h_parameters_.setImminentCollision(false); 

  controlCycleTimer_.start();
  planningCycleTimer_.start();
  imminentCollisionTimer_.start();
}





const MotionState Planner::predictStartPlanning() const {
  //std::cout<<"\nIn predictStartPlanning\n";
  MotionState result;

  // If the orientation is not satisfied, 
  if(!checkOrientation()) {
    result = start_;
    //d = ros::Duration(0);
  }
  else {
    //std::cout<<"\nc_pc: "<<c_pc_<<" m_i.size(): "<<m_i.size()<<" latestUpdate_.size(): "<<latestUpdate_.positions_.size()<<"\n";
    
    // Get the difference between robot's state and what state it should be at
    MotionState diff = m_i.at(c_pc_).subtract(latestUpdate_);
    //std::cout<<"\ndiff: "<<diff.toString();

    // Subtract that difference from startPlanning
    result = m_cc_.subtract(diff);

  }

  //std::cout<<"\nLeaving predictStartPlanning\n";
  return result;
}



/** 
 * This method will replace the starting motion state of each path
 * with s and will update the modifier's paths 
 * */
void Planner::updatePathsStart(const MotionState s) {
  //std::cout<<"\nIn updatePathsStart\n";

  for(unsigned int i=0;i<population_.paths_.size();i++) {
    population_.paths_.at(i).start_ = s;

    population_.paths_.at(i).all_.erase (population_.paths_.at(i).all_.begin());
    population_.paths_.at(i).all_.insert(population_.paths_.at(i).all_.begin(), s);
  } 

  //std::cout<<"\nLeaving updatePathsStart\n";
} // End updatePathsStart



void Planner::planningCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nPlanning cycle occurring, generation = "<<generation_<<"\n";
  
  //std::cout<<"\nAfter generation "<<generation_<<", population: "<<population_.fitnessFeasibleToString();
  //std::cout<<"\nBest: "<<bestTrajec_.toString();


  // At generation x, insert a straight-line trajectory to the goal
  //if(generation_ == 50) {
    //seedPopulationLine();
    //std::cout<<"\nPop: "<<population_.fitnessFeasibleToString();
  //}




  // Make sure not too many PC occur before next CC
  if(c_pc_ < generationsPerCC_ || !cc_started_) {

    /*if(cc_started_) {
      // Update startPlanning
      startPlanning_ = predictStartPlanning();
      //std::cout<<"\nAfter predicting startPlanning_:";
      //std::cout<<"\nstartPlanning: "<<startPlanning_.toString()<<"\n";
      

      // Generate new trajectories
      // Update paths with startPlanning
      updatePathsStart(startPlanning_);
      
      std::vector<RampTrajectory> trajecs = getTrajectories(population_.paths_);
      evaluatePopulation();

      //std::cout<<"\ntrajecs.size(): "<<trajecs.size()<<"\n";
      population_.replaceAll(trajecs);
    }*/


    
    //std::cout<<"\nBefore modification\n";
    // Call modification
    //modification();
    //std::cout<<"\nAfter modification\n";


    bestTrajec_ = evaluateAndObtainBest();

    
    // t=t+1
    generation_++;
    
    if( (generation_-1) % 10 == 0) {
      std::cout<<"\nPlanning cycle "<<generation_-1<<" complete\n";
    }

    c_pc_++;
  
    // Send the new population to the trajectory viewer
    //sendPopulation();
  
    //std::cout<<"\nGeneration "<<generation_-1<<" Completed";
  } // end if c_pc<genPerCC
} // End planningCycleCallback





/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\nControl cycle occurring\n";
  

  if(!stop_) {
    //std::cout<<"\nstartPlanning: "<<startPlanning_.toString();
    //std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString()<<"\n";
    //****SP_LU_diffs_.push_back(startPlanning_.subtract(latestUpdate_));
    
    // Update start
    start_ = latestUpdate_;

    // Reset planning cycle count
    c_pc_ = 0;

    // Send the best trajectory 
    sendBest();

    // Set m_cc_ and startPlanning
    // The motion state that we should reach by the next control cycle
    m_cc_ = bestTrajec_.getPointAtTime(controlCycle_.toSec());
    startPlanning_ = m_cc_;
    
    // After m_cc_ and startPlanning are set, update the population
    adaptPopulation(controlCycle_);
    //evaluatePopulation();

    
    if(subPopulations_) {
      //std::cout<<"\n**********Creating sub-populations in PC***********\n";
      population_.createSubPopulations();
    }

    sendPopulation();

    // Build m_i
    //*****setMi();
  }
  
  // Set flag showing that CCs have started
  if(!cc_started_) {
    cc_started_ = true;
  }

  //std::cout<<"\nControl cycle complete\n";
} // End controlCycleCallback








/******************************************************
 **************** Srv Building Methods ****************
 ******************************************************/



/** Build a TrajectoryRequest srv 
 *  type = 0 if all straight-lines, 1 if bezier, 2 if partial bezier, 3 for transition
 * */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path) const {
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = path.buildPathMsg();
  result.request.resolutionRate = resolutionRate_;
  result.request.type           = PARTIAL_BEZIER;

  return result;
} // End buildTrajectoryRequest





/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = trajec.msg_;
  result.request.goal = goal_.buildMotionStateMsg();

  return result;
} // End buildEvaluationRequest





/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/




/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {

  if(!stop_) {

    // If infeasible and too close to obstacle, 
    // Stop the robot by sending a blank trajectory
    if(!bestTrajec_.msg_.feasible && (bestTrajec_.timeUntilCollision_ < 3.f)) {
      std::cout<<"\nCollision within 3 seconds! Stopping robot!\n";

    }
    else if(!bestTrajec_.msg_.feasible) {
      std::cout<<"\nBest trajectory is not feasible! Time until collision: "<<bestTrajec_.timeUntilCollision_;
    }
    
    h_control_->send(bestTrajec_.msg_);
  
  }
  else {
    std::cout<<"\nSending Blank\n";
    RampTrajectory blank;
    h_control_->send(blank.msg_);
  }
} // End sendBest







/** Send the whole population of trajectories to the trajectory viewer */
void Planner::sendPopulation() {
  ramp_msgs::Population msg;

  if(subPopulations_) {
    Population temp(population_.getNumSubPops());
    std::vector<RampTrajectory> trajecs = population_.getBestFromSubPops();
    for(uint8_t i=0;i<trajecs.size();i++) {
      temp.add(trajecs.at(i));
    }

    temp.getBestID();
    msg = temp.populationMsg();
  }
  else {
    msg = population_.populationMsg();
  }

  msg.robot_id = id_;
  h_control_->sendPopulation(msg);
}

void Planner::displayTrajectory(const ramp_msgs::RampTrajectory traj) const {
  ramp_msgs::Population pop;
  pop.population.push_back(traj);
  h_control_->sendPopulation(pop);
}



/** Returns true if traj's fitness is better than the best fitness */
const bool Planner::compareSwitchToBest(const RampTrajectory traj) const {
  double bestFitness = bestTrajec_.msg_.fitness;
  //std::cout<<"\nBest: "<<bestTrajec_.toString();
  //std::cout<<"\nbestTrajec's last point: "<<utility_.toString(bestTrajec_.msg_.trajectory.points.at(
    //                  bestTrajec_.msg_.trajectory.points.size()-1 ));
  std::cout<<"\ntraj's last point: "<<utility_.toString(traj.msg_.trajectory.points.at(
                      traj.msg_.trajectory.points.size()-1 ));
  std::cout<<"\nbestTrajec.fitness: "<<bestTrajec_.msg_.fitness;
  std::cout<<"\ntraj.fitness: "<<traj.msg_.fitness<<" bestFitness: "<<bestFitness;
  std::cout<<"\nc_pc: "<<c_pc_;
  std::cout<<"\n(generationsPerCC_ - c_pc_) * planningCycle_.toSec(): "<<(generationsPerCC_ - c_pc_) * planningCycle_.toSec();
  std::cout<<"\nAdding on "<<c_pc_ * (generationsPerCC_ - c_pc_) * planningCycle_.toSec()<<" seconds";

  // Add delta t to bestFitness
  // delta t = time until next control cycle occurs = time until startPlanning
  bestFitness -= (generationsPerCC_ - c_pc_) * planningCycle_.toSec();

  return (traj.msg_.fitness > bestFitness);
}



/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
const RampTrajectory Planner::evaluateTrajectory(RampTrajectory trajec) {
  RampTrajectory result = trajec;

  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(trajec);
  
  // Do the evaluation and set the fitness and feasibility members
  if(requestEvaluation(er)) {
    result.msg_.fitness    = er.response.fitness;
    result.msg_.feasible   = er.response.feasible;
    result.timeUntilCollision_  = er.response.time_until_collision;
    //std::cout<<"\nEvaluation Request Time Until Collision: "<<er.response.time_until_collision;
  }
  else {
    // TODO: some error handling
  }


  // If the fitness is close to the best resulttory's fitness
  if(cc_started_ && result.id_ != bestTrajec_.id_ &&
      (result.msg_.fitness > bestTrajec_.msg_.fitness ||
      fabs(result.msg_.fitness - bestTrajec_.msg_.fitness) < 5) ) 
  {
    std::cout<<"\n==============================\n";
    std::cout<<"\ntrajectory being evaluated: "<<result.path_.toString();
    std::cout<<"\ntrajectory being evaluated is close to best fitness";
    std::cout<<"\nresult id: "<<result.id_<<" bestTrajec_.id: "<<bestTrajec_.id_;
    std::cout<<"\nresult_.fitness: "<<result.msg_.fitness<<" bestTrajec__.fitness: "<<bestTrajec_.msg_.fitness;
    stopForDebugging();
    std::cout<<"\n==============================\n";

    // Get transition trajectory
    RampTrajectory temp = getTrajectoryWithCurve(trajec);

    std::cout<<"\n\n\nTransition trajectory: "<<temp.toString();
    std::cout<<"\n***** Displaying trajectory with curve! *****\n";
    displayTrajectory(temp.msg_);


    ramp_msgs::EvaluationRequest er_switch = buildEvaluationRequest(temp);
    
    // Do the evaluation and set the fitness and feasibility member_switchs
    if(requestEvaluation(er_switch)) {
      temp.msg_.fitness    = er_switch.response.fitness;
      temp.msg_.feasible   = er_switch.response.feasible;
      temp.timeUntilCollision_  = er_switch.response.time_until_collision;
    }
    else {
      // TODO: some error handling
    }

    // If the trajectory including the curve to switch is more fit than 
    // the best trajectory, return it
    if(compareSwitchToBest(temp)) {
      std::cout<<"\ncompareSwitchToBest: true\n";
      std::cin.get();
      return temp;
    }
    std::cout<<"\ncompareSwitchToBest: false\n";
    std::cin.get();
  } // end if 

  return result;
} // End evaluateTrajectory



/** 
 * This method evaluates each trajectory in the population
 * It also sete i_best_prev_
 **/
void Planner::evaluatePopulation() {
  std::cout<<"\nbest id: "<<population_.getBestID()<<"\n";
  std::cout<<"\npopulation.size(): "<<population_.size()<<"\n";
  if(population_.getBestID() > -1) {
    population_.replace(population_.getBestID(), 
        evaluateTrajectory(population_.get(population_.getBestID())));
    bestTrajec_ = population_.get(population_.getBestID()); 
  }
  
  // Go through each trajectory in the population and evaluate it
  for(unsigned int i=0;i<population_.size();i++) {
    if(i != bestTrajec_.id_) {
      population_.replace(i, evaluateTrajectory(population_.get(i)));
    }
  } // end for

  i_best_prev_ = population_.getBestID();
} // End evaluatePopulation




/** This method calls evaluatePopulation and population_.getBest() */
const RampTrajectory Planner::evaluateAndObtainBest() {
  // Evaluate population
  evaluatePopulation();
  
  // Find the best trajectory
  int index = population_.getBestID();
  i_best_prev_ = index;

  // Return the trajectory
  return population_.get(index);
}



const std::string Planner::pathsToString() const {
  std::ostringstream result;

  result<<"\nPaths:";
  for(unsigned int i=0;i<population_.paths_.size();i++) {
    result<<"\n  "<<population_.paths_.at(i).toString();
  }
  result<<"\n";
  return result.str();
}




const MotionState Planner::findAverageDiff() {
  MotionState result(SP_LU_diffs_.at(0));

  for(uint16_t i=1;i<SP_LU_diffs_.size();i++) {
    result = result.add(SP_LU_diffs_.at(i).abs());
  }

  result = result.divide(SP_LU_diffs_.size());

  return result;
}




/*******************************************************
 ****************** Start the planner ******************
 *******************************************************/


 void Planner::go() {

  // t=0
  generation_ = 0;
  
  // initialize population
  initPopulation();
  std::cout<<"\nPopulation initialized!\n";
  std::cout<<"\n"<<population_.fitnessFeasibleToString();
  
  std::cout<<"\nSeeding population\n";
  seedPopulation();
  i_best_prev_ = population_.getBestID();
  std::cout<<"\nPopulation seeded!\n";
  std::cout<<"\n"<<population_.fitnessFeasibleToString()<<"\n";

  // Evaluate after seeding
  bestTrajec_ = evaluateAndObtainBest();

  if(subPopulations_) {
    population_.createSubPopulations();
  }

  sendPopulation();
  
  std::cout<<"\nPress enter to continue\n";
  std::cin.get();
  

  
  // Start the planning cycle timer
  planningCycleTimer_.start();

  // Wait for 100 generations before starting 
  //****while(generation_ < generationsBeforeCC_) {ros::spinOnce();}

  std::cout<<"\n***************Starting Control Cycle*****************\n";
  // Start the control cycle timer
  std::cout<<"\n********Robot "<<id_<<": Starting Control Cycle********\n";
  controlCycleTimer_.start();
  imminentCollisionTimer_.start();
  
  // Do planning until robot has reached goal
  // D = 0.4 if considering mobile base, 0.2 otherwise
  goalThreshold_ = 0.2;
  ros::Rate r(20);
  while( (latestUpdate_.comparePosition(goal_, false) > goalThreshold_) && ros::ok()) {
    r.sleep();
    ros::spinOnce(); 
  } // end while

  std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString();
  std::cout<<"\ngoal: "<<goal_.toString();
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();

  
  // Send an empty trajectory
  ramp_msgs::RampTrajectory empty;
  h_control_->send(empty);
  
  std::cout<<"\nPlanning complete!";
  std::cout<<"\nLatest update: "<<latestUpdate_.toString();
  std::cout<<"\nLatest start: "<<start_.toString();
  std::cout<<"\nLatest startPlanning_: "<<startPlanning_.toString();
  //std::cout<<"\nbestTrajec: "<<bestTrajec_.toString()<<"\n";
  

  //std::cout<<"\nFinal population: ";
  //std::cout<<"\n"<<pathsToString(); 
  
  std::cout<<"\nLeaving go\n";
} // End go
