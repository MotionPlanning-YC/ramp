#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(5), D_(3.f), cc_started_(false), c_pc_(0), transThreshold_(1./50.), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), stop_(false), num_controlCycles_(0) 
{
  controlCycle_ = ros::Duration(2.f);
  planningCycle_ = ros::Duration(1.f / 10.f);
  imminentCollisionCycle_ = ros::Duration(1.f / 10.f);
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


/** Explicitly restart the control cycles 
 *  Need to call doControlCycle because the timers first iteration is not now */
void Planner::restartControlCycle(const double t) {
  ROS_INFO("Restarting Control Cycle");
  controlCycleTimer_.stop();

  // If a CC time is specified, change the timer
  if(t != controlCycle_.toSec()) {
    ROS_WARN("New time for next control cycle: %f", t);
    controlCycle_ = ros::Duration(t);
    controlCycleTimer_.setPeriod(controlCycle_);
  }

  doControlCycle();
  controlCycleTimer_.start();
} // End restartControlCycle





const std::vector<Path> Planner::getRandomPaths(const MotionState init, const MotionState goal) {
  std::vector<Path> result;

  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path(init, goal);

    // Each trajectory will have a random number of knot points
    // Put a max of 3 knot points for practicality...
    unsigned int num = (rand() % 3)+1;
  

    // For each knot point to be created
    for(unsigned int j=0;j<num;j++) {

      // Create a random configuration
      MotionState temp_ms;
      temp_ms = randomizeMSPositions(temp_ms);
      
      // Push on velocity values (for target v when getting trajectory)
      for(unsigned int i=0;i<temp_ms.msg_.positions.size();i++) {
        temp_ms.msg_.velocities.push_back(0);
      }
      
      // Add the random configuration to the path
      temp_path.addBeforeGoal(temp_ms); 
    }

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getRandomPaths




/** Create a random Population */
const Population Planner::randomPopulation(const MotionState init, const MotionState goal) {
  //ROS_INFO("In Planner::randomPopulation");
  Population result;

  // Set the max size
  result.maxSize_ = populationSize_;
  
  // Get some random paths
  std::vector<Path> randomPaths =  getRandomPaths(init, goal);


  // Build a list of TrajectoryRequests
  for(uint8_t i=0;i<randomPaths.size();i++) {
    ramp_msgs::TrajectoryRequest temp = buildTrajectoryRequest(randomPaths.at(i));
  }
  

  // Get trajectories for the paths
  std::vector<RampTrajectory> trajecs = getTrajectories(randomPaths);

  // Add each trajectory to the population
  // Use add over replaceAll in case of sub-populations
  for(uint8_t i=0;i<trajecs.size();i++) {
    result.add(trajecs.at(i));
  }

  // Create sub-pops if enabled
  if(subPopulations_) {
    result.createSubPopulations();
  }

  // Evaluate the population 
  result = evaluatePopulation(result);

  //ROS_INFO("Exiting Planner::randomPopulation");
  return result;
} // End randomPopulation



const uint8_t Planner::getIndexStartPathAdapting(const RampTrajectory t) const {
  uint8_t result;
  bool    has_curve = t.msg_.curves.size() > 0;

  //if(has_curve && t.msg_.curves.size() == 2) {
  //}
  if(t.transitionTraj_.trajectory.points.size() > 0) {
    result = bestTrajec_.transitionTraj_.i_knotPoints.size();
  }
  else if(has_curve && t.msg_.curves.at(0).u_0 == 0) {
    result = 2;
  }
  else {
    result = 1;
  }

  //ROS_INFO("getIndexStartPathAdapting returning: %i", result);
  return result;
}




const uint8_t Planner::getNumThrowawayPoints(const RampTrajectory traj, const ros::Duration dur) const {
  uint8_t result = 1;

  // For each knot point,
  // Start at 2 because that is the end of the first bezier curve
  for(uint8_t i_kp=getIndexStartPathAdapting(traj);
      i_kp<traj.msg_.i_knotPoints.size();
      i_kp++) 
  {
    //ROS_INFO("i_kp: %i", (int)i_kp);

    // Get the knot point 
    trajectory_msgs::JointTrajectoryPoint point = traj.msg_.trajectory.points.at( 
                                                    traj.msg_.i_knotPoints.at(i_kp));

    // Compare the durations
    if( (dur > point.time_from_start) || 
        (fabs(dur.toSec() - point.time_from_start.toSec()) < 0.0001) ) 
    {
      //ROS_INFO("Past KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
      result++;
    }
    else {
      //ROS_INFO("Behind KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
      break;
    }
  } // end inner for


  return result;
}

/** 
 * This method updates all the paths with the current configuration 
 * For each knot point in a path, it will remove the knot point if
 * its time_from_start is <= the Duration argument
 * */
const std::vector<Path> Planner::adaptPaths(MotionState start, ros::Duration dur) const {
  //ROS_INFO("In Planner::adaptPaths");
  std::vector<Path> result;

  // Check that time has passed
  if(dur.toSec() > 0) {


    // For each trajectory
    for(uint8_t i=0;i<population_.size();i++) {
      //ROS_INFO("Path: %s", population_.paths_.at(i).toString().c_str());
      //ROS_INFO("Get Path: %s", population_.get(i).getPath().toString().c_str());
      Path temp = population_.paths_.at(i);

      // Track how many knot points we get rid of
      // Initialize to 1 to always remove starting position
      unsigned int throwaway=getNumThrowawayPoints(population_.get(i), dur);
      //ROS_INFO("throwaway: %i", (int)throwaway);

      // For each knot point,
      // Start at 2 because that is the end of the first bezier curve
      /*for(uint8_t i_kp=getIndexStartPathAdapting(population_.get(i));
          i_kp<population_.get(i).msg_.i_knotPoints.size();
          i_kp++) 
      {
        //ROS_INFO("i_kp: %i", (int)i_kp);

        // Get the knot point 
        trajectory_msgs::JointTrajectoryPoint point = population_.get(i).msg_.trajectory.points.at( 
                                                        population_.get(i).msg_.i_knotPoints.at(i_kp));

        // Compare the durations
        if( (dur > point.time_from_start) || 
            (fabs(dur.toSec() - point.time_from_start.toSec()) < 0.0001) ) 
        {
          //ROS_INFO("Past KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
          throwaway++;
        }
        else {
          //ROS_INFO("Behind KP, dur.toSec(): %f kp time: %f", dur.toSec(), point.time_from_start.toSec());
          break;
        }
      } // end inner for*/

      //ROS_INFO("throwaway: %i", throwaway);

      // If the whole path has been passed, adjust throwaway so that 
      //  we are left with a path that is: {new_start_, goal_}
      if( throwaway >= population_.paths_.at(i).size() ) { 
        //ROS_INFO("Decrementing throwaway");
        throwaway = population_.paths_.at(i).size()-1;
      }


      // Print the knot points being removed
      /*for(int c=0;c<throwaway;c++) {
        ROS_INFO("\nRemoving point: %s", (*(population_.paths_.at(i).all_.begin()+c)).toString().c_str());
      }*/

      //std::cout<<"\nErasing "<<population_.paths_.at(i).all_.at(0).toString();
      // Erase the amount of throwaway points (points we have already passed)
      temp.all_.erase( 
          temp.all_.begin(), 
          temp.all_.begin()+throwaway );

      // Insert the new starting configuration
      temp.all_.insert( temp.all_.begin(), start);

      // Set start_ to be the new starting configuration of the path
      temp.start_ = start;
      //ROS_INFO("After adapting Path: %s", temp.toString().c_str());

      result.push_back(temp);
    } // end outer for
  } // end if dur > 0

  return result;
  //ROS_INFO("Exiting adaptPaths");
} // End adaptPaths



// TODO: Check for the 2nd segment as well?
const std::vector<double> Planner::getScaledXY(const MotionState ms, const ramp_msgs::BezierInfo curve) const {
  std::vector<double> result;

  bool xSlope = (curve.segmentPoints.at(1).positions.at(0) - curve.segmentPoints.at(0).positions.at(0) > 0);
  bool ySlope = (curve.segmentPoints.at(1).positions.at(1) - curve.segmentPoints.at(0).positions.at(1) > 0);
  //ROS_INFO("xSlope: %s ySlope: %s", xSlope ? "True" : "False", ySlope ? "True" : "False");

  double x = ms.msg_.positions.at(0);
  double y = ms.msg_.positions.at(1);

  double x_numerator = (xSlope  ? x - curve.controlPoints.at(0).positions.at(0)
                                : curve.controlPoints.at(0).positions.at(0) - x);
  double y_numerator = (ySlope  ? y - curve.controlPoints.at(0).positions.at(1)
                                : curve.controlPoints.at(0).positions.at(1) - y);
  double denom = curve.controlPoints.at(2).positions.at(0) - curve.controlPoints.at(0).positions.at(0);

  double x_scaled = x_numerator / denom;
  double y_scaled = y_numerator / denom;

  result.push_back(x_scaled);
  result.push_back(y_scaled);

  return result;
}


// 1 if before curve, 2 if on curve, 3 if past curve 
// TODO: Check for the 2nd segment as well?
const int Planner::estimateIfOnCurve(const MotionState ms, const ramp_msgs::BezierInfo curve) const {
  //ROS_INFO("In estimateIfOnCurve");
  //ROS_INFO("ms: %s", ms.toString().c_str());
  //ROS_INFO("curve: %s", utility_.toString(curve).c_str());

  std::vector<double> scaled = getScaledXY(ms, curve);
  double x_scaled = scaled.at(0);
  double y_scaled = scaled.at(1);

  ROS_INFO("x_scaled: %f y_scaled: %f", x_scaled, y_scaled);


  bool x_good = (x_scaled >= 0. && x_scaled <= curve.u_target);
  bool y_good = (y_scaled >= 0. && y_scaled <= curve.u_target);


  if(x_scaled < 0. && y_scaled < 0.) {
    ROS_INFO("Returning 1 (before curve)");
    return 1;  
  } 
  else if(x_good && y_good)
  {
    ROS_INFO("Returning 2 (on curve)");
    return 2;
  } 
  else if(x_scaled > curve.u_target && y_scaled > curve.u_target) {
    ROS_INFO("Returning 3 (after curve)");
    return 3;  
  }


  ROS_INFO("Returning 1");
  return 1;
}



const ramp_msgs::BezierInfo Planner::handleCurveEnd(const RampTrajectory traj) const {
  ramp_msgs::BezierInfo result;

  // If there are 2 curves, set the next one up 
  // Currently, the adaptive control cycles always end at
  // the start of the next curve so we can safely assume the position should be 0
  // If the control cycles will ever occur in the middle of a 2nd curve, we will have to calculate u_0
  
  if(traj.msg_.curves.size() > 1) {
    result = traj.msg_.curves.at(1);
    //ROS_INFO("Calling estimateIfOnCurve for second curve");
    estimateIfOnCurve(startPlanning_, traj.msg_.curves.at(1));
    std::vector<double> scaled = getScaledXY(startPlanning_, traj.msg_.curves.at(1));
    
    result.u_0 += (scaled.at(0) + scaled.at(1)) / 2.;
    result.ms_begin = traj.path_.start_.motionState_.msg_;
  }

  // Else if there was only 1 curve, just return a blank one

  return result;
} // End handleCurveEnd



const double Planner::updateCurvePos(const RampTrajectory traj) const {
  ROS_INFO("In Planner::updateCurvePos");
  //ROS_INFO("startPlanning_: %s", startPlanning_.toString().c_str());
  ROS_INFO("traj: %s", traj.toString().c_str());
  ramp_msgs::BezierInfo curve = traj.msg_.curves.at(0); 
  double result = curve.u_0;

  // if u_0==0 then estimateIfOnCurve returned 2 - already on curve
  if(curve.u_0 == 0) {
    ROS_INFO("In if curve.u_0==0");

    // Get the time at the start of the curve
    double t_s0 = traj.msg_.trajectory.points.at(
        traj.msg_.i_knotPoints.at(1)).time_from_start.toSec();
    ROS_INFO("t_s0: %f", t_s0);

    // t = the time spent moving on the curve
    double t = controlCycle_.toSec() - t_s0;

    ROS_INFO("t: %f Adding %f", t, (t*curve.u_dot_0));
    std::vector<double> scaled = getScaledXY(startPlanning_, traj.msg_.curves.at(0));
    

    // At one point I changed this to the average of x and y scaled
    // But my notes don't say why...
    // But that change made a robot's prediction along a curve noticeably more inaccurate
    //ROS_INFO("Adding %f",(scaled.at(0) + scaled.at(1)) / 2.);
    //result += (scaled.at(0) + scaled.at(1)) / 2.;
    ROS_INFO("Adding %f", t*curve.u_dot_0);
    result += t * curve.u_dot_0;
  } // end if already on curve

  // Else, u_0 > 0, simply add to u_0
  else {
    ROS_INFO("Else not moving on curve, curve.u_0 > 0: %f curve.u_dot_0: %f", curve.u_0, curve.u_dot_0);
    result += curve.u_dot_0 * controlCycle_.toSec();
  }

  return result;
} // End updateCurvePos


/** Updates the curve u_0 and ms_begin */
// TODO: Only need to update the bestTrajec's curve
const std::vector<ramp_msgs::BezierInfo> Planner::adaptCurves(const Population pop) const {
  ROS_INFO("In adaptCurves");

  std::vector<ramp_msgs::BezierInfo> result;
  ramp_msgs::BezierInfo blank;

  // Go through each trajectory 
  for(uint16_t i=0;i<population_.size();i++) {
    ROS_INFO("Curve %i", (int)i);
    
    // If the trajectory has a curve
    if(pop.get(i).msg_.curves.size() > 0) {
      ROS_INFO("In if trajectory has curve");

      // Set curve
      ramp_msgs::BezierInfo curve = pop.get(i).msg_.curves.at(0); 
      //ROS_INFO("Set curve to: %s", utility_.toString(curve).c_str());

      // If moving on this curve, update u
      if( i == pop.getBestIndex() && 
            (curve.u_0 > 0 ||
             estimateIfOnCurve(startPlanning_, curve) == 2))
      {
        ROS_INFO("Moving on this curve");

        // Get the new u_0 value
        curve.u_0 = updateCurvePos(pop.get(i));

        // Set the new ms_begin
        curve.ms_begin = startPlanning_.msg_;
      }  //end if moving on curve

     
      /* Separate checking if on the curve and if done with curve
           because we could be done before ever incrementing u_0 */
      // Check if done with current curve
      if(estimateIfOnCurve(startPlanning_, curve) == 3 || curve.u_0 > curve.u_target) {
        //ROS_INFO("Done with curve, u_0: %f", curve.u_0);
        curve = handleCurveEnd(pop.get(i));
      } // end if done with 1st curve

      ROS_INFO("Curve after adapting: %s", utility_.toString(curve).c_str());
      result.push_back(curve);
    } // end if trajectory has curve

    // Else if there is no curve, push on a blank one
    else {
      //ROS_INFO("No curve");
      result.push_back(blank);
    } // end else no curve
  } // end for

  return result;
} // End adaptCurves




/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
// Not const because it calls requestTrajectory which can call getIRT
const Population Planner::adaptPopulation(const Population pop, const MotionState ms, const ros::Duration d) {
  //ROS_INFO("In adaptPopulation");
  //ROS_INFO("startPlanning_: %s \nduration: %f", startPlanning_.toString().c_str(), d.toSec());
  Population result = pop;
  
 
  // Adapt the paths and curves
  std::vector<Path> paths                   = adaptPaths(startPlanning_, d);
  std::vector<ramp_msgs::BezierInfo> curves = adaptCurves(population_);
  
  result.paths_ = paths;

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  // For each path, get a trajectory
  for(uint16_t i=0;i<population_.paths_.size();i++) {
    RampTrajectory tempTraj = population_.get(i);
    ROS_INFO("Getting trajectory %i", (int)i);

    ROS_INFO("curves.size(): %i", (int)curves.size());
    std::vector<ramp_msgs::BezierInfo> c;
    c.push_back(curves.at(i));
    
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(paths.at(i), c);

    /* Get the trajectory */
    RampTrajectory temp = requestTrajectory(tr, result.get(i).msg_.id);
    ROS_INFO("After requesting trajectory");

    // Set temporary evaluation results - need to actually call requestEvaluation to get actual fitness
    temp.msg_.fitness   = result.get(i).msg_.fitness;
    temp.msg_.feasible  = result.get(i).msg_.feasible;

    // Push onto updatedTrajecs
    updatedTrajecs.push_back(temp);
  } // end for

  // Replace the population's trajectories_ with the updated trajectories
  result.replaceAll(updatedTrajecs);

  return result;
  
  //ROS_INFO("Done adapting, pop now: %s", population_.toString().c_str());
  ROS_INFO("Exiting adaptPopulation");
} // End adaptPopulation





// TODO: Clean up
/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<ramp_msgs::BezierInfo> curves, const int id) const {
  ROS_INFO("In buildTrajectoryRequest");
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = path.buildPathMsg();
  result.request.resolutionRate = resolutionRate_;
  result.request.type           = PARTIAL_BEZIER;

  // If path size > 2, assign a curve
  if(path.size() > 2) {
    //ROS_INFO("In if path.size() > 2)");

    // If it's the first time getting a curve 
    if(curves.size() == 0 || curves.at(0).segmentPoints.size() == 0) {
      if(path.size() > 2) {
        //ROS_INFO("In if path.size() >= 3");
        ramp_msgs::BezierInfo temp;
        
        /*if(id > -1 && pop_orig_.size() > 0) {
          ROS_INFO("In if id > -1 && pop_orig_.size() > 0");
          ROS_INFO("pop_orig_.get(population_.getIndexFromId(id)).path_.all_.at(0).motionState_: %s",
pop_orig_.get(population_.getIndexFromId(id)).path_.all_.at(0).motionState_.toString().c_str());
          temp.segmentPoints.push_back( 
              pop_orig_.get(population_.getIndexFromId(id)).path_.all_.at(0).motionState_.msg_ );
        }
        else {*/
          temp.segmentPoints.push_back( path.all_.at(0).motionState_.msg_ );
        //}
        temp.segmentPoints.push_back( path.all_.at(1).motionState_.msg_ );
        temp.segmentPoints.push_back( path.all_.at(2).motionState_.msg_ );
        result.request.bezierInfo.push_back(temp);
      }
    }
    else {
      //ROS_INFO("In else if path.size < 3");
      result.request.bezierInfo = curves;
    } // end else
  } // end if


  ROS_INFO("Exiting Planner::buildTrajectoryRequest");
  return result;
} // End buildTrajectoryRequest


const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const int id) const {
  std::vector<ramp_msgs::BezierInfo> curves;
  return buildTrajectoryRequest(path, curves, id);
}





/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory = trajec.msg_;
  result.request.goal = goal_.msg_;

  return result;
} // End buildEvaluationRequest





/** This method initializes the T_w_odom_ transform object */
void Planner::setT_base_w(std::vector<double> base_pos) {
  T_w_odom_.setRotation(tf::createQuaternionFromYaw(base_pos.at(2)));
  T_w_odom_.setOrigin(  tf::Vector3(base_pos.at(0), base_pos.at(1), 0));
} // End setT_base_w



/** Returns an id for RampTrajectory objects */
const unsigned int Planner::getIRT() { return i_rt++; }




/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) {
  //ROS_INFO("In imminentCollisionCallback");
 

  if(!bestTrajec_.msg_.feasible && (bestTrajec_.msg_.t_firstCollision < D_)) {
    ROS_WARN("Imminent Collision Robot: %i t_firstCollision: %f", 
      id_, 
      bestTrajec_.msg_.t_firstCollision);
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

 
  if(msg.positions.size() < 3 ||
     msg.velocities.size() < 3 ||
     msg.accelerations.size() < 3 )
  { 
    ROS_ERROR("Odometry message from ramp_control does not have all DOFs: %s", 
        utility_.toString(msg).c_str());
  }
  else {
    latestUpdate_ = msg;

    // Transform configuration from odometry to world coordinates
    latestUpdate_.transformBase(T_w_odom_);

    // Set proper velocity values
    latestUpdate_.msg_.velocities.at(0) = msg.velocities.at(0) * 
                                          cos(latestUpdate_.msg_.positions.at(2));
    latestUpdate_.msg_.velocities.at(1) = msg.velocities.at(0) * 
                                          sin(latestUpdate_.msg_.positions.at(2));

    // Set proper acceleration values
    latestUpdate_.msg_.accelerations.at(0) = msg.accelerations.at(0) * 
                                             cos(latestUpdate_.msg_.positions.at(2));
    latestUpdate_.msg_.accelerations.at(1) = msg.accelerations.at(0) * 
                                             sin(latestUpdate_.msg_.positions.at(2));

    //ROS_INFO("latestUpdate: %s", latestUpdate_.toString().c_str());
  } // end else
} // End updateCallback






/** This method sets random values for the position vector of ms
 *  ONLY RANDOMIZES POSITIONS */
const MotionState Planner::randomizeMSPositions(MotionState ms) const {
  MotionState result = ms;
  result.msg_.positions.clear();

  for(unsigned int i=0;i<ranges_.size();i++) {
    result.msg_.positions.push_back(ranges_.at(i).random());
  }

  return result;
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
void Planner::init(const uint8_t i, const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r, const int population_size, const bool sub_populations, const int gens_before_cc, const double t_fixed_cc, const bool errorReduction) {
  //ROS_INFO("In Planner::init");

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
  setT_base_w(start_.msg_.positions);

  // Set misc members
  populationSize_       = population_size;
  subPopulations_       = sub_populations;
  generationsBeforeCC_  = gens_before_cc;
  t_fixed_cc_           = t_fixed_cc;
  errorReduction_       = errorReduction;
} // End init






/** Place code to seed population here */
void Planner::seedPopulation() {

  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(0.5);
  kp.motionState.positions.push_back(3);
  kp.motionState.positions.push_back(1.41282); // 80 degrees 
  
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


  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(0.5);
  kp2.motionState.positions.push_back(3.);
  kp2.motionState.positions.push_back(2.21431);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(start_);
  all2.push_back(kp2);
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
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    new_pop.push_back(evaluateTrajectory(trajec));
  
  } // end for
  /************************************/

  population_.replaceAll(new_pop); 
} // End seedPopulation



/** Will seed population with a straight-line trajectory to the goal */
void Planner::seedPopulationTwo() {


  /**** Create the Paths ****/
  ramp_msgs::KnotPoint kp;
  
  kp.motionState.positions.push_back(1);
  kp.motionState.positions.push_back(1);
  kp.motionState.positions.push_back(1.41282); // 80 degrees 
  

  std::vector<KnotPoint> all;
  all.push_back(startPlanning_);
  all.push_back(kp);
  all.push_back(goal_);

  Path p1(all);


  ramp_msgs::KnotPoint kp2;
  
  kp2.motionState.positions.push_back(1);
  kp2.motionState.positions.push_back(-2);
  kp2.motionState.positions.push_back(2.21431);
  
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);
  kp2.motionState.velocities.push_back(0);

  std::vector<KnotPoint> all2;
  all2.push_back(startPlanning_);
  all2.push_back(kp2);
  all2.push_back(goal_);

  Path p2(all2);
  /****************************/

  /**** Create the vector of Paths ****/

  std::vector<Path> paths;
  paths.push_back(p1);
  paths.push_back(p2);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    new_pop.push_back(evaluateTrajectory(trajec));
  
  } // end for
  /************************************/

  population_.replaceAll(new_pop);  
} // End seedPopulationTwo





/** This method returns true if the robot has orientation to move on the best trajectory */
const bool Planner::checkOrientation() const {
  //std::cout<<"\nEntering checkOrientation\n";
  
  float actual_theta = latestUpdate_.msg_.positions.at(2);
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




void Planner::setMi() {
  ROS_INFO("In Planner::setMi");

  // Clear m_i
  m_i_.clear();
  
  // Need to set m_delta
  // motion difference from previous CC to next CC
  MotionState delta_m = m_cc_.subtract(latestUpdate_);
  ROS_INFO("mcc: %s\nlatestUpdate: %s", 
      m_cc_.toString().c_str(), 
      latestUpdate_.toString().c_str());
  ROS_INFO("delta_m: %s", delta_m.toString().c_str());

  // Divide delta_m by num_pc to get the motion difference for each PC
  MotionState delta_m_inc = delta_m.divide(generationsPerCC_);
  //std::cout<<"\nDelta_m / num_pc: "<<delta_m_inc.toString();
 
  // Set m_i
  // Each m_i will be start + (delta_m_inc * i)
  for(int i=0;i<generationsPerCC_;i++) {
    MotionState temp = delta_m_inc.multiply(i+1);
    MotionState m = latestUpdate_.add(temp);

    m_i_.push_back(m);

    ROS_INFO("m_i[%i]: %s", i, m.toString().c_str());
  } // end for


  ROS_INFO("Exiting Planner::setMi");
} // End setMi




/** Pass in entire RampTrajectory because we need the path info */
const ramp_msgs::BezierInfo Planner::replanCurve(const RampTrajectory trajec, const MotionState ms_start) const {
  ramp_msgs::BezierInfo result = trajec.msg_.curves.at(0);

  double delta_x = trajec.path_.all_.at(1).motionState_.msg_.positions.at(0) - result.segmentPoints.at(0).positions.at(0);
  double delta_y = trajec.path_.all_.at(1).motionState_.msg_.positions.at(1) - result.segmentPoints.at(0).positions.at(1);
  double l = sqrt( pow(delta_x, 2) + pow(delta_y, 2) );

  double theta = ms_start.msg_.positions.at(2);

  double x = l*cos(theta);
  double y = l*sin(theta);

  ROS_INFO("delta_x: %f delta_y: %f l: %f theta: %f x: %f y: %f",
      delta_x, delta_y,
      l, theta,
      x, y);
  result.segmentPoints.at(1).positions.at(0) = x;
  result.segmentPoints.at(1).positions.at(1) = y;
  result.controlPoints.clear();



  return result;
}


const RampTrajectory Planner::replanTrajec(const RampTrajectory trajec, const MotionState ms_start) {
  ROS_INFO("In Planner::replanTrajec");

  RampTrajectory result = trajec;
  ROS_INFO("After setting result");
  ROS_INFO("result: %s", result.toString().c_str());
  ROS_INFO("result.curves.size: %i", (int)result.msg_.curves.size());

  result.path_.start_ = ms_start;
  ROS_INFO("ms_start: %s", ms_start.toString().c_str());

  result.path_.all_.erase( result.path_.all_.begin() );
  result.path_.all_.insert( result.path_.all_.begin(), ms_start);

  double v = sqrt(  pow( ms_start.msg_.velocities.at(0), 2) + 
                    pow( ms_start.msg_.velocities.at(1), 2) );
  ROS_INFO("v: %f", v);
  if(result.msg_.curves.size() > 0 && v > 0.0001 && result.msg_.curves.at(0).u_0 < 0.001) {
    result.msg_.curves.at(0) = replanCurve( trajec, ms_start );
  }
  else if(trajec.path_.size() > 2) {
  }
  else {
    ROS_INFO("Not replanning curve for trajec id: %i", trajec.msg_.id);
    ROS_INFO("v: %f curves.size(): %i",
        v,
        (int)result.msg_.curves.size());
    if(result.msg_.curves.size() > 0) {
      ROS_INFO("curve.u_0: %f", result.msg_.curves.at(0).u_0);
    }
  }

  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(result.path_, result.msg_.curves, trajec.msg_.id);

  result = requestTrajectory(tr, result.msg_.id);

  ROS_INFO("Replanned Trajec: %s", result.toString().c_str());

  ROS_INFO("Exiting Planner::replanTrajec");
  return result;
}

const std::vector<RampTrajectory> Planner::replanTrajecs(const std::vector<RampTrajectory> trajecs, const MotionState ms_start) {
  ROS_INFO("In Planner::replanTrajecs");
  std::vector<RampTrajectory> result;

  for(uint8_t i=0;i<trajecs.size();i++) {
    ROS_INFO("i: %i trajecs.size(): %i", (int)i, (int)trajecs.size());
    RampTrajectory temp = replanTrajec(trajecs.at(i), ms_start);
    result.push_back(temp);
  }

  ROS_INFO("Exiting Planner::replanTrajecs");
  return result;
}


/** This method will return a vector of trajectoies for the vector of paths */
const std::vector<RampTrajectory> Planner::getTrajectories(const std::vector<Path> p) {
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<p.size();i++) {
    ROS_INFO("i: %i p.size(): %i", (int)i, (int)p.size());
    // Get a trajectory
    RampTrajectory temp = requestTrajectory(p.at(i));
    result.push_back(temp);
  } // end for

  ROS_INFO("Exiting getTrajectories");
  return result;
} // End getTrajectories





/** This method will return a vector of trajectoies for the vector of paths */
// TODO: trajectoryrequest reference?
const std::vector<RampTrajectory> Planner::getTrajectories(std::vector<ramp_msgs::TrajectoryRequest> tr) {
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<tr.size();i++) {
    
    // Get a trajectory
    RampTrajectory temp = requestTrajectory(tr.at(i));
    result.push_back(temp);
  } // end for

  return result;
} // End getTrajectories




/** 
 * This function generates the initial population of trajectories,
 *  sets the paths in the Modifier class
 *  and evaluates the population
 **/
void Planner::initPopulation() { 
  population_ = randomPopulation(latestUpdate_, goal_);
  bestTrajec_ = population_.getBest();
} // End init_population




const bool Planner::checkIfSwitchCurveNecessary(const RampTrajectory from, const RampTrajectory to) const {
  //ROS_INFO("In Planner::CheckIfSwitchCurveNecessary");
  double theta_to_move, theta_current = latestUpdate_.msg_.positions.at(2);

  int kp = 1; 
  // Check if 1st two positions in "from" trajectory are the same
  // If they are, the 2nd kp is the end of a rotation
  // Theta after rotating will tell us the theta needed to move on "to" trajectory
  if( fabs(utility_.positionDistance( from.msg_.trajectory.points.at(
                                      from.msg_.i_knotPoints.at(0)).positions,
                                      from.msg_.trajectory.points.at(
                                      from.msg_.i_knotPoints.at(kp)).positions)) < 0.0001)
  {
    //ROS_INFO("In if positions are the same");
    theta_to_move = from.msg_.trajectory.points.at( from.msg_.i_knotPoints.at(1) ).positions.at(2);
  }
  else {
    //ROS_INFO("In else positions are not the same");
    theta_to_move = utility_.findAngleFromAToB(
                          from.msg_.trajectory.points.at(0), 
                          from.msg_.trajectory.points.at(
                            from.msg_.i_knotPoints.at(kp)) ); 
  }


  //ROS_INFO("theta_current: %f theta_to_move: %f", theta_current, theta_to_move);
  //ROS_INFO("fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move)): %f",
        //fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move)));


  // If a difference of 1 degree, compute a curve
  if(fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move)) > 0.017) {
    //ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning true");
    return true;
  }

  //ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning false");
  return false;
}


const RampTrajectory Planner::computeFullSwitch(const RampTrajectory from, const RampTrajectory to) {
  //ROS_INFO("In Planner::computeFullSwitch");
  RampTrajectory result;

  if(checkIfSwitchCurveNecessary(from, to)) {
    //ROS_INFO("Orientation needs to change");

    // Get transition trajectory
    std::vector<RampTrajectory> trajecs = switchTrajectory(movingOn_, to);
    RampTrajectory T_new = trajecs.at(1);

    // Evaluate T_new
    T_new = requestEvaluation(T_new);
    T_new.transitionTraj_ = trajecs.at(0).msg_;

    result = T_new;
  } // end if curve needed

  // If no switch is necessary, return a blank trajectory
  // because no switching-specific motion is needed
  /*else {
    ROS_INFO("No curve needed to switch");
  }*/

  //ROS_INFO("Exiting Planner::computeFullSwitch");
  return result;
}


// Pass in movingOn_?
const std::vector<RampTrajectory> Planner::switchTrajectory(const RampTrajectory from, const RampTrajectory to) {
  std::vector<RampTrajectory> result;
  //std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString();

  // If greater than 90 degrees
  // TODO: Add 1/angle to evaluation
  /*if(fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move)) > PI/2) {
    ROS_INFO("Orientation change > PI/2 - Too much for a switch - adding penalty");
    double denom = 1./from.msg_.fitness;
    denom += fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move));
    from.msg_.fitness = 1./denom;
  }*/

  // If greater than 5 degrees
  
  //ROS_INFO("Target T: %s", to.toString().c_str());
    
  RampTrajectory switching = getTransitionTrajectory(from, to);
  RampTrajectory full = switching;
  
  // Set the proper ID
  full.msg_.id = to.msg_.id;

  // If the target trajec is a straight line, no need to continue
  // Otherwise, concatenate the rest of the trajec
  if( to.path_.size() > 2 ) {
  
    /* After getting transition trajectory, we want to 
       concatenate it with the rest of the target trajectory */
    
    // Set the cycle time and latest point's time
    ros::Duration t_cycle   = switching.msg_.trajectory.points.at(1).time_from_start - 
                              switching.msg_.trajectory.points.at(0).time_from_start;
    ros::Duration t_latest  = switching.msg_.trajectory.points.at(
                              switching.msg_.trajectory.points.size()-1).time_from_start 
                              + t_cycle;

    // Keep a counter for the knot points
    int c_kp = to.path_.size() < 3 ? 1 : 2;
    //std::cout<<"\nc_kp: "<<c_kp;
    //std::cout<<"\ntrgt path: "<<trgt_traj.toString();

    //ROS_INFO("Checking if rotation at beginning");
    // Check if there's rotation at the beginning, if so increment c_kp
    // TODO: Better way of doing this
    if(utility_.positionDistance( to.msg_.trajectory.points.at(0).positions,
          to.msg_.trajectory.points.at( to.msg_.i_knotPoints.at(1)).positions ) < 0.1)
    {
      //std::cout<<"\nIncrementing c_kp";
      c_kp++;
    }
    ROS_INFO("c_kp: %i", c_kp);


    ROS_INFO("c_kp: %i i_knotPoints.size(): %i", c_kp, (int)to.msg_.i_knotPoints.size());

    // Start at the bezier curve in trgt_traj and 
    // push on the rest of the trajectory to result
    for(uint16_t i=to.msg_.i_knotPoints.at(c_kp-1); 
        i<to.msg_.trajectory.points.size(); 
        i++) 
    {
      trajectory_msgs::JointTrajectoryPoint temp = to.msg_.trajectory.points.at(i);

      // Set proper time
      temp.time_from_start = t_latest;
      t_latest += t_cycle;
      
      // Push on new point
      full.msg_.trajectory.points.push_back( temp );
     
      // If knot point, push on the index
      // and push the point onto the trajectory's path
      if( i == to.msg_.i_knotPoints.at(c_kp) ) {
        full.msg_.i_knotPoints.push_back(full.msg_.trajectory.points.size()-1);
        KnotPoint kp(full.msg_.trajectory.points.at(
              full.msg_.trajectory.points.size()-1));
        full.path_.all_.push_back(kp);
        c_kp++;
      }
    } // end for

    // Set bezierPath
    std::vector<KnotPoint> bp;
    for(uint16_t i=0;i<full.msg_.i_knotPoints.size();i++) {
      KnotPoint kp(full.msg_.trajectory.points.at(
            full.msg_.i_knotPoints.at(i)));
      bp.push_back(kp);
    }

    full.bezierPath_ = bp;
   
    
    
    // Push on the target trajectory's Bezier curve
    for(uint8_t i_curve=0;i_curve<to.msg_.curves.size();i_curve++) {
      full.msg_.curves.push_back(to.msg_.curves.at(i_curve));
    }

   
    // Set i_curveEnd for the 1st curve
    if(switching.msg_.curves.size() > 0) {
      //ROS_INFO("In switching curve if");
      full.msg_.i_curveEnd = switching.msg_.i_curveEnd;
    }
    else if(full.msg_.curves.size() == 0) {
      //ROS_INFO("In switching curve full.curves.size() == 0");
      full.msg_.i_curveEnd = 0;
    }
    else {
      /*ROS_INFO("In switching curve else");
      ROS_INFO("to.msg_.i_curveEnd: %i", (int)to.msg_.i_curveEnd);
      ROS_INFO("to_msg.curves.size(): %i", (int)to.msg_.curves.size());*/
      full.msg_.i_curveEnd = to.msg_.curves.at(0).numOfPoints + switching.msg_.trajectory.points.size(); 
    }
  } // end if to's size > 2

  result.push_back(switching);
  result.push_back(full);
  

  return result;
}





const RampTrajectory Planner::getTransitionTrajectory(const RampTrajectory current, const RampTrajectory trgt_traj) {
  //ROS_INFO("In Planner::getTransitionTrajectory");

  double t = (c_pc_+1)*planningCycle_.toSec();// + planningCycle_.toSec();
  //ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
  
  MotionState ms_startSwitch = current.getPointAtTime(t);
  //ROS_INFO("ms_startSwitch: %s", ms_startSwitch.toString().c_str());
  
  std::vector<MotionState> segment_points;
  segment_points.push_back(ms_startSwitch);
  segment_points.push_back(
      movingOn_.msg_.trajectory.points.at(movingOn_.msg_.trajectory.points.size()-1));
 
  // 2nd knot point should be the initial point on that trajectory's bezier 
  // Using start of Bezier rather than segment endpoint ensures that
  // the trajectory will end at the start of the Bezier
  int i_goal = 1;
  if(utility_.positionDistance( trgt_traj.path_.start_.motionState_.msg_.positions, trgt_traj.msg_.trajectory.points.at(trgt_traj.msg_.i_knotPoints.at(1)).positions ) < 0.1)
  {
    i_goal = 2;
  }
  MotionState g(trgt_traj.msg_.trajectory.points.at(trgt_traj.msg_.i_knotPoints.at(i_goal)));
  segment_points.push_back(g);
  //ROS_INFO("i_goal: %i", i_goal);
  //ROS_INFO("g: %s", g.toString().c_str());


  Path p(segment_points);

  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  tr.request.type = TRANSITION;
  tr.request.print = false; 
  tr.request.startBezier = false;

  //std::cout<<"\nRequesting trajectory in getTransitionTrajectory\n";
  RampTrajectory transition = requestTrajectory(tr);
  //std::cout<<"\ntransition: "<<transition.toString();

  //ROS_INFO("Exiting Planner::getTransitionTrajectory");
  return transition;
}






/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
// Not const because it calls getIRT() to get an index for the trajectory if an id is not passed in
const RampTrajectory Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr, const int id) {
  RampTrajectory result;
  //std::cout<<"\nid: "<<id;

  
  if(h_traj_req_->request(tr)) {
   
    ROS_INFO("response.trajectory.size: %i", (int)tr.response.trajectory.trajectory.points.size());
    // Set the actual trajectory msg
    result.msg_ = tr.response.trajectory;
    ROS_INFO("After set msg");

    // Set the paths (straight-line and bezier)
    result.path_        = tr.request.path;

    if(tr.response.newPath.points.size() > 0) {
      result.bezierPath_  = tr.response.newPath;
    }

    // *** Set the previous knot point
    //result.ms_prevSP_ = tr.request.path.points.at(0).motionState;

    // Set the ID of the trajectory
    if(id != -1) {
      result.msg_.id = id;
    }
    else {
      result.msg_.id = getIRT();
    }
  }
  else {
    ROS_ERROR("An error occurred when requesting a trajectory");
  }

  //ROS_INFO("Exiting Planner::requestTrajectory");
  return result;
}



const RampTrajectory Planner::requestTrajectory(const Path p, const int id) {
  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  RampTrajectory result = requestTrajectory(tr, id);
  ROS_INFO("Exiting Planner::requestTrajectory");
  return result;
}



/** Request an evaluation */
const RampTrajectory Planner::requestEvaluation(ramp_msgs::EvaluationRequest& er) {
  // TODO: Get correct ID
  RampTrajectory result = er.request.trajectory; 
  
  if(h_eval_req_->request(er)) {
    result.msg_.fitness         = er.response.fitness;
    result.msg_.feasible        = er.response.feasible; 
    result.msg_.t_firstCollision  = er.response.t_firstCollision;
  }
  else {
    ROS_ERROR("An error occurred when evaluating a trajectory");
  }
  return result;
}


const RampTrajectory Planner::requestEvaluation(const RampTrajectory traj) {
  ramp_msgs::EvaluationRequest er = buildEvaluationRequest(traj);
  RampTrajectory result = requestEvaluation(er);

  // Set non-evaluation related members
  result.path_          = traj.path_;
  result.bezierPath_    = traj.bezierPath_;
  result.msg_.i_subPopulation = traj.msg_.i_subPopulation; 

  return result;
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
    //std::cout<<"\nramp_planner: Modifying trajectory "<<(int)i;
    
    // Get trajectory
    RampTrajectory temp = requestTrajectory(modded_paths.at(i));
    result.push_back(temp);
  
  } // end for
  
  return result;
} // End modifyTrajectory







/** Modification procedure will modify 1-2 random trajectories,
 *  add the new trajectories, evaluate the new trajectories,
 *  set the new best trajectory,
 *  and return the index of the new best trajectory */
const ModificationResult Planner::modification() {
  //ROS_INFO("In Planner::modification()");
  ModificationResult result;

  // Modify 1 or more trajectories
  std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  

  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {
    //std::cout<<"\nramp_planner: Evaluating trajectory "<<(int)i<<"\n";

    // Evaluate the new trajectory
    mod_trajec.at(i) = evaluateTrajectory(mod_trajec.at(i));

    
    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    // If it was successfully added, push its index onto the result
    int index = population_.add(mod_trajec.at(i));
    if(index > -1) {
      result.i_modified_.push_back(index);
    }

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) {
      //std::cout<<"\nCreating sub-pop after modifying\n";
      population_.createSubPopulations();
      //std::cout<<"\nDone creating sub-pop after modifying\n";
    }
  } // end for


  // Find the best trajectory
  int index   = population_.findBestIndex();
  bestTrajec_ = population_.getBest();
  
  // If the best trajectory has changed and the control cycles have started
  if(index != i_best_prev_ && cc_started_) {
  
    // Set index of previous best
    i_best_prev_ = index;
  } // end if


  result.popNew_ = population_;

  //ROS_INFO("After modification, pop now: %s", result.popNew_.toString().c_str());

  //ROS_INFO("Exiting Planner::modification");
  return result;
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

void Planner::pause() {
  stopForDebugging();
  std::cout<<"\nPress Enter to continue\n";
  std::cin.get();
  restartAfterDebugging();
}





const MotionState Planner::predictStartPlanning() const {
  ROS_INFO("In Planner::predictStartPlanning");

  MotionState result;

  ROS_INFO("c_pc: %i", (int)c_pc_);
  ROS_INFO("m_i.size(): %i", (int)m_i_.size());
 
  ROS_INFO("m_i[%i]: %s", c_pc_, m_i_.at(c_pc_).toString().c_str());
  ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  
  // Get the difference between robot's state and what state it should be at
  MotionState diffI = m_i_.at(c_pc_).subtract(latestUpdate_);
  //std::cout<<"\ndiff: "<<diff.toString();

  MotionState diff = diffI.subtract(totalDiff_);
  
  ROS_INFO("m_cc: %s\ndiffI: %s\ntotalDiff: %s\ndiff: %s", 
      m_cc_.toString().c_str(),
      diffI.toString().c_str(),
      totalDiff_.toString().c_str(),
      diff.toString().c_str());

  // Subtract that difference from startPlanning
  //result = startPlanning_.subtract(diff);
  result = m_cc_.subtract(diffI);


  ROS_INFO("Exiting Planner::predictStartPlanning");
  return result;
}



/** 
 * This method will replace the starting motion state of each path
 * with s and will update the modifier's paths 
 * */
void Planner::updatePathsStart(const MotionState s) {
  ROS_INFO("In Planner::updatePathsStart");

  for(unsigned int i=0;i<population_.paths_.size();i++) {
    population_.paths_.at(i).start_ = s;

    population_.paths_.at(i).all_.erase (population_.paths_.at(i).all_.begin());
    population_.paths_.at(i).all_.insert(population_.paths_.at(i).all_.begin(), s);
  }

  ROS_INFO("Exiting Planner::updatePathsStart");
} // End updatePathsStart



void Planner::planningCycleCallback(const ros::TimerEvent&) {
  ROS_INFO("Planning cycle occurring, generation %i", generation_);
  

  if(generation_ > 0) {
    ROS_INFO("c_pc: %i Occurring at %f, diff of time: %f", 
        (int)c_pc_,
        ros::Time::now().toSec(),
        (ros::Time::now()-t_prevPC_).toSec());
    ROS_INFO("Diff from CC: %f", (ros::Time::now()-t_prevCC_).toSec());
  }

  t_prevPC_ = ros::Time::now();
  

  // Make sure not too many PC occur before next CC
  if(c_pc_ < generationsPerCC_ || !cc_started_) {

    if(errorReduction_ && cc_started_) {
      // Update startPlanning
      MotionState ms_sp = predictStartPlanning();
      ROS_INFO("ms_sp: %s", ms_sp.toString().c_str());
      ROS_INFO("startPlanning_.subtract(ms_sp): %s", 
          startPlanning_.subtract(ms_sp).toString().c_str());
      totalDiff_ = totalDiff_.add( startPlanning_.subtract(ms_sp) );
      ROS_INFO("After adding diff, totalDiff_: %s", totalDiff_.toString().c_str());

      startPlanning_ = ms_sp;
      ROS_INFO("After predicting, startPlanning_: %s", startPlanning_.toString().c_str());
      //std::cout<<"\nAfter predicting startPlanning_:";
      //std::cout<<"\nstartPlanning: "<<startPlanning_.toString()<<"\n";


      // Generate new trajectories
      // Update paths with startPlanning
      //updatePathsStart(startPlanning_);

      //std::vector<RampTrajectory> trajecs = getTrajectories(population_.paths_);
      std::vector<RampTrajectory> trajecs = replanTrajecs(population_.getTrajectories(), startPlanning_);

      ROS_INFO("trajecs.size(): %i", (int)trajecs.size());

      population_.replaceAll(trajecs);
      population_ = evaluatePopulation(population_);
      bestTrajec_ = population_.getBest();
    }




    // Call modification
    if(modifications_) {
      ModificationResult mod = modification();

      if(mod.i_modified_.size() > 0) {
        /*ROS_INFO("In checking for switches block");
        ROS_INFO("mod.popNew.size(): %i", (int)mod.popNew_.size());
        ROS_INFO("mod.i_modified.size(): %i", (int)mod.i_modified_.size());*/


        // Find the best of the modified trajectories
        uint16_t i_mostPromising = mod.i_modified_.at(0);
        for(uint8_t i=1;i<mod.i_modified_.size();i++) {
          if(mod.popNew_.get(i).msg_.fitness > mod.popNew_.get(i_mostPromising).msg_.fitness) {
            i_mostPromising = i;
          }
        } // end for
        //ROS_INFO("i_mostPromising: %i", (int)i_mostPromising);


        // Check if most promising is within fitness range of computing a switch
        tf::Vector3 v_linear( latestUpdate_.msg_.velocities.at(0),
                              latestUpdate_.msg_.velocities.at(1), 0);
        double mag_linear   = sqrt(tf::tfDot(v_linear, v_linear));
        double bestFitness  = bestTrajec_.msg_.fitness;
        double f            = mod.popNew_.get(i_mostPromising).msg_.fitness;
        //ROS_INFO("fitness: %f bestFitness: %f transThreshold_: %f diff: %f", f, bestFitness, transThreshold_, fabs(f-bestFitness));
        if( cc_started_         &&
            mag_linear > 0.0001 &&
            (f > bestFitness || fabs(f - bestFitness) < transThreshold_) )
        {
          ROS_INFO("In computing T_new");
          RampTrajectory T_new = computeFullSwitch(bestTrajec_, mod.popNew_.get(i_mostPromising));
          
          // Check if T_new better than current best trajectory
          if(compareSwitchToBest(T_new)) {
            ROS_INFO("T_new is better, switching trajectories");
            ROS_INFO("T_new: %s", T_new.toString().c_str());


            T_new.path_ = mod.popNew_.get(i_mostPromising).path_;

            population_.replace(i_mostPromising, T_new);
            bestTrajec_ = T_new;

            i_best_prev_ = i_mostPromising;
            
            double t;
            // if there's a curve, set next CC to start at the end of the curve
            if(T_new.transitionTraj_.curves.size() > 0) {
              trajectory_msgs::JointTrajectoryPoint j = T_new.transitionTraj_.trajectory.points.at(
               T_new.transitionTraj_.i_curveEnd); 
              t = j.time_from_start.toSec();
            }
            // start next CC at point where robot has stopped to turn
            else {
              trajectory_msgs::JointTrajectoryPoint j = T_new.transitionTraj_.trajectory.points.at(
                  T_new.transitionTraj_.i_knotPoints.at(2)); 
              t = j.time_from_start.toSec();
            }

            restartControlCycle(t);
            sendPopulation();

            num_switches_++;
          } // end if compareSwitchToBest
        } // end if should compute switch
      } // end if a modified trajec was added
    } // end if modifications


    

 
    // Finish up
    // t=t+1
    generation_++;
    c_pc_++;
    sendPopulation();
  
    ROS_INFO("Generation %i completed", (generation_-1));
  } // end if c_pc<genPerCC
} // End planningCycleCallback






/** This methed runs the tasks needed to do a control cycle */
void Planner::doControlCycle() {
  //ROS_INFO("In Planner::doControlCycle");
  
  ROS_INFO("best: %s", bestTrajec_.toString().c_str());
  ROS_INFO("Sending best");
  //pause();
  

  // TODO: set generationPerCC again each CC b/c of adaptive time
  // Send the best trajectory and set movingOn
  sendBest();

  ROS_INFO("Setting movingOn");
  movingOn_ = bestTrajec_.getSubTrajectory(controlCycle_.toSec());
  ROS_INFO("Done setting movingOn");

  // Reset planning cycle count
  c_pc_ = 0;

  // The motion state that we should reach by the next control cycle
  m_cc_ = bestTrajec_.getPointAtTime(controlCycle_.toSec());
  ROS_INFO("Done setting m_cc");

  // At CC, startPlanning is assumed to be perfect (no motion error accounted for yet)
  startPlanning_ = m_cc_;

  //ROS_INFO("bestTrajec: %s", bestTrajec_.toString().c_str());
  //ROS_INFO("New startPlanning_: %s", startPlanning_.toString().c_str());

  // After m_cc_ and startPlanning are set, adapt the population
  //ROS_INFO("Before adapt, pop: %s", population_.toString().c_str());
  population_ = adaptPopulation(population_, startPlanning_, controlCycle_);
  ROS_INFO("Done adapting population");
  population_ = evaluatePopulation(population_);
  bestTrajec_ = population_.getBest();
  //ROS_INFO("After adaptation and evaluation, pop: %s", population_.toString().c_str());

  // If error reduction
  // Set pop_orig_ and totalDiff to 0's
  if(errorReduction_) {
    totalDiff_ = totalDiff_.zero(3);
    pop_orig_ = population_;
  }
  
  // Create sub-populations if enabled
  if(subPopulations_) {
    population_.createSubPopulations();
  }
  
  // Send the population to trajectory_visualization
  sendPopulation();
  
  //ROS_INFO("Exiting Planner::doControlCycle");
} // End doControlCycle





/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\n************* Control cycle occurring *************\n";
  ROS_INFO("Control Cycle %i", num_controlCycles_);
  if(cc_started_) {
    ROS_INFO("Occurring at %fr, diff from prev: %f", 
        ros::Time::now().toSec(), 
        (ros::Time::now()-t_prevCC_).toSec());
  }
  t_prevCC_ = ros::Time::now();
  

  /*std::cout<<"\nstartPlanning: "<<startPlanning_.toString();
  std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString()<<"\n";*/
  
  // Check if the time is adaptive value
  if(controlCycle_.toSec() != t_fixed_cc_) {
    //ROS_WARN("controlCycle_ != t_fixed_cc, \n cc: %f t_fixed_cc_: %f", controlCycle_.toSec(), t_fixed_cc_);
    controlCycle_ = ros::Duration(t_fixed_cc_);
    controlCycleTimer_.setPeriod(controlCycle_);
  }
  
  
  // Uncertainty code
  //****SP_LU_diffs_.push_back(startPlanning_.subtract(latestUpdate_));
  
  // Do the control cycle
  doControlCycle();

  // Uncertainty code
  // Build m_i
  if(errorReduction_) {
    setMi();
  }
  

  // Set flag showing that CCs have started
  if(!cc_started_) {
    cc_started_ = true;
  }
  
  ROS_INFO("Control Cycle %i Ending, next one occurring in %f seconds", num_controlCycles_, controlCycle_.toSec());
  num_controlCycles_++;
} // End controlCycleCallback











/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/




/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {
  //ROS_INFO("Sending best trajectory: %s", bestTrajec_.toString().c_str());

  if(!stop_) {

    // If infeasible and too close to obstacle, 
    // Stop the robot by sending a blank trajectory
    if(!bestTrajec_.msg_.feasible && (bestTrajec_.msg_.t_firstCollision < 3.f)) {
      std::cout<<"\nCollision within 3 seconds! Stopping robot!\n";
    }
    else if(!bestTrajec_.msg_.feasible) {
      std::cout<<"\nBest trajectory is not feasible! Time until collision: "<<bestTrajec_.msg_.t_firstCollision;
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

    temp.findBestIndex();
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
  //ROS_INFO("In Planner::compareSwitchToBest");
  double bestFitness = bestTrajec_.msg_.fitness;

  //ROS_INFO("bestFitness before: %f", bestFitness);
  // fitness = 1/time so time = 1/fitness
  double t = 1. / bestFitness;
  double t_new = t + ((generationsPerCC_ - c_pc_) * planningCycle_.toSec());
  //ROS_INFO("t: %f t_new: %f c_pc: %i generationsPerCC_: %i", t, t_new, c_pc_, generationsPerCC_);

  // Best fitness adjusted for t_new
  bestFitness = 1. / t_new;

  //ROS_INFO("bestFitness after: %f", bestFitness);
  //ROS_INFO("traj.fitness: %f", traj.msg_.fitness);


  //ROS_INFO("Exiting Planner::compareSwitchToBest");
  return (traj.msg_.fitness > bestFitness);
}



/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
const RampTrajectory Planner::evaluateTrajectory(const RampTrajectory trajec) {
  //ROS_INFO("In Planner::evaluateTrajectory");

  RampTrajectory result = requestEvaluation(trajec);
  //ROS_INFO("result: %s", result.toString().c_str());

  //ROS_INFO("Leaving Planner::evaluateTrajectory");
  return result;
} // End evaluateTrajectory



/** 
 * This method evaluates each trajectory in the population
 * It also sets i_best_prev_
 **/
const Population Planner::evaluatePopulation(const Population pop) {
  //ROS_INFO("In Planner::evaluatePopulation");
  Population result = pop;
  
  // Go through each trajectory in the population and evaluate it
  for(uint16_t i=0;i<result.size();i++) {
    //ROS_INFO("i: %i", (int)i);
    result.replace(i, evaluateTrajectory(result.get(i)));
  } // end for
 

  i_best_prev_ = result.findBestIndex();
  
  //ROS_INFO("Exiting Planner::evaluatePopulation");
  return result;
} // End evaluatePopulation





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
   t_start_ = ros::Time::now();

  // t=0
  generation_ = 0;
  
  // initialize population
  initPopulation();
  std::cout<<"\n"<<population_.fitnessFeasibleToString();
  sendPopulation();
  std::cout<<"\nPop: "<<population_.toString();
  std::cout<<"\nPopulation initialized! Press enter to continue\n";
  std::cin.get();
 


  if(seedPopulation_) {
    std::cout<<"\nSeeding population\n";
    seedPopulation();
    i_best_prev_ = population_.findBestIndex();
    std::cout<<"\nPopulation seeded!\n";
    std::cout<<"\n"<<population_.fitnessFeasibleToString()<<"\n";
    std::cout<<"\n** Pop **:"<<population_.toString();
    // Evaluate after seeding
    population_ = evaluatePopulation(population_);
    bestTrajec_ = population_.getBest();
    movingOn_ = bestTrajec_.getSubTrajectory(controlCycle_.toSec());
    ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
    

    sendPopulation();
    std::cout<<"\nPopulation seeded! Press enter to continue\n";
    std::cin.get();
  }


  // Create sub-pops if enabled
  if(subPopulations_) {
    population_.createSubPopulations();
    std::cout<<"\nSub-populations created\n";
  }


  
  // Start the planning cycle timer
  planningCycleTimer_.start();

  // Wait for 100 generations before starting 
  while(generation_ < generationsBeforeCC_) {ros::spinOnce();}

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

  std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString()<<"\n";
  std::cout<<"\ngoal: "<<goal_.toString()<<"\n";
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();

  
  // Send an empty trajectory
  ramp_msgs::RampTrajectory empty;
  h_control_->send(empty);
  h_control_->send(empty);
  h_control_->send(empty);
  

  //std::cout<<"\nFinal population: ";
  //std::cout<<"\n"<<pathsToString(); 
  
  std::cout<<"\nNumber of trajectory switches: "<<num_switches_;
  std::cout<<"\nLeaving go\n";
} // End go
