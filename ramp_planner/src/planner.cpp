#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(5), D_(0.5f), 
  cc_started_(false), c_pc_(0), transThreshold_(1./50.), num_cc_(0), L_(0.33), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), 
  stop_(false) 
{
  controlCycle_ = ros::Duration(2.f);
  planningCycle_ = ros::Duration(1.f / 2.f);
  imminentCollisionCycle_ = ros::Duration(1.f / 2.f);
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
    Path temp_path = getRandomPath(init, goal);

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getRandomPaths


const std::vector<Path> Planner::getAdjustedPaths(const MotionState init, const MotionState goal) 
{
  std::vector<Path> result;

  // Create n random paths, where n=populationSize
  for(unsigned int i=0;i<populationSize_;i++) {
    
    // Create the path with the start and goal
    Path temp_path = getAdjustedPath(init, goal);

    // Add the path to the list of paths
    result.push_back(temp_path);
  } // end for create n paths

  return result;
} // End getAdjustedPaths




/*
 * Generate a path from s to g with completely random intermediate knot points (no constraints imposed)
 */
const Path Planner::getRandomPath(const MotionState s, const MotionState g) const
{
  Path result(s, g);
  

  // Each trajectory will have a random number of knot points
  // Put a max of 3 knot points for practicality...
  uint8_t n = (rand() % 3)+1;

  // Create n knot points 
  for(uint8_t i=0;i<n;i++) 
  {
    // Create a random configuration
    MotionState ms_temp;
    ms_temp = randomizeMSPositions(ms_temp);

    // Push on velocity values?
    // 
    
    result.addBeforeGoal(ms_temp);
  }

  return result;
} // End getRandomPath



/*
 * Return true if the MotionState ms satisfies constraints on Knot Points to be added to Path p
 */
const bool Planner::validKPForPath(const MotionState ms, const Path p) const
{
  bool result=true;

  // Check that ms_temp has a distance > L from all other knot points
  for(uint8_t i=0;i<p.size();i++)
  {
    if( sqrt( pow(ms.msg_.positions.at(0) - p.at(i).motionState_.msg_.positions.at(0), 2) +
              pow(ms.msg_.positions.at(1) - p.at(i).motionState_.msg_.positions.at(1), 2) ) < L_)
    {
      result = false;
      i = p.size();
    }
  }

  return result;
} // End validForKP



/*
 * Generate a path from s to g with intermediate knot points that are constrained
 */
const Path Planner::getAdjustedPath(const MotionState s, const MotionState g) const
{
  Path result(s, g);
  
  // Each trajectory will have a random number of knot points
  // Put a max of 3 knot points for practicality...
  uint8_t n = (rand() % 3)+1;

  while(result.size() < (n+2))
  {
    // Create a random configuration
    MotionState ms_temp;
    ms_temp = randomizeMSPositions(ms_temp);

    // Check that it satisfies any constraints on KPs
    if(validKPForPath(ms_temp, result))
    {
      result.addBeforeGoal(ms_temp);
    }
  } // end while

  return result;
} // End getAdjustedPath




const Population Planner::getPopulation( const MotionState init, const MotionState goal, const bool random)
{
  Population result;

  // Set the size
  result.maxSize_ = populationSize_;

  // Get some random paths
  std::vector<Path> paths = random ?  getRandomPaths  (init, goal)  : 
                                      getAdjustedPaths(init, goal)  ;

  // Get trajectories for the paths
  std::vector<RampTrajectory> trajecs = getTrajectories(paths);

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

  //ROS_INFO("Exiting Planner::getRandomPopulation");
  return result;
} // End getPopulation



const uint8_t Planner::getIndexStartPathAdapting(const RampTrajectory t) const {
  uint8_t result;
  bool    has_curve = t.msg_.curves.size() > 0;

  if(t.transitionTraj_.trajectory.points.size() > 0) {
    result = t.transitionTraj_.i_knotPoints.size();
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
    ROS_INFO("i_kp: %i", (int)i_kp);

    // Get the knot point 
    trajectory_msgs::JointTrajectoryPoint point = traj.msg_.trajectory.points.at( 
                                                    traj.msg_.i_knotPoints.at(i_kp));
    ROS_INFO("point: %s", utility_.toString(point).c_str());

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
const std::vector<Path> Planner::adaptPaths(const Population pop, const MotionState start, ros::Duration dur) const {
  ROS_INFO("In Planner::adaptPaths");
  std::vector<Path> result;

  // Check that time has passed
  if(dur.toSec() > 0) {


    // For each trajectory
    for(uint8_t i=0;i<pop.size();i++) {
      //ROS_INFO("Path: %s", pop.paths_.at(i).toString().c_str());
      //ROS_INFO("Get Path: %s", pop.get(i).getPath().toString().c_str());
      Path temp = pop.paths_.at(i);

      // Track how many knot points we get rid of
      // Initialize to 1 to always remove starting position
      unsigned int throwaway=getNumThrowawayPoints(pop.get(i), dur);
      //ROS_INFO("throwaway: %i", (int)throwaway);

      
      // If the whole path has been passed, adjust throwaway so that 
      //  we are left with a path that is: {new_start_, goal_}
      if( throwaway >= pop.paths_.at(i).size() ) { 
        //ROS_INFO("Decrementing throwaway");
        throwaway = pop.paths_.at(i).size()-1;
      }


      // Print the knot points being removed
      /*for(int c=0;c<throwaway;c++) {
        ROS_INFO("\nRemoving point: %s", (*(pop.paths_.at(i).all_.begin()+c)).toString().c_str());
      }*/

      // Erase the amount of throwaway points (points we have already passed)
      temp.all_.erase( 
          temp.all_.begin(), 
          temp.all_.begin()+throwaway );

      // Insert the new starting configuration
      temp.all_.insert( temp.all_.begin(), start);

      // Set start_ to be the new starting configuration of the path
      temp.start_ = start;
      ROS_INFO("After adapting Path: %s", temp.toString().c_str());

      result.push_back(temp);
    } // end outer for
  } // end if dur > 0

  ROS_INFO("Exiting adaptPaths");
  return result;
} // End adaptPaths





// 1 if before curve, 2 if on curve, 3 if past curve 
// TODO: Check for the 2nd segment as well?
const int Planner::estimateIfOnCurve(const MotionState ms, const ramp_msgs::BezierInfo curve) const {
  ROS_INFO("In estimateIfOnCurve");
  ROS_INFO("ms: %s", ms.toString().c_str());
  ROS_INFO("curve: %s", utility_.toString(curve).c_str());

  double x = ms.msg_.positions.at(0);
  double y = ms.msg_.positions.at(1);

  bool xSlope     = (curve.segmentPoints.at(1).positions.at(0) - curve.segmentPoints.at(0).positions.at(0) > 0);
  bool xSlopeTwo  = (curve.segmentPoints.at(2).positions.at(0) - curve.segmentPoints.at(1).positions.at(0) > 0);
  bool ySlope     = (curve.segmentPoints.at(1).positions.at(1) - curve.segmentPoints.at(0).positions.at(1) > 0);
  bool ySlopeTwo  = (curve.segmentPoints.at(2).positions.at(1) - curve.segmentPoints.at(1).positions.at(1) > 0);
  
  bool xSegOne =  xSlope ?  (x >= curve.controlPoints.at(0).positions.at(0)) &&
                            (x <= curve.controlPoints.at(1).positions.at(0)) :
                            (x <= curve.controlPoints.at(0).positions.at(0)) &&
                            (x >= curve.controlPoints.at(1).positions.at(0));

  bool xSegTwo =  xSlopeTwo ?   (x >= curve.controlPoints.at(1).positions.at(0)) &&
                                (x <= curve.controlPoints.at(2).positions.at(0)) :
                                (x <= curve.controlPoints.at(1).positions.at(0)) &&
                                (x >= curve.controlPoints.at(2).positions.at(0));

  
  bool ySegOne =  ySlope ?  (y >= curve.controlPoints.at(0).positions.at(1)) &&
                            (y <= curve.controlPoints.at(1).positions.at(1)) :
                            (y <= curve.controlPoints.at(0).positions.at(1)) &&
                            (y >= curve.controlPoints.at(1).positions.at(1));

  bool ySegTwo =  ySlopeTwo ?   (y >= curve.controlPoints.at(1).positions.at(1)) &&
                                (y <= curve.controlPoints.at(2).positions.at(1)) :
                                (y <= curve.controlPoints.at(1).positions.at(1)) &&
                                (y >= curve.controlPoints.at(2).positions.at(1));


  //ROS_INFO("xSegOne: %s xSegTwo: %s ySegOne: %s ySegTwo: %s", xSegOne ? "True" : "False", xSegTwo ? "True" : "False", ySegOne ? "True" : "False", ySegTwo ? "True" : "False");

  bool xGood = (xSegOne || xSegTwo);
  bool yGood = (ySegOne || ySegTwo);

  if(xGood && yGood) {
    ROS_INFO("Returning 2 (on curve)");
    return 2;
  }
  
  // Check if past curve
  bool xPast = xSlopeTwo ?  x > curve.controlPoints.at(2).positions.at(0) :
                            x < curve.controlPoints.at(2).positions.at(0) ; 
  // Check if past curve
  bool yPast = xSlopeTwo ?  y > curve.controlPoints.at(2).positions.at(1) :
                            y < curve.controlPoints.at(2).positions.at(1) ; 

  ROS_INFO("xPast: %s yPast: %s", xPast ? "True" : "False", yPast ? "True" : "False");
  if(xPast || yPast)
  {
    ROS_INFO("Returning 3 (after curve)");
    return 3;
  }

  // Else, robot has not reached curve, return 1
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
    ROS_INFO("Curve 0 has ended, new curve: %s", utility_.toString(result).c_str());
    if(estimateIfOnCurve(startPlanning_, result) == 2) {
      ROS_INFO("Adding .000001");
      result.u_0 += 0.000001;
      result.ms_begin = startPlanning_.msg_;
    }
    else {
      result.u_0 += 0.;
    }
  }

  // Else if there was only 1 curve, just return a blank one

  return result;
} // End handleCurveEnd



const double Planner::updateCurvePos(const RampTrajectory traj, const ros::Duration d) const {
  //ROS_INFO("In Planner::updateCurvePos");
  //ROS_INFO("startPlanning_: %s", startPlanning_.toString().c_str());
  //ROS_INFO("traj: %s", traj.toString().c_str());
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
    double t = d.toSec() - t_s0;

    ROS_INFO("t: %f Adding %f", t, (t*curve.u_dot_0));
    

    // At one point I changed this to the average of x and y scaled
    // But my notes don't say why...
    // But that change made a robot's prediction along a curve noticeably more inaccurate
    //std::vector<double> scaled = getScaledXY(startPlanning_, traj.msg_.curves.at(0));
    //ROS_INFO("Adding %f",(scaled.at(0) + scaled.at(1)) / 2.);
    //result += (scaled.at(0) + scaled.at(1)) / 2.;
    result += t * curve.u_dot_0;
  } // end if already on curve

  // Else, u_0 > 0, simply add to u_0
  else {
    //ROS_INFO("Else not moving on curve, curve.u_0 > 0: %f curve.u_dot_0: %f", curve.u_0, curve.u_dot_0);
    result += curve.u_dot_0 * d.toSec();
  }

  return result;
} // End updateCurvePos


/** Updates the curve u_0 and ms_begin */
// TODO: Only need to update the bestTrajec's curve
const std::vector<ramp_msgs::BezierInfo> Planner::adaptCurves(const Population pop, const MotionState ms, const ros::Duration d) const {
  ROS_INFO("In Planner::adaptCurves");

  std::vector<ramp_msgs::BezierInfo> result;
  ramp_msgs::BezierInfo blank;

  // Go through each trajectory 
  for(uint16_t i=0;i<pop.size();i++) 
  {
    ROS_INFO("Curve %i", (int)i);
    
    // If the trajectory has a curve
    if(pop.get(i).msg_.curves.size() > 0) 
    {
      ROS_INFO("In if trajectory has curve");

      // Set curve
      ramp_msgs::BezierInfo curve = pop.get(i).msg_.curves.at(0); 
      ROS_INFO("Set curve to: %s", utility_.toString(curve).c_str());

      ROS_INFO("pop.getBestIndex: %i", (int)pop.calcBestIndex());
      // If moving on this curve, update u
      if( i == pop.calcBestIndex() && 
            (curve.u_0 > 0 ||
             estimateIfOnCurve(ms, curve) == 2))
      {
        ROS_INFO("Moving on this curve");

        // Get the new u_0 value
        curve.u_0 = updateCurvePos(pop.get(i), d);

        // Set the new ms_begin
        curve.ms_begin = ms.msg_;
      }  //end if moving on curve

     
      /* Separate checking if on the curve and if done with curve
           because we could be done before ever incrementing u_0 */
      // Check if done with current curve
      if(curve.u_0 > curve.u_target || estimateIfOnCurve(ms, curve) == 3) {
        ROS_INFO("Done with curve, u_0: %f", curve.u_0);
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

  ROS_INFO("Exiting Planner::adaptCurves");
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
  std::vector<Path> paths                   = adaptPaths(pop, ms, d);
  std::vector<ramp_msgs::BezierInfo> curves = adaptCurves(pop, ms, d);
  
  result.paths_ = paths;

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  // For each path, get a trajectory
  for(uint16_t i=0;i<pop.paths_.size();i++) {
    RampTrajectory tempTraj = pop.get(i);
    ROS_INFO("Getting trajectory %i", (int)i);

    std::vector<ramp_msgs::BezierInfo> c;
    c.push_back(curves.at(i));
    
    ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(paths.at(i), c);

    /* Get the trajectory */
    RampTrajectory temp = requestTrajectory(tr, result.get(i).msg_.id);

    // Set temporary evaluation results - need to actually call requestEvaluation to get actual fitness
    temp.msg_.fitness   = result.get(i).msg_.fitness;
    temp.msg_.feasible  = result.get(i).msg_.feasible;

    // Push onto updatedTrajecs
    updatedTrajecs.push_back(temp);
  } // end for

  ROS_INFO("updatedTrajecs size: %i", (int)updatedTrajecs.size());
  // Replace the population's trajectories_ with the updated trajectories
  result.replaceAll(updatedTrajecs);
  
  //ROS_INFO("Done adapting, pop now: %s", population_.toString().c_str());
  ROS_INFO("Exiting adaptPopulation");

  return result;
} // End adaptPopulation





// TODO: Clean up
/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<ramp_msgs::BezierInfo> curves, const int id) const {
  //ROS_INFO("In buildTrajectoryRequest");
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
        
        temp.segmentPoints.push_back( path.all_.at(0).motionState_.msg_ );
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


  //ROS_INFO("Exiting Planner::buildTrajectoryRequest");
  return result;
} // End buildTrajectoryRequest


const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const int id) const {
  std::vector<ramp_msgs::BezierInfo> curves;
  return buildTrajectoryRequest(path, curves, id);
}





/** Build an EvaluationRequest srv */
const ramp_msgs::EvaluationRequest Planner::buildEvaluationRequest(const RampTrajectory trajec) {
  ramp_msgs::EvaluationRequest result;

  result.request.trajectory   = trajec.msg_;
  result.request.currentTheta = latestUpdate_.msg_.positions.at(2);
  result.request.t_start      = controlCycle_.toSec();

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
 

  if(!population_.get(population_.calcBestIndex()).msg_.feasible && 
      (population_.get(population_.calcBestIndex()).msg_.t_firstCollision < D_)) 
  {
    ROS_WARN("Imminent Collision Robot: %i t_firstCollision: %f", id_, 
        population_.get(population_.calcBestIndex()).msg_.t_firstCollision);
    h_parameters_.setImminentCollision(true); 
  } 
  else 
  {
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






/** This method sets random values for the position vector of ms */
const MotionState Planner::randomizeMSPositions(const MotionState ms) const {
  MotionState result = ms;
  result.msg_.positions.clear();

  for(unsigned int i=0;i<ranges_.size();i++) 
  {
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
  //all.push_back(kp);
  //all.push_back(kp1);
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
  //paths.push_back(p1);
  /************************************/

  /**** Get trajectories ****/  
  std::vector<RampTrajectory> new_pop;
  for(uint8_t i=0;i<paths.size();i++) {
  
    // Make request
    RampTrajectory trajec = requestTrajectory(paths.at(i));
    trajec = requestEvaluation(trajec);
    ROS_INFO("Seede trajec: %s", trajec.toString().c_str());
    population_.add(trajec); 
  
  } // end for
  /************************************/

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


void Planner::setMi() {
  ROS_INFO("In Planner::setMi");

  // Clear m_i
  m_i_.clear();
  
 
  // Set m_i
  // Each m_i will be start + (delta_m_inc * i)
  for(int i=0;i<generationsPerCC_;i++) {
    MotionState temp = movingOn_.getPointAtTime(planningCycle_.toSec()*i);
    
    m_i_.push_back(temp);

    ROS_INFO("m_i[%i]: %s", i, temp.toString().c_str());
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
  population_ = getPopulation(latestUpdate_, goal_, true);
} // End init_population




const bool Planner::checkIfSwitchCurveNecessary(const RampTrajectory from, const RampTrajectory to) const {
  ROS_INFO("In Planner::CheckIfSwitchCurveNecessary");
  // TODO: lastestUpdate or from's first theta?
  double thetaToSwitch, thetaCurrent = latestUpdate_.msg_.positions.at(2);

  ROS_INFO("to.msg.trajectory.points.size(): %i", (int)to.msg_.trajectory.points.size());
  ROS_INFO("to.msg.i_knotPoints.size(): %i", (int)to.msg_.i_knotPoints.size());
  if(to.msg_.i_knotPoints.size() > 1) {
    ROS_INFO("to.msg.i_knotPoints.at(1): %i", (int)to.msg_.i_knotPoints.at(1));
  }

  int kp = 1; 
  // Check if 1st two positions in "to" trajectory are the same
  // If they are, the 2nd kp is the end of a rotation
  // Theta after rotating will tell us the theta needed to move on "to" trajectory
  if( to.msg_.i_knotPoints.size() > 1 && fabs(utility_.positionDistance( to.msg_.trajectory.points.at(
                                            to.msg_.i_knotPoints.at(0)).positions,
                                            to.msg_.trajectory.points.at(
                                            to.msg_.i_knotPoints.at(kp)).positions)) < 0.0001)
  {
    //ROS_INFO("In if positions are the same");
    thetaToSwitch = to.msg_.trajectory.points.at( to.msg_.i_knotPoints.at(1) ).positions.at(2);
  }
  else if( to.msg_.i_knotPoints.size() > 1) {
    //ROS_INFO("In else positions are not the same");
    thetaToSwitch = utility_.findAngleFromAToB(
                          to.msg_.trajectory.points.at(0), 
                          to.msg_.trajectory.points.at(
                            to.msg_.i_knotPoints.at(kp)) ); 
  }
  else 
  {
    ROS_INFO("to trajec: %s", to.toString().c_str());
    thetaToSwitch = to.msg_.trajectory.points.at(0).positions.at(2);
  }


  ROS_INFO("thetaCurrent: %f thetaToSwitch: %f", thetaCurrent, thetaToSwitch);
  ROS_INFO("fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)): %f",
        fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)));


  // If a difference of 1 degree, compute a curve
  if(fabs(utility_.findDistanceBetweenAngles(thetaCurrent, thetaToSwitch)) > 0.017) {
    //ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning true");
    return true;
  }

  ROS_INFO("Exiting Planner::CheckIfSwitchCurveNecessary, returning false");
  return false;
}


const RampTrajectory Planner::computeFullSwitch(const RampTrajectory from, const RampTrajectory to) {
  ROS_INFO("In Planner::computeFullSwitch");
  RampTrajectory result;

  if(checkIfSwitchCurveNecessary(from, to)) {
    ROS_INFO("Orientation needs to change");

    // Get transition trajectory
    std::vector<RampTrajectory> trajecs = switchTrajectory(from, to);

    // If a switch was possible
    if(trajecs.size() > 0)
    {
      RampTrajectory T_new = trajecs.at(1);

      // Evaluate T_new
      ramp_msgs::EvaluationRequest er = buildEvaluationRequest(T_new);
      er.request.t_start = planningCycle_.toSec();
      T_new = requestEvaluation(er);
      T_new.transitionTraj_ = trajecs.at(0).msg_;

      result = T_new;
      result.transitionTraj_ = trajecs.at(0).msg_;
    }
  } // end if curve needed

  // If no switch is necessary, return a blank trajectory
  // because no switching-specific motion is needed
  else {
    ROS_INFO("No curve needed to switch");
  }

  ROS_INFO("Exiting Planner::computeFullSwitch");
  return result;
}


const std::vector<RampTrajectory> Planner::switchTrajectory(const RampTrajectory from, const RampTrajectory to) {
  ROS_INFO("In Planner::switchTrajectory");
  std::vector<RampTrajectory> result;

    
  RampTrajectory switching = getTransitionTrajectory(from, to);
  if(switching.msg_.trajectory.points.size() < 1) 
  {
    return result;
  }
  RampTrajectory full = switching.clone();
  ROS_INFO("Switching trajectory: %s", switching.toString().c_str());
  
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
  

  ROS_INFO("Exiting Planner::switchTrajectory");
  return result;
} // End switchTrajectory





const RampTrajectory Planner::getTransitionTrajectory(const RampTrajectory current, const RampTrajectory trgt_traj) {
  ROS_INFO("In Planner::getTransitionTrajectory");
  ROS_INFO("Current: %s", current.toString().c_str());

  double t = (c_pc_+1)*planningCycle_.toSec();
  //ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
  
  MotionState ms_startSwitch = current.getPointAtTime(t);
  ROS_INFO("ms_startSwitch: %s", ms_startSwitch.toString().c_str());
  
  std::vector<MotionState> segment_points;
  segment_points.push_back(ms_startSwitch);
  segment_points.push_back(
      current.msg_.trajectory.points.at(current.msg_.trajectory.points.size()-1));



 
  // 2nd knot point should be the initial point on that trajectory's bezier 
  // Using start of Bezier rather than segment endpoint ensures that
  // the trajectory will end at the start of the Bezier
  int i_goal = 1;
  if(trgt_traj.msg_.i_knotPoints.size() == 1) 
  {
    ROS_ERROR("trgt_traj.msg_.i_knotPoints.size() == 1, trgt_traj: %s", trgt_traj.toString().c_str());
    i_goal = 0;
  }

  else if(trgt_traj.msg_.i_knotPoints.size() > 2 && 
      utility_.positionDistance( trgt_traj.path_.start_.motionState_.msg_.positions, trgt_traj.msg_.trajectory.points.at(trgt_traj.msg_.i_knotPoints.at(1)).positions ) < 0.1)
  {
    i_goal = 2;
  }
  
  ROS_INFO("i_goal: %i, trgt.knotpoints.size(): %i", i_goal, (int)trgt_traj.msg_.i_knotPoints.size());
  MotionState g(trgt_traj.msg_.trajectory.points.at(trgt_traj.msg_.i_knotPoints.at(i_goal)));
  segment_points.push_back(g);
  ROS_INFO("g: %s", g.toString().c_str());


  double firstSlope = utility_.findAngleFromAToB(segment_points.at(0).msg_.positions,
      segment_points.at(1).msg_.positions);
  ROS_INFO("firstSlope: %f", firstSlope);

  // Check that the robot has orientation needed to move on first segment
  if( fabs(utility_.findDistanceBetweenAngles( ms_startSwitch.msg_.positions.at(2),
        firstSlope)) > 0.08)
  {
    ROS_WARN("Transition not possible because robot is not oriented towards first segment");
    RampTrajectory blank;
    return blank;
  }




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
} // End getTransitionTrajectory






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
  
  if(h_eval_req_->request(er)) 
  {
    result.msg_.fitness           = er.response.fitness;
    result.msg_.feasible          = er.response.feasible;
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
  result.path_                = traj.path_;
  result.bezierPath_          = traj.bezierPath_;
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
  ROS_INFO("In Planner::modification()");
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
      ROS_INFO("Modification Trajectory added to population at index: %i", index);
      ROS_INFO("Modded Trajec %i: %s", i, mod_trajec.at(i).toString().c_str());
      result.i_modified_.push_back(index);
    }
    else {
      ROS_INFO("Modification Trajectory not added to population");
    }

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) 
    {
      population_.createSubPopulations();
    }
  } // end for


  // Find the best trajectory
  ROS_INFO("Finding pop's best index");
  int index   = population_.calcBestIndex();

  result.popNew_ = population_;

  //ROS_INFO("After modification, pop now: %s", result.popNew_.toString().c_str());

  ROS_INFO("Exiting Planner::modification");
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





/*const MotionState Planner::predictStartPlanning() const {
  ROS_INFO("In Planner::predictStartPlanning");

  MotionState result;

  ROS_INFO("c_pc: %i", (int)c_pc_);
  ROS_INFO("m_i.size(): %i", (int)m_i_.size());
 
  ROS_INFO("m_i[%i]: %s", c_pc_, m_i_.at(c_pc_).toString().c_str());
  ROS_INFO("latestUpdate_: %s", latestUpdate_.toString().c_str());
  
  // Get the difference between robot's state and what state it should be at
  MotionState diffI = m_i_.at(c_pc_).subtractPosition(latestUpdate_);
  //std::cout<<"\ndiff: "<<diff.toString();

  MotionState diff = diffI.subtractPosition(totalDiff_);
  
  ROS_INFO("m_cc: %s\ndiffI: %s\ntotalDiff: %s\ndiff: %s", 
      m_cc_.toString().c_str(),
      diffI.toString().c_str(),
      totalDiff_.toString().c_str(),
      diff.toString().c_str());

  // subtractPositioin that difference from startPlanning
  //result = startPlanning_.subtractPositioin(diff);
  result = m_cc_.subtractPosition(diffI);


  ROS_INFO("Exiting Planner::predictStartPlanning");
  return result;
}*/



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
  ROS_INFO("********************************************************************");
  ROS_INFO("********************************************************************");
  ROS_INFO("********************************************************************");
  ROS_INFO("Planning cycle occurring, generation %i", generation_);
  ROS_INFO("********************************************************************");
  ROS_INFO("********************************************************************");
  ROS_INFO("********************************************************************");
  

  // Make sure not too many PC occur before next CC
  if(c_pc_ < generationsPerCC_ || !cc_started_) {




    /*if(errorReduction_ && cc_started_ && latestUpdate_.msg_.velocities.at(2) < 0.01) {
      // Update startPlanning
      MotionState ms_sp = predictStartPlanning();
      ROS_INFO("ms_sp: %s", ms_sp.toString().c_str());
      ROS_INFO("startPlanning_.subtractPosition(ms_sp): %s", 
          startPlanning_.subtractPosition(ms_sp).toString().c_str());

      startPlanning_ = ms_sp;
      ROS_INFO("After predicting, startPlanning_: %s", startPlanning_.toString().c_str());

      std::vector<RampTrajectory> trajecs = replanTrajecs(population_.getTrajectories(), startPlanning_);

      ROS_INFO("trajecs.size(): %i", (int)trajecs.size());

      population_.replaceAll(trajecs);
      //population_ = evaluatePopulation(population_);
      //bestTrajec_ = population_.getBest();
    }*/




    // Call modification
    if(modifications_) 
    {
      ROS_INFO("*****************************");
      ROS_INFO("Performing modification");
      ModificationResult mod = modification();
      ROS_INFO("Done with modification");
      ROS_INFO("*****************************");

      ROS_INFO("Evaluating entire population!");
      population_ = mod.popNew_;
      population_ = evaluatePopulation(population_);
      ROS_INFO("After mod and eval, pop now: %s", population_.toString().c_str());


      if(cc_started_) 
      {
        ROS_INFO("In checking for switches block, c_pc: %i", c_pc_);
        ROS_INFO("mod.i_modified.size(): %i", (int)mod.i_modified_.size());


      } // end if a modified trajec was added
      else {
        ROS_INFO("cc_started_: %s", cc_started_ ? "True" : "False");
        ROS_INFO("i_modified.size(): %i", (int)mod.i_modified_.size());
        ROS_INFO("Not checking for switch because trajectory was not added to the population");
      }
    } // end if modifications


    

 
    // Finish up
    // t=t+1
    generation_++;
    c_pc_++;
    sendPopulation();
  
    ROS_INFO("********************************************************************");
    ROS_INFO("********************************************************************");
    ROS_INFO("********************************************************************");
    ROS_INFO("Generation %i completed", (generation_-1));
    ROS_INFO("********************************************************************");
    ROS_INFO("********************************************************************");
    ROS_INFO("********************************************************************");
  } // end if c_pc<genPerCC
} // End planningCycleCallback






/** This methed runs the tasks needed to do a control cycle */
void Planner::doControlCycle() {
  ROS_INFO("In Planner::doControlCycle");

  // Set the bestT
  RampTrajectory bestT = population_.get(population_.calcBestIndex());

  
  // Send the best trajectory and set movingOn
  ROS_INFO("Sending best");
  ROS_INFO("bestT: %s", bestT.toString().c_str());
  sendBest();

  
  // Set gensPerCC based on CC time
  generationsPerCC_ = controlCycle_.toSec() / planningCycle_.toSec();
  ROS_INFO("CC Time: %f PC Time: %f generationsPerCC_: %i", 
      controlCycle_.toSec(), 
      planningCycle_.toSec(),
      generationsPerCC_);



  ROS_INFO("Setting movingOn_");
  movingOn_ = bestT.getSubTrajectory(controlCycle_.toSec());


  // Reset planning cycle count
  c_pc_ = 0;


  // The motion state that we should reach by the next control cycle
  m_cc_ = bestT.getPointAtTime(controlCycle_.toSec());

  // At CC, startPlanning is assumed to be perfect (no motion error accounted for yet)
  startPlanning_ = m_cc_;
  ROS_INFO("New startPlanning_: %s", startPlanning_.toString().c_str());

  // After m_cc_ and startPlanning are set, adapt the population
  ROS_INFO("Before adaptation and evaluation, pop size: %i pop: %s\nDone printing pop", 
      population_.size(), 
      population_.toString().c_str());

  population_ = adaptPopulation(population_, startPlanning_, controlCycle_);
  population_ = evaluatePopulation(population_);
  ROS_INFO("After adaptation and evaluation, pop size: %i pop: %s\nDone printing pop", 
      population_.size(), 
      population_.toString().c_str());
  


  // If error reduction
  // Set pop_orig_ and totalDiff to 0's
  if(errorReduction_) 
  {
    setMi();
  }
  
  // Create sub-populations if enabled
  if(subPopulations_) 
  {
    population_.createSubPopulations();
  }
  
  // Send the population to trajectory_visualization
  sendPopulation();

  
  ROS_INFO("Exiting Planner::doControlCycle");
} // End doControlCycle





/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\n************* Control cycle occurring *************\n";
  
  // Check if the time is adaptive value
  if(controlCycle_.toSec() != t_fixed_cc_) 
  {
    controlCycle_ = ros::Duration(t_fixed_cc_);
    controlCycleTimer_.setPeriod(controlCycle_);
  }
  
  
  // Do the control cycle
  doControlCycle();


  // Set flag showing that CCs have started
  if(!cc_started_) 
  {
    cc_started_ = true;
  }
  
  ROS_INFO("Control Cycle %i Ending, next one occurring in %f seconds", num_cc_, controlCycle_.toSec());
  num_cc_++;
} // End controlCycleCallback











/*******************************************************
 ******************** Miscellaneous ********************
 *******************************************************/




/** Send the fittest feasible trajectory to the robot package */
void Planner::sendBest() {
  //ROS_INFO("Sending best trajectory: %s", population_.get(population_.calcBestIndex()).toString().c_str());

  if(!stop_) {
    RampTrajectory best = population_.get(population_.calcBestIndex());
    //RampTrajectory best = bestTrajec_;

    // If infeasible and too close to obstacle, 
    // Stop the robot by sending a blank trajectory
    if(!best.msg_.feasible && (best.msg_.t_firstCollision < 3.f)) 
    {
      std::cout<<"\nCollision within 3 seconds! Stopping robot!\n";
    }
    else if(best.msg_.feasible) {
      ROS_INFO("Best trajectory is not feasible! Time until collision: %f", best.msg_.t_firstCollision);
    }
    
    h_control_->send(best.msg_);
  } // end if not stopped
  else {
    ROS_INFO("Sending blank!");
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

    temp.calcBestIndex();
    msg = temp.populationMsg();
  }
  else {
    msg = population_.populationMsg();
  }

  msg.robot_id = id_;

  msg.population.push_back(movingOn_.msg_);
  h_control_->sendPopulation(msg);
}

void Planner::displayTrajectory(const ramp_msgs::RampTrajectory traj) const {
  ramp_msgs::Population pop;
  pop.population.push_back(traj);
  h_control_->sendPopulation(pop);
}



/** Returns true if traj's fitness is better than the best fitness */
const bool Planner::compareSwitchToBest(const RampTrajectory traj, const Population pop) const {
  ROS_INFO("In Planner::compareSwitchToBest");
  
  RampTrajectory bestT = pop.getBest();

  ROS_INFO("Comparing against best: %s", bestT.fitnessFeasibleToString().c_str());
  double bestFitness = bestT.msg_.fitness;

  ROS_INFO("bestFitness before: %f", bestFitness);
  // fitness = 1/time so time = 1/fitness
  double t = bestT.msg_.trajectory.points.at(
      bestT.msg_.trajectory.points.size()-1).time_from_start.toSec();
  double t_new = t + ((generationsPerCC_ - c_pc_) * planningCycle_.toSec());
  ROS_INFO("t: %f t_new: %f c_pc: %i generationsPerCC_: %i", t, t_new, c_pc_, generationsPerCC_);

  // Normalize 
  t_new /= 50.;
  ROS_INFO("After normalizing, t_new: %f", t_new);

  if(!bestT.msg_.feasible) 
  {
    t_new += (1000. / bestT.msg_.t_firstCollision);
  }

  // Best fitness adjusted for t_new
  bestFitness = 1. / t_new;

  ROS_INFO("bestFitness after: %f", bestFitness);
  ROS_INFO("traj.fitness: %f", traj.msg_.fitness);


  ROS_INFO("Exiting Planner::compareSwitchToBest, returning: %s", traj.msg_.fitness > bestFitness ? "True" : "False");
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
 

  i_best_prev_ = result.calcBestIndex();
  
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
    i_best_prev_ = population_.calcBestIndex();
    std::cout<<"\nPopulation seeded!\n";
    std::cout<<"\n"<<population_.fitnessFeasibleToString()<<"\n";
    std::cout<<"\n** Pop **:"<<population_.toString();

    // Evaluate after seeding
    population_ = evaluatePopulation(population_);

    // Set movingOn
    movingOn_ = population_.get(population_.calcBestIndex()).getSubTrajectory(controlCycle_.toSec());
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

  // Set bestTrajec before starting initial CC
  //bestTrajec_ = population_.get(population_.calcBestIndex());

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

  ROS_INFO("Planning done!");
  ROS_INFO("latestUpdate_: %s\ngoal: %s", latestUpdate_.toString().c_str(), goal_.toString().c_str());
  
  // Stop timer
  controlCycleTimer_.stop();
  planningCycleTimer_.stop();
  imminentCollisionTimer_.stop();

  
  // Send an empty trajectory
  ramp_msgs::RampTrajectory empty;
  h_control_->send(empty);
  h_control_->send(empty);
  h_control_->send(empty);
  
  
  ROS_INFO("Total number of planning cycles: %i", generation_-1);
  ROS_INFO("Total number of control cycles:  %i", num_cc_);
  ROS_INFO("Exiting Planner::go");
} // End go
