#include "planner.h"


/*****************************************************
 ************ Constructors and destructor ************
 *****************************************************/

Planner::Planner() : resolutionRate_(1.f / 10.f), generation_(0), i_rt(1), goalThreshold_(0.4), num_ops_(5), D_(3.f), cc_started_(false), c_pc_(0), transThreshold_(1/20.), h_traj_req_(0), h_eval_req_(0), h_control_(0), modifier_(0), stop_(false), num_controlCycles_(0) 
{
  controlCycle_ = ros::Duration(2.f);
  planningCycle_ = ros::Duration(1.f / 5.f);
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
    unsigned int num = rand() % 3;
  

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
  result = evaluatePopulation(result, false);

  //ROS_INFO("Exiting Planner::randomPopulation");
  return result;
} // End randomPopulation



const uint8_t Planner::getIndexStartPathAdapting(const RampTrajectory t) const {
  uint8_t result;
  bool    has_curve = t.msg_.curves.size() > 0;

  if(has_curve && t.msg_.curves.size() == 2) {
    result = bestTrajec_.transitionTraj_.i_knotPoints.size()-1;
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


/** 
 * This method updates all the paths with the current configuration 
 * For each knot point in a path, it will remove the knot point if
 * its time_from_start is <= the Duration argument
 * */
void Planner::adaptPaths(MotionState start, ros::Duration dur) {
  //ROS_INFO("In Planner::adaptPaths");

  // Check that time has passed
  if(dur.toSec() > 0) {


    // For each trajectory
    for(uint8_t i=0;i<population_.size();i++) {
      //ROS_INFO("Path: %s", population_.paths_.at(i).toString().c_str());
      //ROS_INFO("Get Path: %s", population_.get(i).getPath().toString().c_str());

      // Track how many knot points we get rid of
      // Initialize to 1 to always remove starting position
      unsigned int throwaway=1;

      // For each knot point,
      // Start at 2 because that is the end of the first bezier curve
      for(uint8_t i_kp=getIndexStartPathAdapting(population_.get(i));
          i_kp<population_.get(i).msg_.i_knotPoints.size();
          i_kp++) 
      {
        //ROS_INFO("i_kp: %i", (int)i_kp);
        //ROS_INFO("population_.get(%i).curves.size(): %i", (int)i_kp, (int)population_.get(i).msg_.curves.size());

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
      } // end inner for

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
      population_.paths_.at(i).all_.erase( 
          population_.paths_.at(i).all_.begin(), 
          population_.paths_.at(i).all_.begin()+throwaway );

      // Insert the new starting configuration
      population_.paths_.at(i).all_.insert( population_.paths_.at(i).all_.begin(), start);

      // Set start_ to be the new starting configuration of the path
      population_.paths_.at(i).start_ = start;
      ROS_INFO("After adapting Path: %s", population_.paths_.at(i).toString().c_str());
    } // end outer for
  } // end if dur > 0

  //ROS_INFO("Exiting adaptPaths");
} // End adaptPaths




// 1 if before curve, 2 if on curve, 3 if past curve 
// TODO: Check for the 2nd segment as well?
const int Planner::estimateIfOnCurve(const MotionState ms, const ramp_msgs::BezierInfo curve) const {
  //ROS_INFO("In estimateIfOnCurve");
  //ROS_INFO("ms: %s", ms.toString().c_str());
  //ROS_INFO("curve: %s", utility_.toString(curve).c_str());
  
  bool xSlope = (curve.segmentPoints.at(1).positions.at(0) - curve.segmentPoints.at(0).positions.at(0) > 0);
  bool ySlope = (curve.segmentPoints.at(1).positions.at(1) - curve.segmentPoints.at(0).positions.at(1) > 0);

  double x = ms.msg_.positions.at(0);
  double y = ms.msg_.positions.at(1);

  double x_numerator = (xSlope  ? x - curve.controlPoints.at(0).positions.at(0)
                                : curve.controlPoints.at(0).positions.at(0) - x);
  double y_numerator = (ySlope  ? y - curve.controlPoints.at(0).positions.at(1)
                                : curve.controlPoints.at(0).positions.at(1) - y);
  double denom = curve.controlPoints.at(2).positions.at(0) - curve.controlPoints.at(0).positions.at(0);

  double x_scaled = x_numerator / denom;
  double y_scaled = y_numerator / denom;
  
  ROS_INFO("x: %f y: %f x_num: %f y_num: %f denom: %f x_scaled: %f y_scaled: %f", x, y, x_numerator, y_numerator, denom, x_scaled, y_scaled);

  bool x_good = (x_scaled >= 0. && x_scaled <= curve.u_target);
  bool y_good = (y_scaled >= 0. && y_scaled <= curve.u_target);


  if(x_scaled < 0. && y_scaled < 0.) {
    ROS_INFO("Returning 1 (before curve)");
    return 1;  
  } 
  else if(x_good && y_good)
  {
    ROS_INFO("Returning true");
    return 2;
  } 
  else if(x_scaled > curve.u_target && y_scaled > curve.u_target) {
    ROS_INFO("Returning 3 (after curve)");
    return 3;  
  }


  ROS_INFO("Returning 1");
  return 1;
}




void Planner::adaptCurves() {
  std::vector<ramp_msgs::BezierInfo> result;
  ramp_msgs::BezierInfo blank;

  for(uint16_t i=0;i<population_.size();i++) {
    
    // If the trajectory has a curve
    if(population_.get(i).msg_.curves.size() > 0) {
      std::cout<<"\nIn if trajectory has curve\n";

      // Set curve
      ramp_msgs::BezierInfo curve = population_.get(i).msg_.curves.at(0); 
      std::cout<<"\nSet curve to: "<<utility_.toString(curve)<<"\n";

      // If moving on this curve, update u, u_dot_0
      if( i == population_.getBestIndex() && 
            (curve.u_0 > 0 ||
             estimateIfOnCurve(startPlanning_, curve) == 2))
      {
        std::cout<<"\nIn if moving on curve\n";

        // if u_0==0 then estimateIfOnCurve returned 2 - already on curve
        if(curve.u_0 == 0) {
          //ROS_INFO("In if curve.u_0==0");

          // Get the time at the end of the curve
          double t_s0 = population_.get(i).msg_.trajectory.points.at(
              population_.get(i).msg_.i_knotPoints.at(1)).time_from_start.toSec();
          //ROS_INFO("t_s0: %f", t_s0);
          
          // t = the time spent moving on the curve
          double t = controlCycle_.toSec() - t_s0;

          //ROS_INFO("Adding %f", (t*curve.u_dot_0));
          curve.u_0 += t * population_.get(i).msg_.curves.at(0).u_dot_0;
        } // end if already on curve

        // Else, u_0 > 0, simply add to u_0
        else {
          //ROS_INFO("curve.u_0 > 0: %f curve.u_dot_0: %f", curve.u_0, curve.u_dot_0);
          curve.u_0 += 
            population_.get(i).msg_.curves.at(0).u_dot_0 * controlCycle_.toSec();
        }

        // Set the new ms_begin
        curve.ms_begin = startPlanning_.msg_;
      } // end if moving on curve


      ROS_INFO("Checking if done with curve");

      // Check if done with current curve
      if(estimateIfOnCurve(startPlanning_, curve) == 3 || curve.u_0 > curve.u_target) {
        ROS_INFO("Done with curve, u_0: %f", curve.u_0);

        // If we were using 2 curves
        if(population_.get(i).msg_.curves.size() > 1) {
          ROS_INFO("In if using 2 curves");

          curve = population_.get(i).msg_.curves.at(1);

          // Then we should start the next curve
          curve.u_0 += 0.000001; 
          curve.ms_begin = population_.paths_.at(i).start_.motionState_.msg_;

          // Done with 1st curve so erase it
          //population_.get(i).msg_.curves.erase(
              //population_.get(i).msg_.curves.begin() );
          //tr = buildTrajectoryRequest(population_.paths_.at(i), population_.get(i).msg_.curves);
        } // end if using 2 curves

        // Else if we only have 1 curve
        // we are done so set curve to be blank
        else {
          ROS_INFO("In else done with curve and only have 1 curve");
          curve = blank; 
          //tr = buildTrajectoryRequest(population_.paths_.at(i));
        } // end else 1 curve
      } // end if done with 1st curve

      result.push_back(curve);
    } // end if trajectory has curve

    // Else if there is no curve, push on a blank one
    else {
      result.push_back(blank);
    } // end else no curve
  } // end for
} // End adaptCurves




// TODO: Clean up
/** This method updates the population with the current configuration 
 *  The duration is used to determine which knot points still remain in the trajectory */
void Planner::adaptPopulation(ros::Duration d) {
  ROS_INFO("In adaptPopulation");
  ROS_INFO("startPlanning_: %s \nduration: %f", startPlanning_.toString().c_str(), d.toSec());
  
 
  // Update the paths with the new starting configuration
  adaptPaths(startPlanning_, d);

  // Create the vector to hold updated trajectories
  std::vector<RampTrajectory> updatedTrajecs;

  // For each path, get a trajectory
  for(unsigned int i=0;i<population_.paths_.size();i++) {
    //std::cout<<"\nPopulation member "<<i<<"'s Bezier path: "<<population_.get(i).bezierPath_.toString()<<"\n";
    std::cout<<"\ni: "<<(int)i;

    ramp_msgs::TrajectoryRequest tr;

    // If the trajectory has a curve
    if(population_.get(i).msg_.curves.size() > 0) {
      std::cout<<"\nIn if trajectory has curve\n";

      // Set curve
      ramp_msgs::BezierInfo curve = population_.get(i).msg_.curves.at(0); 
      std::cout<<"\nSet curve to: "<<utility_.toString(curve)<<"\n";
      
      // If moving on this curve, update u, u_dot_0
      if( i == population_.getBestIndex() && 
            (curve.u_0 > 0 ||
             estimateIfOnCurve(startPlanning_, curve) == 2))
      {
        std::cout<<"\nIn if moving on curve\n";

        // if u_0==0 then estimateIfOnCurve returned 2 - already on curve
        if(curve.u_0 == 0) {
          //ROS_INFO("In if curve.u_0==0");

          // Get the time at the end of the curve
          double t_s0 = population_.get(i).msg_.trajectory.points.at(
              population_.get(i).msg_.i_knotPoints.at(1)).time_from_start.toSec();
          //ROS_INFO("t_s0: %f", t_s0);
          
          // t = the time spent moving on the curve
          double t = controlCycle_.toSec() - t_s0;

          //ROS_INFO("Adding %f", (t*curve.u_dot_0));
          curve.u_0 += t * population_.get(i).msg_.curves.at(0).u_dot_0;
        } // end if already on curve

        // Else, u_0 > 0
        else {
          ROS_INFO("curve.u_0 > 0: %f curve.u_dot_0: %f", curve.u_0, curve.u_dot_0);
          curve.u_0 += 
            population_.get(i).msg_.curves.at(0).u_dot_0 * controlCycle_.toSec();
        }

        // Set the new ms_begin
        curve.ms_begin = startPlanning_.msg_;

        // Set the updated curve
        population_.get(i).msg_.curves.at(0) = curve;
      } // end if moving on curve

      else {
        ROS_INFO("Not moving on curve");
      }

      ROS_INFO("Checking if done with curve");

      // Check if done with current curve
      if(estimateIfOnCurve(startPlanning_, curve) == 3 || curve.u_0 > curve.u_target) {
        ROS_INFO("Done with curve, u_0: %f", curve.u_0);

        // If we were using 2 curves
        if(population_.get(i).msg_.curves.size() > 1) {
          ROS_INFO("In if using 2 curves");

          // Then we should start the next curve
          population_.get(i).msg_.curves.at(1).u_0 += 0.000001; 
          population_.get(i).msg_.curves.at(1).ms_begin = population_.paths_.at(i).start_.motionState_.msg_;

          // Done with 1st curve so erase it
          population_.get(i).msg_.curves.erase(
              population_.get(i).msg_.curves.begin() );
          tr = buildTrajectoryRequest(population_.paths_.at(i), population_.get(i).msg_.curves);
        } // end if using 2 curves

        // Else if we only have 1 curve
        else {
          ROS_INFO("In else done with curve and only have 1 curve");

          // Do not include curve because we are making a new curve
          // to replace the last one
          tr = buildTrajectoryRequest(population_.paths_.at(i));
        } // end else 1 curve
      } // end if done with 1st curve
      else {
        std::cout<<"\nNot done with curve";
        tr = buildTrajectoryRequest(population_.paths_.at(i), population_.get(i).msg_.curves, i);
      }
    } // end if trajectory has curve
    else {
      std::cout<<"\nTrajectory has no curve";
      tr = buildTrajectoryRequest(population_.paths_.at(i));
    }



    /* Get the trajectory */
    std::cout<<"\nRequest trajectory in adaptPopuation\n";
    ROS_INFO("$$$Path: %s\n$$$", utility_.toString(tr.request.path).c_str());
    RampTrajectory temp = requestTrajectory(tr, population_.get(i).msg_.id);
    temp.msg_.fitness = population_.get(i).msg_.fitness;
    temp.msg_.feasible = population_.get(i).msg_.feasible;
    ROS_INFO("Done getting new trajectory");
    ROS_INFO("New trajec: %s", temp.toString().c_str());

    // Push onto updatedTrajecs
    updatedTrajecs.push_back(temp);
  } // end for
  ROS_INFO("Outside of for");

  // Replace the population's trajectories_ with the updated trajectories
  population_.replaceAll(updatedTrajecs);
  ROS_INFO("Done replacing");
  ROS_INFO("Done adapting, pop now: %s", population_.toString().c_str());
  ROS_INFO("Exiting adaptPopulation");
} // End adaptPopulation





// TODO: Clean up
/** Build a TrajectoryRequest srv */
const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const std::vector<ramp_msgs::BezierInfo> curves, const int id) {
  ROS_INFO("In buildTrajectoryRequest");
  ramp_msgs::TrajectoryRequest result;

  result.request.path           = path.buildPathMsg();
  result.request.resolutionRate = resolutionRate_;
  result.request.type           = PARTIAL_BEZIER;

  if(path.size() > 2) {
    ROS_INFO("In if path.size() > 2)");
    // If it's the first time the trajectory is planned
    if(curves.size() == 0) {
      if(path.size() >= 3) {
        ROS_INFO("In if path.size() >= 3");
        ramp_msgs::BezierInfo temp;
        temp.segmentPoints.push_back( path.all_.at(0).motionState_.msg_ );
        temp.segmentPoints.push_back( path.all_.at(1).motionState_.msg_ );
        temp.segmentPoints.push_back( path.all_.at(2).motionState_.msg_ );
        result.request.bezierInfo.push_back(temp);
      }
    }
    else {
      ROS_INFO("In else if path.size < 3");
      result.request.bezierInfo = curves;
      
      if(result.request.bezierInfo.size() > 0) {

        // If the curve has been entered
        if( id == population_.getBestIndex() && 
            (result.request.bezierInfo.at(0).u_0 > 0 ||
              estimateIfOnCurve(startPlanning_, curves.at(0)) == 2) )
        {
          std::cout<<"\nstartBezier: true\n";
          result.request.startBezier = true;
        }
        // Set u_0=0. Not sure why, but sometimes u_0 was > 0 but not moving on curve
        // Maybe previously moving on curve, but switched to new curve
        else {
          std::cout<<"\nCurve not entered\n";
          result.request.bezierInfo.at(0).u_0 = 0;
        }
      }
    } // end else
  } // end if

  else {
    ROS_INFO("In path.size() == 2");
  }


  return result;
} // End buildTrajectoryRequest


const ramp_msgs::TrajectoryRequest Planner::buildTrajectoryRequest(const Path path, const int id) {
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
unsigned int Planner::getIRT() { return i_rt++; }




/** Check if there is imminent collision in the best trajectory */
void Planner::imminentCollisionCallback(const ros::TimerEvent& t) {
  //std::cout<<"\nIn imminentCollisionCycle_\n";

  if(!bestTrajec_.msg_.feasible && (bestTrajec_.msg_.t_firstCollision < D_)) {
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
    ROS_ERROR("Odometry message from ramp_control does not have all DOFs: %s", utility_.toString(msg).c_str());
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
void Planner::init(const uint8_t i, const ros::NodeHandle& h, const MotionState s, const MotionState g, const std::vector<Range> r, const int population_size, const bool sub_populations, const int gens_before_cc, const double t_fixed_cc) {

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
    new_pop.push_back(evaluateTrajectory(trajec, false));
  
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
    new_pop.push_back(evaluateTrajectory(trajec, false));
  
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

  // Clear m_i
  m_i.clear();
  
  // Need to set m_delta
  // motion difference from previous CC to next CC
  MotionState delta_m = m_cc_.subtract(latestUpdate_);
  //std::cout<<"\nDelta_m: "<<delta_m.toString();

  // Divide delta_m by num_pc to get the motion difference for each PC
  MotionState delta_m_inc = delta_m.divide(generationsPerCC_);
  //std::cout<<"\nDelta_m / num_pc: "<<delta_m_inc.toString();
  
  // Set m_i
  // Each m_i will be start + (delta_m_inc * i)
  for(int i=0;i<generationsPerCC_;i++) {
    MotionState temp = delta_m_inc.multiply(i+1);
    MotionState m = latestUpdate_.add(temp);

    m_i.push_back(m);

    //std::cout<<"\n\nPC: "<<i<<": Pushing on "<<m.toString();
  }
}




/** This method will return a vector of trajectoies for the vector of paths */
const std::vector<RampTrajectory> Planner::getTrajectories(const std::vector<Path> p) {
  std::cout<<"\np.size(): "<<p.size()<<"\n";
  std::vector<RampTrajectory> result;

  // For each path
  for(unsigned int i=0;i<p.size();i++) {
    std::cout<<"\ni: "<<i<<"\n"; 
    // Get a trajectory
    RampTrajectory temp = requestTrajectory(p.at(i));
    result.push_back(temp);
  } // end for

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



const std::vector<RampTrajectory> Planner::switchTrajectory(const RampTrajectory from, const RampTrajectory to) {
  std::vector<RampTrajectory> result;

  RampTrajectory switching = getTransitionTrajectory(from, to);
  RampTrajectory full = switching;
  
  // Set the proper ID
  full.msg_.id = to.msg_.id;
  
 
  std::cout<<"\nTransition trajectory in getTrajectoryWithCurve: "<<full.toString();
  std::cout<<"\nCurves: ";
  for(int i=0;i<switching.msg_.curves.size();i++) {
    std::cout<<"\n"<<i<<": "<<utility_.toString(switching.msg_.curves.at(i));
  }
  /*displayTrajectory(result.msg_);
  std::cout<<"\nDisplaying transition trajectory\n";
  stopForDebugging();
  std::cin.get();
  restartAfterDebugging();*/


  /* After getting transition trajectory, we want to 
     concatenate it with the rest of the target trajectory */
  
  ROS_INFO("Getting cycle time");
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

  ROS_INFO("Checking if rotation at beginning");
  // Check if there's rotation at the beginning, if so increment c_kp
  // TODO: Better way of doing this
  if(utility_.positionDistance( to.msg_.trajectory.points.at(0).positions,
        to.msg_.trajectory.points.at( to.msg_.i_knotPoints.at(1)).positions ) < 0.1)
  {
    //std::cout<<"\nIncrementing c_kp";
    c_kp++;
  }
  ROS_INFO("c_kp: %i", c_kp);
  //std::cout<<"\ntrgt_traj.msg_.i_knotPoints.at(c_kp): "<<trgt_traj.msg_.i_knotPoints.at(c_kp);
  //std::cout<<"\nMS at c_kp: "<<utility_.toString(trgt_traj.msg_.trajectory.points.at( trgt_traj.msg_.i_knotPoints.at(c_kp)));

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
      std::cout<<"\ni: "<<(int)i<<" trgt_traj.msg_.i_knotPoints.at("<<c_kp<<"): "<<to.msg_.i_knotPoints.at(c_kp);
      full.msg_.i_knotPoints.push_back(full.msg_.trajectory.points.size()-1);
      KnotPoint kp(full.msg_.trajectory.points.at(
            full.msg_.trajectory.points.size()-1));
      full.path_.all_.push_back(kp);
      c_kp++;
    }
  } // end for
 
  

  std::cout<<"\nTrajectory with curve path: "<<full.path_.toString();
  
  // Push on the target trajectory's Bezier curve
  for(uint8_t i_curve=0;i_curve<to.msg_.curves.size();i_curve++) {
    full.msg_.curves.push_back(to.msg_.curves.at(i_curve));
  }

  result.push_back(switching);
  result.push_back(full);

  return result;
}





const RampTrajectory Planner::getTransitionTrajectory(const RampTrajectory current, const RampTrajectory trgt_traj) {
  std::cout<<"\nIn getTransitionTrajectory\n";
  std::cout<<"\ntrgt_traj: "<<trgt_traj.toString()<<"\n";
  std::cout<<"\ntrgt_traj Path: "<<trgt_traj.path_.toString()<<"\n";
  std::cout<<"\ntrgt_traj Bezier Path: "<<trgt_traj.bezierPath_.toString()<<"\n";

  double t = (c_pc_+1)*planningCycle_.toSec();// + planningCycle_.toSec();
  std::cout<<"\nt: "<<t<<" PC time: "<<planningCycle_.toSec();
  ROS_INFO("movingOn: %s", movingOn_.toString().c_str());
  
  MotionState ms_startSwitch = current.getPointAtTime(t);
  ROS_INFO("ms_startSwitch: %s", ms_startSwitch.toString().c_str());
  
  std::vector<MotionState> segment_points;
  segment_points.push_back(ms_startSwitch);
  //segment_points.push_back(startPlanning_);
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
  ROS_INFO("i_goal: %i", i_goal);
  ROS_INFO("g: %s", g.toString().c_str());

  //std::cout<<"\nstart: "<<start_.toString()<<"\n";
  //std::cout<<"\nstartPlaning: "<<startPlanning_.toString()<<"\n";
  //std::cout<<"\nbestTrajec: "<<bestTrajec_.toString();

  Path p(segment_points);
  //std::cout<<"\nPath to change trajectories: "<<p.toString()<<"\n";

  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  tr.request.type = TRANSITION;
  tr.request.print = false; 
  tr.request.startBezier = false;

  //std::cout<<"\nRequesting trajectory in getTransitionTrajectory\n";
  RampTrajectory transition = requestTrajectory(tr);
  //std::cout<<"\ntransition: "<<transition.toString();

  //std::cout<<"\nLeaving getTransitionTrajectory()\n";
  return transition;
}






/*****************************************************
 ****************** Request Methods ******************
 *****************************************************/

/** Request a trajectory */
const RampTrajectory Planner::requestTrajectory(ramp_msgs::TrajectoryRequest& tr, const int id) {
  RampTrajectory result;
  //std::cout<<"\nid: "<<id;

  
  if(h_traj_req_->request(tr)) {
   
    // Set the actual trajectory msg
    result.msg_ = tr.response.trajectory;

    // Set the paths (straight-line and bezier)
    result.path_        = tr.request.path;
    result.bezierPath_  = tr.response.newPath;

    // *** Set the previous knot point
    result.ms_prevSP_ = tr.request.path.points.at(0).motionState;

    // Set the ID of the trajectory
    if(id != -1) {
      result.msg_.id = id;
    }
    else {
      result.msg_.id = getIRT();
      //std::cout<<"\ni_rt: "<<i_rt;
    }
  }
  else {
    ROS_ERROR("An error occurred when requesting a trajectory");
  }

  return result;
}



const RampTrajectory Planner::requestTrajectory(const Path p, const int id) {
  ramp_msgs::TrajectoryRequest tr = buildTrajectoryRequest(p);
  RampTrajectory result = requestTrajectory(tr, id);
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
void Planner::modification() {

  //std::cout<<"\nBefore modifying trajectory\n";
  // Modify 1 or more trajectories
  std::vector<RampTrajectory> mod_trajec = modifyTrajec();
  /*std::cout<<"\nAfter modifying trajectory, mod_trajec.size(): "<<mod_trajec.size()<<"\n";
  for(int i=0;i<mod_trajec.size();i++) {
    std::cout<<"\nModified Trajec "<<i<<": "<<mod_trajec.at(i).toString()<<"\n";
  }*/
  

  // Evaluate and add the modified trajectories to the population
  // and update the planner and the modifier on the new paths
  for(unsigned int i=0;i<mod_trajec.size();i++) {
    //std::cout<<"\nramp_planner: Evaluating trajectory "<<(int)i<<"\n";

    // Evaluate the new trajectory
    mod_trajec.at(i) = evaluateTrajectory(mod_trajec.at(i));

    
    // Add the new trajectory to the population
    // Index is where the trajectory was added in the population (may replace another)
    int index = population_.add(mod_trajec.at(i));

    // If sub-populations are being used and
    // the trajectory was added to the population, update the sub-populations 
    // (can result in infinite loop if not updated but re-evaluated)
    if(subPopulations_ && index >= 0) {
      //std::cout<<"\nCreating sub-pop after modifying\n";
      population_.createSubPopulations();
      //std::cout<<"\nDone creating sub-pop after modifying\n";
    }
  } // end for
  //std::cout<<"\nAfter for\n";

    
  // Evaluate entire population
  int index = population_.getBestIndex();
  //std::cout<<"\nindex: "<<index<<"\n";
  bestTrajec_ = population_.get(index);
  
  // If the best trajectory has changed and the control cycles have started
  if(index != i_best_prev_ && cc_started_) {
    //std::cout<<"\nBest trajectory now index: "<<index;
  
    // Set index of previous best
    i_best_prev_ = index;
  } // end if

  //std::cout<<"\nLeaving modification\n";
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
  //std::cout<<"\nIn predictStartPlanning\n";
  MotionState result;

  // If the orientation is not satisfied, 
  if(!checkOrientation()) {
    result = latestUpdate_;
    //d = ros::Duration(0);
  }
  else {
    //std::cout<<"\nc_pc: "<<c_pc_<<" m_i.size(): "<<m_i.size()<<" latestUpdate_.size(): "<<latestUpdate_.msg_.positions.size()<<"\n";
    
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
  std::cout<<"\nPlanning cycle occurring, generation = "<<generation_<<"\n";
  
  //std::cout<<"\nAfter generation "<<generation_<<", population: "<<population_.fitnessFeasibleToString();
  //std::cout<<"\nBest: "<<bestTrajec_.toString();


  // At generation x, insert a straight-line trajectory to the goal
  if(generation_ == 150) {
    //seedPopulationTwo();
    //std::cout<<"\nPop: "<<population_.fitnessFeasibleToString();
  }

  tf::Vector3 v_linear( latestUpdate_.msg_.velocities.at(0), 
                        latestUpdate_.msg_.velocities.at(1), 0);
  double mag_linear = sqrt(tf::tfDot(v_linear, v_linear));
  //if(generation_ > 0 && generation_ % 25 == 0) {
  //if(mag_linear > 0 && cc_started_ && c_pc_ == generationsPerCC_/2) {
  /*if(mag_linear > 0 && cc_started_ && generation_ == 25) {
    ROS_INFO("generationsPerCC_: %i", generationsPerCC_);
    ROS_INFO("Creating random population at c_pc: %i", c_pc_);
    seedPopulationTwo();
    //population_ = randomPopulation(startPlanning_, goal_);
    int i = population_.getBestIndex();
    if(i == 0) {
      bestTrajec_ = population_.get(1);
    }
    else if(i == 1) {
      bestTrajec_ = population_.get(0);
    }*/
    /*stopForDebugging();
    sendPopulation();
    std::cout<<"\n*************** New population: "<<population_.toString()<<"\n";
    std::cout<<"\nBest: "<<bestTrajec_.toString();
    std::cout<<"\nPress Enter to continue\n";
    std::cin.get();*/
    //std::cout<<"\n ========== Re-evaluating =========\n";
    /*population_ = evaluatePopulation(population_);

    int i_kp = bestTrajec_.transitionTraj_.i_knotPoints.size()-1;
    double t = bestTrajec_.msg_.trajectory.points.at(
        bestTrajec_.msg_.i_knotPoints.at(i_kp)).time_from_start.toSec();
    ROS_INFO("transTraj: %s", utility_.toString(bestTrajec_.transitionTraj_).c_str());
    ROS_WARN("i_kp: %i", i_kp);
    ROS_WARN("t: %f", t);

    //bestTrajec_ = population_.getBest();
    restartControlCycle(t);
    //sendBest();
    sendPopulation();
    //ROS_INFO("Final new pop: %s", population_.toString().c_str());
    //std::cin.get();
    //restartAfterDebugging();
  }
  else {
    ROS_INFO("mag_linear: %f cc_started_: %s generation_: %i", mag_linear, cc_started_ ? "true" : "false", generation_);
  }*/


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
    if(modifications_) {
      modification();
    }
    else {
      std::cout<<"\nModifications not enabled";
    }
    //std::cout<<"\nAfter modification\n";


    if(evaluations_) {
      // TODO: Exactly how to do evaluations
      //bestTrajec_ = evaluateAndObtainBest();
      //bestTrajec_ = population_.getBest();
    }

    
    // t=t+1
    generation_++;
    
    if( (generation_-1) % 10 == 0) {
      std::cout<<"\nPlanning cycle "<<generation_-1<<" complete\n";
    }

    c_pc_++;
  
    // Taking out because we create sub-pops after modifying 
    /*if(subPopulations_) {
      std::cout<<"\nCreating sub-pops in PC\n";
      population_.createSubPopulations();
      std::cout<<"\nDone creating sup-pops in PC\n";
    }*/
    
    // Send the new population to the trajectory viewer
    sendPopulation();
    //std::cout<<"\nAfter sending pop\n";
  
    std::cout<<"\nGeneration "<<generation_-1<<" Completed\n";
  } // end if c_pc<genPerCC
} // End planningCycleCallback






/** This methed runs the tasks needed to do a control cycle */
void Planner::doControlCycle() {
  ROS_INFO("In doControlCycle");
  ROS_INFO("bestTrajec_.size(): %i", (int)bestTrajec_.msg_.trajectory.points.size());
  ROS_INFO("controlCycle_.toSec(): %f", controlCycle_.toSec());
  
  // Send the best trajectory and set movingOn
  sendBest();
  movingOn_ = bestTrajec_.getSubTrajectory(controlCycle_.toSec());

  // Reset planning cycle count
  c_pc_ = 0;

  // The motion state that we should reach by the next control cycle
  m_cc_ = bestTrajec_.getPointAtTime(controlCycle_.toSec());
  startPlanning_ = m_cc_;

  // After m_cc_ and startPlanning are set, adapt the population
  adaptPopulation(controlCycle_);
  bestTrajec_ = evaluateAndObtainBest(population_);
  
  // Create sub-populations if enabled
  if(subPopulations_) {
    population_.createSubPopulations();
  }
  
  // Send the population to trajectory_visualization
  sendPopulation();
} // End doControlCycle





/** This method updates the population based on the latest 
 *  configuration of the robot, re-evaluates the population,
 *  and sends a new (and better) trajectory for the robot to move along */
void Planner::controlCycleCallback(const ros::TimerEvent&) {
  //std::cout<<"\n************* Control cycle occurring *************\n";
  std::cout<<"\nControl Cycle "<<num_controlCycles_<<"\n";
  ROS_INFO("Population: %s", population_.toString().c_str());
  

  /*std::cout<<"\nstartPlanning: "<<startPlanning_.toString();
  std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString()<<"\n";*/
  
  // Check if the time is adaptive value
  if(controlCycle_.toSec() != t_fixed_cc_) {
    ROS_WARN("controlCycle_ != t_fixed_cc, \n cc: %f t_fixed_cc_: %f", controlCycle_.toSec(), t_fixed_cc_);
    controlCycle_ = ros::Duration(t_fixed_cc_);
    controlCycleTimer_.setPeriod(controlCycle_);
  }
  
  
  // Uncertainty code
  //****SP_LU_diffs_.push_back(startPlanning_.subtract(latestUpdate_));
  
  // Do the control cycle
  doControlCycle();

  // Uncertainty code
  // Build m_i
  //*****setMi();
  

  // Set flag showing that CCs have started
  if(!cc_started_) {
    cc_started_ = true;
  }
  
  ROS_INFO("After adapting population: %s", population_.toString().c_str());
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

    temp.getBestIndex();
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
  double bestFitness = 0;
  //double bestFitness = bestTrajec_.msg_.fitness;

  // fitness = 1/time so time = 1/fitness
  double t = 1. / bestFitness;
  double t_new = t + ((generationsPerCC_ - c_pc_) * planningCycle_.toSec());

  // Best fitness adjusted for t_new
  bestFitness = 1. / t_new;

  return (traj.msg_.fitness > bestFitness);
}



/** This method evaluates one trajectory.
 *  Eventually, we should be able to evaluate only specific segments along the trajectory  */
const RampTrajectory Planner::evaluateTrajectory(RampTrajectory trajec, const bool computeSwitch) {
  //ROS_INFO("xxxxx In evaluateTrajectory xxxxx");
  //ROS_INFO("trajec: %s", trajec.toString().c_str());

  RampTrajectory result = requestEvaluation(trajec);
  //std::cout<<"\nresult.fitness: "<<result.msg_.fitness<<" bestTrajec.fitness: "<<bestTrajec_.msg_.fitness;
  //std::cout<<"\ntrajec.id: "<<trajec.msg_.id<<" bestTrajec.id: "<<bestTrajec_.msg_.id;

  if(computeSwitch) {
    tf::Vector3 v_linear(latestUpdate_.msg_.velocities.at(0), latestUpdate_.msg_.velocities.at(1), 0);
    double mag_linear = sqrt(tf::tfDot(v_linear, v_linear));
    double mag_angular = latestUpdate_.msg_.velocities.at(2);
   
    //ROS_INFO("v: %f w: %f", mag_linear, mag_angular);
    
    double bestFitness = bestTrajec_.msg_.fitness;
    //ROS_INFO("bestFitness: %f", bestFitness);

    // If the fitness is close to the best result trajectory's fitness
    // and its not the best traj
    // and it is not just rotating 
    if(cc_started_ && result.msg_.id != bestTrajec_.msg_.id && mag_linear > 0.0001 && 
        (result.msg_.fitness > bestFitness ||
        fabs(result.msg_.fitness - bestFitness) < transThreshold_) ) 
    {
      //ROS_INFO("Close enough to compute a switching curve");
      //std::cout<<"\nlatestUpdate: "<<latestUpdate_.toString();

      double theta_current = latestUpdate_.msg_.positions.at(2);

      int kp = (trajec.msg_.i_knotPoints.size() == 2) ? 1 : 2;
      double theta_to_move;
      // Check if positions are the same
      // TODO: If there is a rotation, check 1st and 3rd
      //       otherwise check 1st and 2nd
      if( fabs(utility_.positionDistance(trajec.msg_.trajectory.points.at(
                                    trajec.msg_.i_knotPoints.at(0)).positions,
                                   trajec.msg_.trajectory.points.at(
                                     trajec.msg_.i_knotPoints.at(kp)).positions)) < 0.0001)
      {
        //std::cout<<"\nIn if positions are the same\n";
        theta_to_move = trajec.msg_.trajectory.points.at(0).positions.at(2);
      }
      else {
        //std::cout<<"\nIn else positions are not the same\n";
        theta_to_move = utility_.findAngleFromAToB(
                              trajec.msg_.trajectory.points.at(0), 
                              trajec.msg_.trajectory.points.at(
                                trajec.msg_.i_knotPoints.at(kp)) ); 
      }
      //std::cout<<"\ntheta_current: "<<theta_current<<" theta_to_move: "<<theta_to_move;
      //std::cout<<"\nDifference: "<<utility_.findDistanceBetweenAngles(theta_current, theta_to_move)<<"\n";


      // If greater than 90 degrees
      if(fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move)) > PI/2) {
        //ROS_INFO("Orientation change > PI/2 - Too much for a switch - adding penalty");
        result.msg_.fitness = 0;
      }

      // If greater than 5 degrees
      else if(fabs(utility_.findDistanceBetweenAngles(theta_current, theta_to_move)) > 0.017) {
        //std::cout<<"\nOrientation needs to change\n";
        //std::cout<<"\nBest trajec: "<<bestTrajec_.toString()<<"\n";

        // Get transition trajectory
        std::vector<RampTrajectory> trajecs = switchTrajectory(movingOn_, trajec);
        ROS_INFO("trajecs size: %i", (int)trajecs.size());
        RampTrajectory T_new = trajecs.at(1);
        
        // Evaluate T_new
        T_new = requestEvaluation(T_new);
        T_new.transitionTraj_ = trajecs.at(0).msg_;
        //std::cout<<"\nT_new: "<<T_new.toString()<<"\n";

        // If the trajectory including the curve to switch is more fit than 
        // the best trajectory, return it
        if(compareSwitchToBest(T_new)) {
          //std::cout<<"\n************ Switching Trajectories **************\n";

          result = T_new;
          result.bezierPath_ = trajec.bezierPath_;
          result.path_ = trajec.path_;
          result.path_.changeStart(latestUpdate_); 

          bestTrajec_ = result;

          ROS_INFO("Displaying T_new: %s", T_new.toString().c_str());
          ROS_INFO("Latest update: %s", latestUpdate_.toString().c_str());
          /*displayTrajectory(T_new.msg_);
          std::cout<<"\nPress Enter to continue\n";
          std::cin.get();*/

          num_switches_++;
        } // end if switching
        else {
          //ROS_INFO("compareSwitchToBest: false");
        }
      } // end if switch traj needed 

      // Else, no switch necessary
      else {
        //ROS_DEBUG("No switching trajectory being computed because the orientation is within 5 degrees of current orientation");
      }
    } // end if fitness close enough to compute switch
    else {
      //std::cout<<"\nFitness not within "<<transThreshold_<<"\n";
    }
  } // end if computeSwitch
  else {
    //std::cout<<"\ncomputeSwitch: False\n";
  }

  //ROS_INFO("xxxxx Leaving evaluateTrajectory xxxxx");
  return result;
} // End evaluateTrajectory



// TODO: Return evaluated population instead of passing in reference?
/** 
 * This method evaluates each trajectory in the population
 * It also sete i_best_prev_
 **/
const Population Planner::evaluatePopulation(Population pop, const bool computeSwitch) {
  //std::cout<<"\n********* In evaluatePopulation *********\n";
  //ROS_INFO("computeSwitch: %s", computeSwitch ? "true" : "false");
  //std::cout<<"\nPopulation: "<<pop.toString();
  Population result = pop;
  
  int i_best = pop.getBestIndex();
  //std::cout<<"\ni_best: "<<i_best;
  
  if(i_best > -1) {
    //std::cout<<"\nEvaluating best: ";
    result.replace(i_best, 
        evaluateTrajectory(result.get(i_best), computeSwitch));
    bestTrajec_ = result.get(i_best); 
  }
  
  // Go through each trajectory in the population and evaluate it
  for(unsigned int i=0;i<result.size();i++) {
    //std::cout<<"\ni: "<<(int)i;
    if(i != i_best) {
      result.replace(i, evaluateTrajectory(result.get(i), computeSwitch));
      //result.replace(i, evaluateTrajectory(result.get(i), false));
    }
  } // end for

  i_best_prev_ = result.getBestIndex();
  //std::cout<<"\nPopulation now: "<<result.toString();
  //std::cout<<"\ni_best_prev: "<<i_best_prev_;
  //std::cout<<"\n******** Leaving evaluatePopulation ********\n";
  
  return result;
} // End evaluatePopulation




/** This method calls evaluatePopulation and population_.getBest() */
const RampTrajectory Planner::evaluateAndObtainBest(Population pop) {

  // Evaluate population
  pop = evaluatePopulation(pop, false);
  
  // Find the best trajectory
  int index = pop.getBestIndex();
  i_best_prev_ = index;

  // Return the trajectory
  return pop.get(index);
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
    i_best_prev_ = population_.getBestIndex();
    std::cout<<"\nPopulation seeded!\n";
    std::cout<<"\n"<<population_.fitnessFeasibleToString()<<"\n";
    std::cout<<"\n** Pop **:"<<population_.toString();
    // Evaluate after seeding
    bestTrajec_ = evaluateAndObtainBest(population_);
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
  //sendBest();
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
  

  //std::cout<<"\nFinal population: ";
  //std::cout<<"\n"<<pathsToString(); 
  
  std::cout<<"\nNumber of trajectory switches: "<<num_switches_;
  std::cout<<"\nLeaving go\n";
} // End go
