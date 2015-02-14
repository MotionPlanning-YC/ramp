#include "mobile_base.h"



/** Constructor */
MobileBase::MobileBase() {
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters = 0;
  reflexxesData_.outputParameters = 0;
} 




/** Destructor */
MobileBase::~MobileBase() {
  if(reflexxesData_.rml != 0) {
    delete reflexxesData_.rml;
    reflexxesData_.rml = 0;
  }
  if(reflexxesData_.inputParameters) {
    delete reflexxesData_.inputParameters;
    reflexxesData_.inputParameters = 0;
  }
  if(reflexxesData_.outputParameters != 0) {
    delete reflexxesData_.outputParameters;
    reflexxesData_.outputParameters = 0;
  }
}



/** Initialize Reflexxes variables */
void MobileBase::initReflexxes() {

  // Set DOF
  reflexxesData_.NUMBER_OF_DOFS = 3;

  // Initialize all relevant objects of the Type II Reflexxes Motion Library
  if(reflexxesData_.rml == 0) {
    reflexxesData_.rml = new ReflexxesAPI( 
            reflexxesData_.NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );

    reflexxesData_.inputParameters = new RMLPositionInputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );

    reflexxesData_.outputParameters = new RMLPositionOutputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );
  } // end if


  
  // Phase sync makes the orientation correct to drive in a straight line
  reflexxesData_.flags.SynchronizationBehavior = 
    RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;


  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[2] = 3*PI/4;
  

  // Maximum acceleration
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 1.;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1] = 1.;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[2] = PI/4;
  

  // As the maximum jerk values are not known, this is just to try
  reflexxesData_.inputParameters->MaxJerkVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[1] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[2] = PI/3;

  // Set flag value to know if Reflexxes has been called yet
  reflexxesData_.outputParameters->NewPositionVector->VecData[0] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[1] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[2] = -99;
  
  // Result
  reflexxesData_.resultValue = 0;
} // End initReflexxes



/** Initialize class object with a request */
// TODO: change 3 booleans to 1 enum
void MobileBase::init(const ramp_msgs::TrajectoryRequest::Request req) {
  //ROS_INFO("Entered MobileBase::init");
  //std::cout<<"\nRequest received: "<<utility_.toString(req)<<"\n";

  bezierStart = req.startBezier;
  //if(req.bezierInfo.u_0 > 0)
    //std::cout<<"\nBezier Info passed in: "<<utility_.toString(req.bezierInfo);

  // Store the path
  path_ = req.path;

  // Set print
  print_ = req.print;
 
  // Set trajectory type
  type_ = (TrajectoryType)req.type;

  // Initialize Reflexxes
  initReflexxes();
 
  // Set the initial conditions of the reflexxes library
  setInitialMotion();

  // Set SelectionVector
  setSelectionVector();

  // Starting time
  t_started_ = ros::Time::now();

  // Set the time to cutoff generating points
  // Mostly for catching any small bugs that
  // make Reflexxes unable to find goal
  timeCutoff_ = ros::Duration(50);
  
  //ROS_INFO("Leaving MobileBase::init");
} // End init



/** This method sets the new target of Reflexxes */
// ********************** Add a check for if there is a target for a non-selected dimension that's
// different than its current value **********************
void MobileBase::setTarget(const ramp_msgs::MotionState ms) {
  
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(i);
    if(ms.velocities.size() > 0) {
      reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(i);
    }
  }
  
  // Phase sync makes the orientation correct to drive in a straight line
  reflexxesData_.flags.SynchronizationBehavior = 
    RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
} // End setTarget







/** This method sets the SelectionVector for x,y trajectories */
void MobileBase::setSelectionVector() {

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[2] = false;
} // End setSelectionVector




/** This method sets the SelectionVector for rotation parts */
void MobileBase::setSelectionVectorRotation() {
  reflexxesData_.inputParameters->SelectionVector->VecData[0] = false;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = false;
  reflexxesData_.inputParameters->SelectionVector->VecData[2] = true;
} // End setSelectionVectorRotation


/**
 * Initialize variables after receiving a service request
 * Set-up the input parameters
 **/
void MobileBase::setInitialMotion() {
  
  // Initialise the time to use for each trajectory point
  timeFromStart_ = ros::Duration(0);
  
  
  // Set the positions of the robot as Reflexxes input
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(i);
  }

  // Set the current velocities of the robot as Reflexxes input
  if(path_.points.at(0).motionState.velocities.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = path_.points.at(0).motionState.velocities.at(i);
    }
  }
  else {//log some error
  }

  // Set the current accelerations of the robot as Reflexxes input
  if(path_.points.at(0).motionState.accelerations.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = path_.points.at(0).motionState.accelerations.at(i);
    }
  }
  else {//log some error
  }

} // End setCurrentMotion







/** Inserts a MotionState into the response trajectory */
void MobileBase::insertPoint(const ramp_msgs::MotionState ms, ramp_msgs::TrajectoryRequest::Response& res) {
  trajectory_msgs::JointTrajectoryPoint jp = utility_.getTrajectoryPoint(ms);
  jp.time_from_start = timeFromStart_;
  insertPoint(jp, res);
} // End insertPoint


/** Inserts a JointTrajectoryPoint at the back of response trajectory and sets Reflexxes to that point */
void MobileBase::insertPoint(const trajectory_msgs::JointTrajectoryPoint jp, ramp_msgs::TrajectoryRequest::Response& res) {
  res.trajectory.trajectory.points.push_back(jp);

  /** Update Reflexxes */
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);
  
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = jp.positions.at(0);
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[1] = jp.positions.at(1);
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = jp.positions.at(2);
  
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = jp.velocities.at(0);
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = jp.velocities.at(1);
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2] = jp.velocities.at(2);
  
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = jp.accelerations.at(0);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = jp.accelerations.at(1);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2] = jp.accelerations.at(2);


} // End insertPoint




/** Tests if a lambda value will have Bezier equations that are defined */
const bool MobileBase::lambdaOkay(const std::vector<ramp_msgs::MotionState> segment_points, const double lambda) const {
  //ROS_INFO("In lambdaOkay, lambda: %f", lambda);
  ramp_msgs::MotionState X0, X1, X2, p0, p1, p2;

  p0 = segment_points.at(0);
  p1 = segment_points.at(1);
  p2 = segment_points.at(2);
  
  X1 = segment_points.at(1);


  // Find how far along segment we already are
  // Can use x or y...here we use x
  double min_lambda = (path_.points.at(0).motionState.positions.at(0) - segment_points.at(0).positions.at(0)) 
                      / (segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0));
  //ROS_INFO("min_lambda in lambdaOkay: %f", min_lambda); 

  // TODO: Check for v
  if(lambda < min_lambda) {
    return false;
  }
  

  
  double l_s1 = utility_.positionDistance(segment_points.at(1).positions, segment_points.at(0).positions);
  double l_s2 = utility_.positionDistance(segment_points.at(2).positions, segment_points.at(1).positions);

  if(l_s2 < l_s1) {
    X2.positions.push_back( (1-lambda)*p1.positions.at(0) + lambda*p2.positions.at(0) );
    X2.positions.push_back( (1-lambda)*p1.positions.at(1) + lambda*p2.positions.at(1) );
    X2.positions.push_back(utility_.findAngleFromAToB(p1.positions, p2.positions));

    double theta = utility_.findAngleFromAToB(p0.positions, p1.positions);

    X0.positions.push_back( p1.positions.at(0) - l_s2*cos(theta) );
    X0.positions.push_back( p1.positions.at(1) - l_s2*sin(theta) );
    X0.positions.push_back( theta );
  }
  else {
    X0.positions.push_back( (1-lambda)*p0.positions.at(0) + lambda*p1.positions.at(0) );
    X0.positions.push_back( (1-lambda)*p0.positions.at(1) + lambda*p1.positions.at(1) );
    X0.positions.push_back(utility_.findAngleFromAToB(p0.positions, p1.positions));

    double theta = utility_.findAngleFromAToB(p1.positions, p2.positions);

    X2.positions.push_back( p1.positions.at(0) + l_s2*cos(theta) );
    X2.positions.push_back( p1.positions.at(1) + l_s2*sin(theta) );
    X2.positions.push_back( theta );
  }


  // If both A and B will equal 0
  if(X1.positions.at(0) == ( (X0.positions.at(0) + X2.positions.at(0)) / 2. ) &&
      X1.positions.at(1) == ( (X0.positions.at(1) + X2.positions.at(1)) / 2. )) 
  {
    //ROS_INFO("%f not okay", lambda);
    return false;
  }
  
  return true;
} // End lambdaOkay




/** Returns a lambda value that will lead to defined Bezier equations */
const double MobileBase::getControlPointLambda(const std::vector<ramp_msgs::MotionState> segment_points) const {
  std::vector<double> result;
  
  double l_s1 = utility_.positionDistance(segment_points.at(1).positions, segment_points.at(0).positions);
  double l_s2 = utility_.positionDistance(segment_points.at(2).positions, segment_points.at(1).positions);
  //std::cout<<"\nl_s1: "<<l_s1<<" l_s2: "<<l_s2;

  // Start transition trajectories right away,
  // otherwise, try to go straight for a while
  double lambda = type_ == TRANSITION ? 0.1 : 0.5;

  double min_lambda = (path_.points.at(0).motionState.positions.at(0) - segment_points.at(0).positions.at(0)) 
                      / (segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0));
  ROS_INFO("min_lambda: %f", min_lambda);

  if(min_lambda > 1) {
    
    ROS_ERROR("Minimum lambda: %f", min_lambda);
    ROS_ERROR("Minimum lambda > 1");

    lambda = min_lambda;
  }
  else {

    bool loopedOnce=false;
    while(!lambdaOkay(segment_points, lambda) && !loopedOnce) {
      if(type_ == TRANSITION) {
        lambda-=0.05;
      }
      else {
        lambda+=0.05;
      }

      if(lambda > 0.91) {
        lambda = 0.1;
        loopedOnce = true;
      }
      else if(lambda < 0.05) {
        lambda = 0.9;
        loopedOnce = true;
      }
    }
    //ROS_INFO("lambda final: %f", lambda);
  }

  return lambda;
} // End getControlPointLambda


const ramp_msgs::MotionState MobileBase::getMaxMS() const {
  ramp_msgs::MotionState result;

  result.velocities.push_back(
      reflexxesData_.inputParameters->
      MaxVelocityVector->VecData[0]);
  result.velocities.push_back(
      reflexxesData_.inputParameters->
      MaxVelocityVector->VecData[1]);
  result.accelerations.push_back(
      reflexxesData_.inputParameters->
      MaxAccelerationVector->VecData[0]);
  result.accelerations.push_back(
      reflexxesData_.inputParameters->
      MaxAccelerationVector->VecData[1]);

  return result;
}


/** */
const std::vector<BezierCurve> MobileBase::bezier(ramp_msgs::Path& p, const bool only_curve) {
  //ROS_INFO("Entered MobileBase::bezier");

  std::vector<BezierCurve> result;

  ramp_msgs::Path p_copy = p;

  // Set the index of which knot point to stop at
  //int stop = (req_.type == TRANSITION) ? 3 : 2; 
  int stop = req_.bezierInfo.size()+1;
  //std::cout<<"\nstop: "<<stop;


  // Find the first segment point for the curve
  // Increment index until the two points are different
  bool differentPoint = utility_.positionDistance(req_.bezierInfo.at(0).segmentPoints.at(0).positions, 
          req_.bezierInfo.at(0).segmentPoints.at(1).positions) > 0.01;
  int inc = 2;
  while(!differentPoint && inc < p_copy.points.size()) {
    /*std::cout<<"\nPoints 0 and 1 are the same";
    std::cout<<"\nPath: "<<utility_.toString(p)<<"\n";
    std::cout<<"\nSegment point 0: "<<utility_.toString(req_.bezierInfo.at(0).segmentPoints.at(0));
    std::cout<<"\nSegment point 1: "<<utility_.toString(req_.bezierInfo.at(0).segmentPoints.at(1))<<"\n";*/
    req_.bezierInfo.at(0).segmentPoints.at(1) = p_copy.points.at(inc).motionState;
    differentPoint = utility_.positionDistance(req_.bezierInfo.at(0).segmentPoints.at(0).positions, 
          req_.bezierInfo.at(0).segmentPoints.at(1).positions) > 0.01;
    //std::cin.get();
    inc++;

    if(inc == p_copy.points.size()) {
      //ROS_INFO("Cannot plan Bezier, returning same Path at 402");
      type_ = ALL_STRAIGHT_SEGMENTS;
      return result;
    }
  }

  // Find the second segment point for the curve
  // Increment index until the two points are different
  differentPoint = utility_.positionDistance(req_.bezierInfo.at(0).segmentPoints.at(1).positions, 
          req_.bezierInfo.at(0).segmentPoints.at(2).positions) > 0.01;
  inc = 3;
  while(!differentPoint && inc < p_copy.points.size()) {
    /*std::cout<<"\nPoints 1 and 2 are the same";
    std::cout<<"\nPath: "<<utility_.toString(p);
    std::cout<<"\nSegment point 1: "<<utility_.toString(req_.bezierInfo.at(0).segmentPoints.at(1));
    std::cout<<"\nSegment point 2: "<<utility_.toString(req_.bezierInfo.at(0).segmentPoints.at(2))<<"\n";*/
    req_.bezierInfo.at(0).segmentPoints.at(2) = p_copy.points.at(inc).motionState;
    //std::cin.get();
    differentPoint = utility_.positionDistance(req_.bezierInfo.at(0).segmentPoints.at(1).positions, 
          req_.bezierInfo.at(0).segmentPoints.at(2).positions) > 0.01;
    inc++;

    if(inc == p_copy.points.size()) {
      //ROS_INFO("Cannot plan Bezier, returning same Path at 422");
      type_ = ALL_STRAIGHT_SEGMENTS;
      return result;
    }
  }

  ROS_INFO("stop: %i", stop);

  // Go through the path's knot points
  //std::cout<<"\np.points.size(): "<<p.points.size()<<"\n";
  for(uint8_t i=1;i<stop;i++) {
    std::cout<<"\n---i: "<<(int)i<<"---\n";

    // Check that all of the points are different
    if(utility_.positionDistance(req_.bezierInfo.at(i-1).segmentPoints.at(0).positions, 
          req_.bezierInfo.at(i-1).segmentPoints.at(1).positions) > 0.01 &&
        (utility_.positionDistance(req_.bezierInfo.at(i-1).segmentPoints.at(1).positions, 
          req_.bezierInfo.at(i-1).segmentPoints.at(2).positions) > 0.01) )
    {
      ROS_INFO("In if");

      BezierCurve bc;
      bc.print_ = print_;

      // Set segment points
      std::vector<ramp_msgs::MotionState> segment_points = 
        req_.bezierInfo.at(i-1).segmentPoints;
      
      double theta = utility_.findAngleFromAToB(
          segment_points.at(0).positions, segment_points.at(1).positions);

      // If we are starting with a curve
      // For transition trajectories, the segment points are the 
      // control points, so we have all the info now
      // TODO: Change bezierStart to check u>0
      //if(bezierStart && i==1) {
      if(req_.bezierInfo.at(0).u_0 > 0 && i==1) {
        std::cout<<"\nIn if transition or bezierStart\n";
        
        ramp_msgs::MotionState ms_maxVA = getMaxMS();
        
        double lambda = (req_.bezierInfo.at(i-1).controlPoints.size() > 0) ?  
          req_.bezierInfo.at(i-1).l : getControlPointLambda(segment_points);


        // TODO: Make a method to return a BezierInitializer
        // TODO: Just use req_.bezierInfo?
        ramp_msgs::BezierInfo bi;
        bi.segmentPoints  = segment_points;
        bi.controlPoints  = req_.bezierInfo.at(i-1).controlPoints;
        bi.ms_maxVA       = ms_maxVA;
        bi.u_0            = req_.bezierInfo.at(i-1).u_0;
        bi.u_dot_0        = req_.bezierInfo.at(i-1).u_dot_0;
        bi.ms_begin       = p_copy.points.at(0).motionState;
        bi.l              = lambda;

        bc.init(bi, path_.points.at(0).motionState);
      } // end if bezierStart
      


      // If a "normal" bezier trajectory,
      else {
        std::cout<<"\nIn else a normal trajectory\n";

        // Get lambda value for segment points
        double lambda = (req_.bezierInfo.at(i-1).controlPoints.size() > 0) ?  req_.bezierInfo.at(i-1).l :
                                                        getControlPointLambda(segment_points);

        ramp_msgs::MotionState ms_maxVA = getMaxMS();

        // TODO: Make a method to return a BezierInitializer
        ramp_msgs::BezierInfo bi;
        bi.segmentPoints  = segment_points;
        bi.l              = lambda;
        bi.ms_maxVA       = ms_maxVA;
        //bi.ms_begin       = path_.points.at(0).motionState;

        bc.init(bi, path_.points.at(0).motionState);
        ROS_INFO("Done initializing curve");
      } // end else "normal" trajectory


      // Verify the curve
      if(bc.verify()) {
        ROS_INFO("Curve is verified, generating points");
        // Generate the curve
        bc.generateCurve();
        result.push_back(bc);
      }
      else if(type_ == TRANSITION) {
        ROS_INFO("Curve not verified, doing a transition so setting 0 velocity for KP: %s", 
            utility_.toString(path_.points.at(1).motionState).c_str());

        uint8_t num_dof = path_.points.at(1).motionState.velocities.size();
        for(uint8_t i_v=0;i_v<num_dof;i_v++) {
          path_.points.at(1).motionState.velocities.at(i_v) = 0;
        }

        type_ = ALL_STRAIGHT_SEGMENTS;
      }
      else {
        //ROS_INFO("Curve not verified, but not a transition trajectory");
        type_ = ALL_STRAIGHT_SEGMENTS;
      }
    } // end if
    else {
      //ROS_WARN("Two of the three segment points for Bezier curve are too close");
      type_ = ALL_STRAIGHT_SEGMENTS;
    }
  } // end for

  //ROS_INFO("Outside of for");

  if(type_ != ALL_STRAIGHT_SEGMENTS) {

    if(type_ == TRANSITION) {
      //ROS_INFO("In type == transition");
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.erase(p.points.begin()+2);
      p.points.insert(p.points.begin()+2, 
          utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }

    // If we are switching and have more than 1 curve
    else if(req_.bezierInfo.size() > 1) {
      //std::cout<<"\nIn first if\n";
      //std::cout<<"\np.points.size(): "<<p.points.size();
      
      // Insert the 1st curve's last CP after the 1st KP
      // used to be +1
      //p.points.insert( p.points.begin()+2, 
          //utility_.getKnotPoint( result.at(0).points_.at(result.at(0).points_.size()-1)));

      //ROS_INFO("Path p: %s", utility_.toString(p).c_str());
      
      // Remove the 3rd KP 
      // used to be begin()+2, but i changed it when doing two cuvres and restarting CC
      //p.points.erase( p.points.begin() + 2 );
      //p.points.erase( p.points.begin() + 3 );
      // Insert the 2nd curve's first and last CP
      //p.points.insert(p.points.begin()+3, utility_.getKnotPoint(result.at(1).points_.at(0)));
      //p.points.insert(p.points.begin()+4, 
          //utility_.getKnotPoint(result.at(1).points_.at(result.at(1).points_.size()-1)));

      // When switching at gen==21, robot is in the 1st curve
      // Need to erase the start and finish of 1st curve and add in CPs
      //ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+2) ).c_str());
      p.points.erase( p.points.begin()+2 );
      //ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin()+1 );
      // Insert the 1st curve's last CP
      p.points.insert( p.points.begin()+1, 
          utility_.getKnotPoint( result.at(0).points_.at(result.at(0).points_.size()-1)));

      //ROS_INFO("Path p: %s", utility_.toString(p).c_str());
      
      // Erase 2nd curve's middle segment point (it's in path)
      //ROS_INFO("Actually Erasing: %s", utility_.toString( *(p.points.begin()+2) ).c_str());
      // Removed for gen==21 on switch
      //p.points.erase( p.points.begin()+2 );
      
      // Insert the 2nd curve's 1st and last CPs
      p.points.insert(p.points.begin()+2, utility_.getKnotPoint(result.at(1).points_.at(0)));
      p.points.insert(p.points.begin()+3, 
          utility_.getKnotPoint(result.at(1).points_.at(result.at(1).points_.size()-1)));
    }

    // If already moving on curve
    else if(req_.bezierInfo.at(0).u_0 > 0) {
      //ROS_INFO("In else if bezierStart");
      // Commented out when gen == 23 for the switch. at CC after restart, it was removing 1,1 and 3.5,2
      //ROS_INFO("Erasing: %s", utility_.toString( *(p.points.begin()+2) ).c_str());
      //p.points.erase( p.points.begin() + 2 );
      //ROS_INFO("Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin() + 1 );
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }
    // Else not on a curve
    // maybe erase twice if two curves
    else {
      //ROS_INFO("In else");
      //ROS_INFO("Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
      p.points.erase( p.points.begin() + 1 );
      
      // Added for when gen==22, the robot does not start on curve so must erase 2
      /*if(req_.bezierInfo.size() > 1) {
        ROS_INFO("Erasing: %s", utility_.toString( *(p.points.begin()+1) ).c_str());
        p.points.erase( p.points.begin() + 1 );
      }*/
      
      // Insert
      p.points.insert(p.points.begin()+1, utility_.getKnotPoint(result.at(0).points_.at(0)));
      p.points.insert(p.points.begin()+2, utility_.getKnotPoint(result.at(0).points_.at(result.at(0).points_.size()-1)));
    }
  }

  //ROS_INFO("Leaving MobileBase::bezier");
  return result;
} // End bezier



/** Print Reflexxes input and output for the latest call */
void MobileBase::printReflexxesSpinInfo() const {
  std::cout<<"\n\n*****************************************************************************";
  std::cout<<"\nCalled reflexxes with input:";
  
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[0]: "<<
                reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[1]: "<<
                reflexxesData_.inputParameters->CurrentPositionVector->VecData[1];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[2]: "<<
                reflexxesData_.inputParameters->CurrentPositionVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]: "<<
                  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[1]: "<<
                reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[2]: "<<
                reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0]: "<<
                  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1]: "<<
                reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2]: "<<
                reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2];
  
  
  std::cout<<"\n\nOutput: ";
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[0]: "<<
                reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[1]: "<<
                reflexxesData_.outputParameters->NewPositionVector->VecData[1];
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[2]: "<<
                reflexxesData_.outputParameters->NewPositionVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.outputParameters->NewVelocityVector->VecData[0]: "<<
                reflexxesData_.outputParameters->NewVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewVelocityVector->VecData[1]: "<<
                reflexxesData_.outputParameters->NewVelocityVector->VecData[1];
  std::cout<<"\nreflexxesData_.outputParameters->NewVelocityVector->VecData[2]: "<<
                reflexxesData_.outputParameters->NewVelocityVector->VecData[2];
  
  std::cout<<"\n\nreflexxesData_.outputParameters->NewAccelerationVector->VecData[0]: "<<
                reflexxesData_.outputParameters->NewAccelerationVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewAccelerationVector->VecData[1]: "<<
                reflexxesData_.outputParameters->NewAccelerationVector->VecData[1];
  std::cout<<"\nreflexxesData_.outputParameters->NewAccelerationVector->VecData[2]: "<<
                reflexxesData_.outputParameters->NewAccelerationVector->VecData[2];

  std::cout<<"\n\nResult value: "<<reflexxesData_.resultValue;
  std::cout<<"\nFinalStateReached: "<<finalStateReached();
  std::cout<<"\n*****************************************************************************";
} // End printReflexxesInfo




/** Execute one iteration of the Reflexxes control function */
const trajectory_msgs::JointTrajectoryPoint MobileBase::spinOnce() {

  // Calling the Reflexxes OTG algorithm
  reflexxesData_.resultValue = reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, 
                                                  reflexxesData_.outputParameters, 
                                                  reflexxesData_.flags);


  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  trajectory_msgs::JointTrajectoryPoint point = buildTrajectoryPoint(reflexxesData_);


  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = 
    *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = 
    *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = 
    *reflexxesData_.outputParameters->NewAccelerationVector;

  return point;
} // End spinOnce




/** Given Reflexxes data, return a trajectory point */
const trajectory_msgs::JointTrajectoryPoint MobileBase::buildTrajectoryPoint(const ReflexxesData data) {
  trajectory_msgs::JointTrajectoryPoint point;




  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      
    // If Reflexxes has not been called yet
    if(data.outputParameters->NewPositionVector->VecData[0] == -99) {
      point.positions.push_back(data.inputParameters->CurrentPositionVector->VecData[i]);
      point.velocities.push_back(data.inputParameters->CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(data.inputParameters->CurrentAccelerationVector->VecData[i]);
    }
    
    // If selection vector is true
    else if(reflexxesData_.inputParameters->SelectionVector->VecData[i]) {
      point.positions.push_back(data.outputParameters->NewPositionVector->VecData[i]);
      point.velocities.push_back(data.outputParameters->NewVelocityVector->VecData[i]);
      point.accelerations.push_back(data.outputParameters->NewAccelerationVector->VecData[i]);

      if(i == 2) {
        point.positions.at(2) = utility_.displaceAngle(prevKP_.positions.at(2), 
                                  data.outputParameters->NewPositionVector->VecData[i]);
      }
    } // end if selection vector is true
    
    // Else if we're at orientation dof
    else if(i == 2) {
      
      // If straight-line paths, make theta be towards next point
      /*double theta = utility_.findAngleFromAToB( data.inputParameters->CurrentPositionVector->VecData[0],
                                                 data.inputParameters->CurrentPositionVector->VecData[1],
                                                 data.outputParameters->NewPositionVector->VecData[0],
                                                 data.outputParameters->NewPositionVector->VecData[1]);*/
      
      double theta = utility_.findAngleFromAToB( path_.points.at(i_kp_-1).motionState.positions,
                                                 path_.points.at(i_kp_).motionState.positions);
                                                 //data.outputParameters->NewPositionVector->VecData[0],
                                                 //data.outputParameters->NewPositionVector->VecData[1]);



      //ROS_INFO("theta: %f", theta);
      // Get angular velocity
      double w = (theta - data.inputParameters->CurrentPositionVector->VecData[2]) / 
                  CYCLE_TIME_IN_SECONDS;

      // Push on p,v,a
      // TODO: Manually keep acceleration?
      point.positions.push_back(theta);
      point.velocities.push_back(w);
      point.accelerations.push_back(0);

      // Set values in Reflexxes
      // TODO: Necessary?
      reflexxesData_.outputParameters->NewPositionVector->VecData[2] = theta;
      reflexxesData_.outputParameters->NewVelocityVector->VecData[2] = w;
    } // end else-if orientation
      
    // Else, just push on the current value
    else {
      point.positions.push_back(
          reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
      point.velocities.push_back(
          reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(
          reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i]);
    }
  } // end for 


  // The timeFromStart_ is the time of the previous point 
  // plus the cycle period
  point.time_from_start = timeFromStart_;
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  return point;
} // End buildTrajectoryPoint



const std::vector<uint8_t> MobileBase::getCurveKPs(const std::vector<BezierCurve> curves) const {
  std::vector<uint8_t> result;
  
  for(uint8_t i_c=0; i_c < curves.size(); i_c++) {
    //ROS_INFO("curves.at(%i): %s", (int)i_c, utility_.toString(curves.at(i_c).points_.at(0)).c_str());
    
    for(uint8_t i_kp=0;i_kp < path_.points.size();i_kp++) {
      //ROS_INFO("path.poinst.at(%i): %s", (int)i_kp, utility_.toString(path_.points.at(i_kp).motionState).c_str());
      
      if(utility_.positionDistance(curves.at(i_c).points_.at(0).positions,
                                   path_.points.at(i_kp).motionState.positions) < 0.0001)
      {
        result.push_back(i_kp);
        i_kp = path_.points.size();
      } // end if
      else {
      }
    } // end for kp
  } // end for i_c

  return result;
} // End getCurveKPs




// Service callback, the input is a path and the output a trajectory
bool MobileBase::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req, ramp_msgs::TrajectoryRequest::Response& res) {
  //std::cout<<"\nTrajectory Request Received: "<<utility_.toString(req)<<"\n";
  
  // If there's less than 3 points,
  // make it have straight segments
  if(req.path.points.size() < 3 && req.type != TRANSITION) {
    ROS_WARN("Path request size < 3");
    req.type = ALL_STRAIGHT_SEGMENTS;
  }

  // If there's 2 points and they have the same x,y position
  if(req.path.points.size() == 2 && 
      utility_.positionDistance(req.path.points.at(0).motionState.positions,
                                req.path.points.at(1).motionState.positions) < 0.001)
  {
    res.trajectory.trajectory.points.push_back(utility_.getTrajectoryPoint(req.path.points.at(0).motionState));
    res.trajectory.i_knotPoints.push_back(0);
    return true;
  }
  

  // Initialize with request
  init(req);
  req_ = req;

  // Set start time
  t_started_ = ros::Time::now();

  // Set the trajectory's resolution rate
  res.trajectory.resolutionRate = CYCLE_TIME_IN_SECONDS;

  std::vector<BezierCurve> curves;

  // Use Bezier curves to smooth path
  if(type_ != ALL_STRAIGHT_SEGMENTS) {
    ROS_INFO("Path before Bezier: %s", utility_.toString(path_).c_str());
    curves = bezier(path_, type_ == TRANSITION);
    ROS_INFO("Path after Bezier: %s", utility_.toString(path_).c_str());


    // Currently adding 0 for both because 
    i_cs = getCurveKPs(curves);
    /*ROS_INFO("Curve indices: ");
    for(int i=0;i<i_cs.size();i++) {
      ROS_INFO("i_cs[%i]: ", (int)i_cs.at(i));
    }*/
  } // end if curves

  if(curves.size() == 0) {
    type_ = ALL_STRAIGHT_SEGMENTS;
  }

  // Print curves
  /*if(print_) {
    for(int c=0;c<curves.size();c++) {
      std::cout<<"\nCurve "<<c<<": ";
      for(int p=0;p<curves.at(c).points_.size();p++) {
        std::cout<<"\n"<<utility_.toString(curves.at(c).points_.at(p));
      }
    }
  }*/
 

  // Push 0 onto knot point indices
  res.trajectory.i_knotPoints.push_back(0);

 
  uint8_t c=0;
  // Go through every knotpoint in the path
  // (or until timeCutoff has been reached)
  for (i_kp_ = 1; i_kp_<path_.points.size(); i_kp_++) {
    ROS_INFO("i_kp_: %i", (int)i_kp_);
    reflexxesData_.resultValue = 0;

    // Push the initial state onto trajectory
    // And set previous knot point
    if(i_kp_ == 1) {
      res.trajectory.trajectory.points.push_back(buildTrajectoryPoint(reflexxesData_));
      prevKP_ = res.trajectory.trajectory.points.at(0);
    }

    // *** Set the new target ***
    setTarget(path_.points.at(i_kp_).motionState);
    //ROS_INFO("Prev KP: %s", utility_.toString(prevKP_).c_str());
    //ROS_INFO("Target: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str());




    /** Bezier */
    // If its a Bezier curve traj, and we're at a Bezier point
    // all points between first and last are bezier point
    //std::cout<<"\nc: "<<c<<" i_cs.size(): "<<i_cs.size()<<"\n";
    //std::cout<<"\ncurves.size(): "<<curves.size()<<"\n";
    if( (c < i_cs.size() && path_.points.size() > 2 && i_kp_ == i_cs.at(c)+1))
    {
      ROS_INFO("At Bezier Curve %i", c);
      ROS_INFO("timeFromStart_: %f", timeFromStart_.toSec());
      //std::cout<<"\ncurves.at("<<(int)c<<").size(): "<<curves.at(c).points_.size();

      // Insert all points on the curves into the trajectory
      // TODO: Why am I not pushing on every point of the curve?
      for(uint32_t p=1;p<curves.at(c).points_.size();p++) {
        insertPoint(curves.at(c).points_.at(p), res);

        // If it's the first or last point on the curve, 
        // push the index to knot point vector
        // Only add the first point on 1st curve - other curves will
        // have their first points as the last point from reflexxes straight-line
        if(p==curves.at(c).points_.size()-1) //||
            //(c == i_cs.at(0) && p == 1) ) 
        {
          res.trajectory.i_knotPoints.push_back(
                          res.trajectory.trajectory.points.size() - 1 );
        } // end if knot point
      } // end for


      // Create a BezierInfo for the curve to return with the trajec
      ramp_msgs::BezierInfo bi;
      bi.segmentPoints  = curves.at(c).segmentPoints_;
      bi.controlPoints  = curves.at(c).controlPoints_;
      bi.ms_maxVA       = curves.at(c).ms_max_;
      bi.ms_initialVA   = curves.at(c).ms_init_;
      bi.numOfPoints    = curves.at(c).points_.size();
      bi.u_0            = req_.bezierInfo.at(c).u_0;
      bi.u_dot_0        = curves.at(c).u_dot_0_;
      bi.l              = curves.at(c).l_;
      bi.u_target       = curves.at(c).u_target_;
      res.trajectory.curves.push_back(bi);

      if(i_kp_ == path_.points.size()-1) {
        res.trajectory.i_curveEnd = res.trajectory.trajectory.points.size()-1;
      }
      else {
        res.trajectory.i_curveEnd = res.trajectory.trajectory.points.size();
      }

      //ROS_INFO("Setting i_curveEnd: %i", (int)res.trajectory.trajectory.points.size()-1);

      c++;
    } // end if bezier


    /** Straight Line Segment */
    // Else if straight-line segment
    else {
      ROS_INFO("In else");

      // Get rotation if needed
      double trajec_size = res.trajectory.trajectory.points.size();

      trajectory_msgs::JointTrajectoryPoint last = 
            res.trajectory.trajectory.points.at(trajec_size-1);

      trajectory_msgs::JointTrajectoryPoint next_knot =
            utility_.getTrajectoryPoint(path_.points.at(i_kp_).motionState);

      /*ROS_INFO("=== Orientation Information ===");
      ROS_INFO("last: %s", utility_.toString(last).c_str());
      ROS_INFO("next_knot: %s", utility_.toString(next_knot).c_str());
      ROS_INFO("utility_.findAngleFromAToB(last, next_knot): %f", utility_.findAngleFromAToB(last, next_knot));
      ROS_INFO("utility_.findDistanceBetweenAngles(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot)): %f",
                  utility_.findDistanceBetweenAngles(last.positions.at(2), 
                          utility_.findAngleFromAToB(last, next_knot)));*/


      // Check for goal because the robot should not rotate
      // if it overshoots the goal.
      if(!checkTarget() || type_ == ALL_STRAIGHT_SEGMENTS) {

        // Set orientation threshold that requires a rotation 
        // before continuing to the next knot point
        double threshold = 0.15; 
        //ROS_INFO("threshold: %f", threshold);

        // If we need to rotate towards the next knot point
        // 0.0872664 = 5 degrees
        if(fabs(utility_.findDistanceBetweenAngles(last.positions.at(2), 
                utility_.findAngleFromAToB(last, next_knot))) > threshold) 
        {
          //ROS_INFO("Calling rotate");
          std::vector<trajectory_msgs::JointTrajectoryPoint> rotate_points = 
            rotate(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot),
                    last.velocities.at(2), last.accelerations.at(2));

          for(uint16_t p=0;p<rotate_points.size();p++) {
            res.trajectory.trajectory.points.push_back(rotate_points.at(p));
          } // end for


          if(timeFromStart_ < timeCutoff_) {
            res.trajectory.i_knotPoints.push_back(res.trajectory.trajectory.points.size() - 1);
          }

          setSelectionVector();
          reflexxesData_.resultValue = 0;
          prevKP_ = res.trajectory.trajectory.points.at(
              res.trajectory.trajectory.points.size() - 1);
        } // end if rotate
        /*else {
          ROS_INFO("No rotation needed");
        }*/
      } // end if final state is not already reached
      /*else {
        ROS_INFO("Check goal returns true");
      }*/


      setTarget(path_.points.at(i_kp_).motionState);
      //ROS_INFO("Prev KP: %s", utility_.toString(prevKP_).c_str());
      //ROS_INFO("Target: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str());
      
      // Check they are not the same point
      if(utility_.positionDistance(res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size()-1).positions, 
            path_.points.at(i_kp_).motionState.positions) > 0.0001)
      {
        //ROS_INFO("Pushing on points b/c dist: %f", utility_.positionDistance(res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size()-1).positions, 
              //path_.points.at(i_kp_).motionState.positions));
              
        // We go to the next knotpoint only once we reach this one
        while (!finalStateReached()) {

          trajectory_msgs::JointTrajectoryPoint p = spinOnce();
          //ROS_INFO("p: %s", utility_.toString(p).c_str());
          //ROS_INFO("result: %i", reflexxesData_.resultValue);

          // Compute the motion state at t+1 and save it in the trajectory
          res.trajectory.trajectory.points.push_back(p);
        } // end while

        // Once we reached the target, we set that the latest point is a knotpoint
        if(timeFromStart_ < timeCutoff_) {
          res.trajectory.i_knotPoints.push_back(res.trajectory.trajectory.points.size() - 1);
        }
      } // end if different points

      // Else if there's only 2 points and the current point and next knot point are the same
      // It was a path with the same point
      else if(req.path.points.size() == 2) {
        //ROS_INFO("Last position and next knot point are the same position, path size == 2");
        res.trajectory.trajectory.points.push_back(res.trajectory.trajectory.points.at(0));
      }
      //else {
        //ROS_INFO("Last position and next knot point are the same position, path size > 2");
      //}
    } // end if

    // Set previous knot point
    prevKP_ = res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size() - 1);


    // Check if Reflexxes overshot target
    if(!lastPointClosest(res.trajectory)) {
      ROS_INFO("Last point is not closest");

      res.trajectory.trajectory.points.pop_back();
      res.trajectory.i_knotPoints.at(res.trajectory.i_knotPoints.size()-1) =
       res.trajectory.trajectory.points.size()-1; 

      timeFromStart_ -= ros::Duration(CYCLE_TIME_IN_SECONDS);
      
      // If it's the first kp and there's no curve
      if(i_kp_ == 1 && type_ != PARTIAL_BEZIER) {
        res.trajectory.i_knotPoints.pop_back();
      }
    } // end if checking Reflexxes overshooting

      //ROS_INFO("Reached target: %s \nAt state: %s", utility_.toString(path_.points.at(i_kp_).motionState).c_str(),
            //utility_.toString(res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size()-1)).c_str());
  } // end for each knot point (outer-most loop)
 

  // Lastly, set newPath in case the path changed
  res.newPath = path_;
 
  return true;
} // End trajectoryRequest callback



/** This performs a rotation using Reflexxes */
const std::vector<trajectory_msgs::JointTrajectoryPoint> MobileBase::rotate(const double start, const double goal, const double start_v, const double start_a) {
  //ROS_INFO("In MobileBase::rotate, start: %f goal: %f start_v: %f start_a: %f", start, goal, start_v, start_a);
  std::vector<trajectory_msgs::JointTrajectoryPoint> result;

  double targetTheta = utility_.findDistanceBetweenAngles(start, goal);

  // Set Selection Vector up for rotation
  setSelectionVectorRotation();

  // Set current velocity and acceleration values for x,y to 0
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = 0;

  // Set current values for orientation
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = 0;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2] = start_v;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2] = start_a;

  // Set target values
  reflexxesData_.inputParameters->TargetPositionVector->VecData[2] = targetTheta;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[2] = 0;
  
  // Call Reflexxes until goal is reached
  reflexxesData_.resultValue = 0;
  while(!finalStateReached()) {
    trajectory_msgs::JointTrajectoryPoint p = spinOnce();
    result.push_back(p);
  }
  
  if(result.size() > 0) {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = 
      result.at(result.size()-1).positions.at(2);
  }


  //ROS_INFO("Exiting rotate");
  return result;
} // End rotate




const bool MobileBase::checkTarget() {
  std::vector<double> p_current, p_target, v_current, v_target;
  
  for(uint8_t i=0;i<3;i++) {
    p_current.push_back(reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
    p_target.push_back( reflexxesData_.inputParameters->TargetPositionVector->VecData[i] );
    
    v_current.push_back(reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
    v_target.push_back( reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] );
  }

  if(fabs(utility_.positionDistance(p_current, p_target)) < 0.0001 &&
      fabs(utility_.positionDistance(v_current, v_target)) < 0.0001 ) 
  {
    return true; 
  }

  return false;
}


const bool MobileBase::lastPointClosest(const ramp_msgs::RampTrajectory traj) const {
  //ROS_INFO("In lastPointClosest");
  
  std::vector<double> target_p, target_v; 
  target_p.push_back(reflexxesData_.inputParameters->TargetPositionVector->VecData[0]);
  target_p.push_back(reflexxesData_.inputParameters->TargetPositionVector->VecData[1]);
  target_p.push_back(reflexxesData_.inputParameters->TargetPositionVector->VecData[2]);

  target_v.push_back(reflexxesData_.inputParameters->TargetVelocityVector->VecData[0]);
  target_v.push_back(reflexxesData_.inputParameters->TargetVelocityVector->VecData[1]);
  target_v.push_back(reflexxesData_.inputParameters->TargetVelocityVector->VecData[2]);

  //ROS_INFO("After setting targets");

  //ROS_INFO("traj size: %i", (int)traj.trajectory.points.size());
  trajectory_msgs::JointTrajectoryPoint last = traj.trajectory.points.at(traj.trajectory.points.size()-1);
  trajectory_msgs::JointTrajectoryPoint nextToLast_to_last = 
    traj.trajectory.points.at(traj.trajectory.points.size()-2);

  //ROS_INFO("last: %s \nnext_to_last: %s", utility_.toString(last).c_str(), utility_.toString(nextToLast_to_last).c_str());

  //ROS_INFO("Done setting last and nextToLast_to_last");
  //ROS_INFO("last.positions size(): %i target_p.size: %i", (int)last.positions.size(), (int)target_p.size());
 
  double dist_last_p = utility_.getEuclideanDist(last.positions, target_p);
  double dist_nextToLast_p = utility_.getEuclideanDist(nextToLast_to_last.positions, target_p);

  //ROS_INFO("dist_last_p: %f dist_nextToLast_p: %f", dist_last_p, dist_nextToLast_p);
 
  double dist_last_v = utility_.getEuclideanDist(last.velocities, target_v);
  double dist_nextToLast_v = utility_.getEuclideanDist(nextToLast_to_last.velocities, target_v);
  
  //ROS_INFO("dist_last_v: %f dist_nextToLast_v: %f", dist_last_v, dist_nextToLast_v);

  double dist_last = dist_last_p + dist_last_v;
  double dist_nextToLast = dist_nextToLast_p + dist_nextToLast_v;
  
  //ROS_INFO("dist_last: %f dist_nextToLast: %f fabs(dist_last - dist_nextToLast): %f", dist_last, dist_nextToLast, fabs(dist_last - dist_nextToLast));

  //ROS_INFO("Exiting MobileBase::lastPointClosest");
  return (dist_last < dist_nextToLast) && fabs(dist_last - dist_nextToLast) > 0.0001;
}


// Returns true if the target has been reached
bool MobileBase::finalStateReached() const {
  
  if(timeFromStart_ >= timeCutoff_) {
    ROS_WARN("timeFromStart_ > timeCutoff_ (%f)", timeCutoff_.toSec());
    ROS_WARN("Check this trajectory request");
  }
 
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED)  
      || (timeFromStart_ >= timeCutoff_);
} // End finalStateReached



