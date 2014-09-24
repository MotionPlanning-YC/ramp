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


  
  // Use time synchronization so the robot drives in a straight line towards goal 
  reflexxesData_.flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;


  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[2] = PI/4;
  

  // Maximum acceleration
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 0.66;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1] = 0.66;
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
  resultValue_ = 0;
} // End initReflexxes



/** Initialize class object with a request */
// TODO: change 3 booleans to 1 enum
void MobileBase::init(const ramp_msgs::TrajectoryRequest::Request req) {
  std::cout<<"\nRequest received: "<<utility_.toString(req)<<"\n";

  bezierStart = req.startBezier;
  //if(req.bezierInfo.u_0 > 0)
    //std::cout<<"\nBezier Info passed in: "<<utility_.toString(req.bezierInfo);

  // Store the path
  path_ = req.path;

  // Set print
  print_ = req.print;
  
  // Set trajectory type
  type_ = (TrajectoryType)req.type;

  // Set curve point
  curveStart_ = req.curve_start;

  // Initialize Reflexxes
  initReflexxes();
 
  // Set the initial conditions of the reflexxes library
  setInitialMotion();

  // Set SelectionVector
  setSelectionVector();

  // Starting time
  t_started_ = ros::Time::now();

  // Set the time to cutoff generating points
  timeCutoff_ = ros::Duration(3.5);
  
  //std::cout<<"\nLeaving init\n";
} // End init



/** This method sets the new target of Reflexxes */
void MobileBase::setTarget(const ramp_msgs::MotionState ms) {
  
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(i);
    if(ms.velocities.size() > 0) {
      reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(i);
    }
  }  
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
  std::cout<<"\nlambda: "<<lambda;
  ramp_msgs::MotionState X0, X1, X2, p0, p1, p2;

  p0 = segment_points.at(0);
  p1 = segment_points.at(1);
  p2 = segment_points.at(2);
  
  X1 = segment_points.at(1);


  // Find how far along segment we already are
  // Can use x or y...here we use x
  double min_lambda = (path_.points.at(0).motionState.positions.at(0) - segment_points.at(0).positions.at(0)) 
                      / (segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0));
  std::cout<<"\nmin_lambda: "<<min_lambda;
  
  if(lambda <= (min_lambda+0.1)) {
    std::cout<<"\nFailing because lambda <= min_lambda";
    return false;
  }
  

  
  double l_s1 = utility_.positionDistance(segment_points.at(1).positions, segment_points.at(0).positions);
  double l_s2 = utility_.positionDistance(segment_points.at(2).positions, segment_points.at(1).positions);

  if(l_s2 < l_s1) {
    std::cout<<"\nl_s2 < l_s1";
    X2.positions.push_back( (1-lambda)*p1.positions.at(0) + lambda*p2.positions.at(0) );
    X2.positions.push_back( (1-lambda)*p1.positions.at(1) + lambda*p2.positions.at(1) );
    X2.positions.push_back(utility_.findAngleFromAToB(p1.positions, p2.positions));

    double theta = utility_.findAngleFromAToB(p0.positions, p1.positions);

    X0.positions.push_back( p1.positions.at(0) - l_s2*cos(theta) );
    X0.positions.push_back( p1.positions.at(1) - l_s2*sin(theta) );
    X0.positions.push_back( theta );
  }
  else {
    std::cout<<"\nl_s2 >= l_s1";
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
    std::cout<<"\nFailing because A and B = 0";
    return false;
  }
  
  return true;
} // End lambdaOkay




/** Returns a lambda value that will lead to defined Bezier equations */
const double MobileBase::getControlPointLambda(const std::vector<ramp_msgs::MotionState> segment_points) const {
  std::vector<double> result;

  std::cout<<"\npath.points.at(0): "<<utility_.toString(path_.points.at(0).motionState);
  
  double l_s1 = utility_.positionDistance(segment_points.at(1).positions, segment_points.at(0).positions);
  double l_s2 = utility_.positionDistance(segment_points.at(2).positions, segment_points.at(1).positions);

  // Start transition trajectories right away,
  // otherwise, try to go straight for a while
  double lambda = type_ == TRANSITION ? 0. : 0.5;

  double min_lambda = (path_.points.at(0).motionState.positions.at(0) - segment_points.at(0).positions.at(0)) 
                      / (segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0));
  std::cout<<"\nmin_lambda: "<<min_lambda<<"\n";
  if(min_lambda > 1) {
    std::cin.get();
  }

  while(!lambdaOkay(segment_points, lambda)) {
    if(type_ == TRANSITION) {
      lambda+=0.05; 
    }
    else {
      lambda+=0.05;
    }

    // TODO: Report/Handle error
    if(lambda < 0.1) {
      lambda = 0.9;
    }

    if(lambda > 0.9) {
      lambda = 0.1;
    }
  }

  std::cout<<"\nlambda: "<<lambda;

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
  std::vector<BezierCurve> result;

  // Check that all of the points are different
  if(utility_.positionDistance(req_.bezierInfo.segmentPoints.at(0).positions, 
        req_.bezierInfo.segmentPoints.at(1).positions) > 0.01 &&
      (utility_.positionDistance(req_.bezierInfo.segmentPoints.at(1).positions, 
        req_.bezierInfo.segmentPoints.at(2).positions) > 0.01) )
  {

    ramp_msgs::Path p_copy = p;

    // Set the index of which knot point to stop at
    int stop = 2; 

    // Go through the path's knot points
    //std::cout<<"\np.points.size(): "<<p.points.size()<<"\n";
    for(uint8_t i=1;i<stop;i++) {
      //std::cout<<"\n---i: "<<(int)i<<"---\n";
      BezierCurve bc;
      bc.print_ = print_;

      // Set segment points
      std::vector<ramp_msgs::MotionState> segment_points = 
        req_.bezierInfo.segmentPoints;
      
      double theta = utility_.findAngleFromAToB(
          segment_points.at(0).positions, segment_points.at(1).positions);

      // If we are starting with a curve
      // For transition trajectories, the segment points are the 
      // control points, so we have all the info now
      if(bezierStart) {
        std::cout<<"\nIn if transition or bezierStart\n";

        
        ramp_msgs::MotionState ms_maxVA = getMaxMS();


        // TODO: Make a method to return a BezierInitializer
        // TODO: Just use req_.bezierInfo?
        ramp_msgs::BezierInfo bi;
        bi.segmentPoints = segment_points;
        bi.controlPoints = req_.bezierInfo.controlPoints;
        bi.ms_maxVA = ms_maxVA;
        bi.u_0 = req_.bezierInfo.u_0;
        bi.u_dot_0 = req_.bezierInfo.u_dot_0;
        bi.ms_begin = p_copy.points.at(0).motionState;

        bc.init(bi, path_.points.at(0).motionState);
      } // end if bezierStart
      


      // If a "normal" bezier trajectory,
      else {
        std::cout<<"\nIn else a normal trajectory\n";

        // Get lambda value for segment points
        double lambda = (req_.bezierInfo.lambda > 0) ?  req_.bezierInfo.lambda :
                                                        getControlPointLambda(segment_points);

        ramp_msgs::MotionState ms_maxVA = getMaxMS();

        // TODO: Make a method to return a BezierInitializer
        ramp_msgs::BezierInfo bi;
        bi.segmentPoints  = segment_points;
        bi.lambda         = lambda;
        bi.ms_maxVA       = ms_maxVA;
        //bi.ms_begin       = path_.points.at(0).motionState;

        bc.init(bi, path_.points.at(0).motionState);
      } // end else "normal" trajectory


      // Generate the curve
      bc.generateCurve();

      // If the curve was valid,
      if(bc.points_.size() > 0) {
        //std::cout<<"\nReplacing knot points, bc.points.size(): "<<bc.points_.size();
        //std::cout<<"\nRemoving knot point: "<<utility_.toString(p.points.at(i).motionState);
        // Remove knot point
        p.points.erase(p.points.begin()+i);

        // If we started with a Bezier
        if(bezierStart) {
          //std::cout<<"\nInserting bc.points_.at(size()-1): "<<utility_.toString(bc.points_.at(bc.points_.size()-1));
          p.points.insert(p.points.begin()+i, utility_.getKnotPoint(bc.points_.at(bc.points_.size()-1)));
        }
        
        // Else we are starting with a straight-line
        // Replace the next knot point with the first and 
        //  last points on the curve
        // The first point is control point c0 with v 
        //  and a information, 
        // The last point is control point c2 with v 
        //  and a information,
        // If it's a transition, the first point on curve 
        //  is the first point that's already there
        else if(type_ != TRANSITION) {
          //std::cout<<"\nInserting bc.points_.at(0): "<<utility_.toString(bc.points_.at(0));
          //std::cout<<"\nInserting bc.points_.at(size()-1): "<<utility_.toString(bc.points_.at(bc.points_.size()-1));
          p.points.insert(p.points.begin()+i, utility_.getKnotPoint(bc.points_.at(0)));
          p.points.insert(p.points.begin()+i+1, utility_.getKnotPoint(bc.points_.at(bc.points_.size()-1)));
        }
        

        // Push back the curve
        result.push_back(bc);
      } // end if
    } // end for
  } // end if
  else {
    ROS_WARN("Two of the three segment points for Bezier curve are too close");
    type_ = ALL_STRAIGHT_SEGMENTS;
    p.points.erase(p.points.begin()+2);
    // TODO: If all_bezier path, check each set of knot points
  }

  //std::cout<<"\nPath after Bezier: "<<utility_.toString(result)<<"\n";
  return result;
} // End bezier



/** Print Reflexxes input and output for the latest call */
void MobileBase::printReflexxesSpinInfo() const {
  std::cout<<"\n\nCalled reflexxes with input:";
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

} // End printReflexxesInfo




/** Execute one iteration of the Reflexxes control function */
const trajectory_msgs::JointTrajectoryPoint MobileBase::spinOnce() {

  // Calling the Reflexxes OTG algorithm
  resultValue_ = reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, reflexxesData_.outputParameters, reflexxesData_.flags);
  /*if(print_) {
    printReflexxesSpinInfo();
    std::cout<<"\n";
    std::cin.get();
  }*/
  

  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  trajectory_msgs::JointTrajectoryPoint point = buildTrajectoryPoint(reflexxesData_);


  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = *reflexxesData_.outputParameters->NewAccelerationVector;

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
    } // end if selection vector is true
    
    // Else if we're at orientation dof
    else if(i == 2) {
      
      // If straight-line paths, make theta be towards next knot point
      double theta = utility_.findAngleFromAToB( prevKP_.positions.at(0),
                                                 prevKP_.positions.at(1),
                                  data.inputParameters->TargetPositionVector->VecData[0],
                                  data.inputParameters->TargetPositionVector->VecData[1]);

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






// Service callback, the input is a path and the output a trajectory
bool MobileBase::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req, ramp_msgs::TrajectoryRequest::Response& res) {
  //std::cout<<"\nTrajectory Request Received: "<<utility_.toString(req)<<"\n";

  // If there's less than 3 points,
  // make it have straight segments
  if(req.path.points.size() < 3 && req.type != TRANSITION) {
    ROS_WARN("Path request size < 3");
    req.type = ALL_STRAIGHT_SEGMENTS;
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
    //if(print_ || bezierStart) 
      std::cout<<"\nPath before Bezier: "<<utility_.toString(path_)<<"\n";
    curves = bezier(path_, type_ == TRANSITION);
    //if(print_ || bezierStart)
      std::cout<<"\n*******************Path after Bezier: "<<utility_.toString(path_)<<"\n";
  } // end if curves
  else {
    std::cout<<"\nALL_STRAIGHT_SEGMENTS, no Bezier\n";
  }

  // Print curves
  //if(print_) {
    for(int c=0;c<curves.size();c++) {
      std::cout<<"\nCurve "<<c<<": ";
      for(int p=0;p<curves.at(c).points_.size();p++) {
        std::cout<<"\n"<<utility_.toString(curves.at(c).points_.at(p));
      }
    }
  //}
  

  // Push 0 onto knot point indices
  res.trajectory.i_knotPoints.push_back(0);

  
  uint8_t c=0;
  // Go through every knotpoint in the path
  // (or until timeCutoff has been reached)
  for (i_kp_ = 1; i_kp_<path_.points.size(); i_kp_++) {
    std::cout<<"\ni_kp: "<<(int)i_kp_<<"\n";
    resultValue_ = 0;
      
    // Push the initial state onto trajectory
    // And set previous knot point
    if(i_kp_ == 1) {
      res.trajectory.trajectory.points.push_back(buildTrajectoryPoint(reflexxesData_));
      prevKP_ = res.trajectory.trajectory.points.at(0);
    }


    /** Bezier */
    // If its a Bezier curve traj, and we're at a Bezier point
    // all points between first and last are bezier point
    if( path_.points.size() > 2 && 
        ((type_ == ALL_BEZIER && i_kp_ > 1 && i_kp_ < path_.points.size()-1) ||
        (type_ == PARTIAL_BEZIER && i_kp_ > 1 && i_kp_ < 3 && !bezierStart) ||
        (type_ == TRANSITION && i_kp_ == 1) ||
        (bezierStart && i_kp_ == 1) ))
    {
      std::cout<<"\nIn if\n";

      // Insert all points on the curves into the trajectory
      for(uint32_t p=1;p<curves.at(c).points_.size()-1;p++) {
        insertPoint(curves.at(c).points_.at(p), res);

        // If it's the first or last point on the curve, 
        // push the index to knot point vector
        if(p==curves.at(c).points_.size()-2) {
          res.trajectory.i_knotPoints.push_back(
                          res.trajectory.trajectory.points.size() - 1 );
        } // end if knot point
      } // end for

      // If the first curve, set the time
      /*if(c == 1) {
        res.t_nextKnotPoint = res.trajectory.trajectory.points.at(
                                 res.trajectory.trajectory.points.size()-1).time_from_start;
      }*/

      // Create a BezierInfo for the curve to return with the trajec
      ramp_msgs::BezierInfo bi;
      bi.segmentPoints  = curves.at(c).segmentPoints_;
      bi.controlPoints  = curves.at(c).controlPoints_;
      bi.ms_maxVA       = curves.at(c).ms_max_;
      bi.ms_initialVA   = curves.at(c).ms_init_;
      bi.numOfPoints    = curves.at(c).points_.size();
      bi.u_0            = req_.bezierInfo.u_0;
      bi.u_dot_0        = curves.at(c).u_dot_0_;
      bi.lambda         = curves.at(c).lambda_;
      res.trajectory.curves.push_back(bi);

      c++;
    } // end if bezier


    /** Straight Line Segment */
    // Else if straight-line segment
    else {
      std::cout<<"\nIn else\n";

        // Get rotation if needed
        double trajec_size = res.trajectory.trajectory.points.size();
        
        trajectory_msgs::JointTrajectoryPoint last = 
              res.trajectory.trajectory.points.at(trajec_size-1);

        trajectory_msgs::JointTrajectoryPoint next_knot = 
              utility_.getTrajectoryPoint(path_.points.at(i_kp_).motionState);
        
          std::cout<<"\nlast: "<<utility_.toString(last);
          std::cout<<"\nnext_knot: "<<utility_.toString(next_knot);
          std::cout<<"\nutility_.findAngleFromAToB(last, next_knot): "<<
                          utility_.findAngleFromAToB(last, next_knot);
          std::cout<<"\nutility_.findDistanceBetweenAngles(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot)): "<<
                        utility_.findDistanceBetweenAngles(last.positions.at(2), 
                                  utility_.findAngleFromAToB(last, next_knot))<<"\n";

        if(!finalStateReached()) {

          // If we need to rotate towards the next knot point
          if(fabs(utility_.findDistanceBetweenAngles(last.positions.at(2), 
                  utility_.findAngleFromAToB(last, next_knot))) > 0.173) 
          {
            std::vector<trajectory_msgs::JointTrajectoryPoint> rotate_points = 
              rotate(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot));
            
            //if(print_) {
              std::cout<<"\nrotate points size: "<<rotate_points.size();
            //}
            for(uint8_t p=0;p<rotate_points.size();p++) {
              if(print_) {
                std::cout<<"\nPoint "<<p<<": "<<utility_.toString(rotate_points.at(p));
              }
              res.trajectory.trajectory.points.push_back(rotate_points.at(p));
            } // end for


            setSelectionVector();
            resultValue_ = 0;
          } // end if rotate
        }


      setTarget(path_.points.at(i_kp_).motionState);
      if(print_) {
        std::cout<<"\nPrev KP: "<<utility_.toString(prevKP_)<<"\n";
        std::cout<<"\nTarget: "<<utility_.toString(path_.points.at(i_kp_).motionState)<<"\n";
      }


      // We go to the next knotpoint only once we reach this one
      while (!finalStateReached()) {

        trajectory_msgs::JointTrajectoryPoint p = spinOnce();

        // Compute the motion state at t+1 and save it in the trajectory
        res.trajectory.trajectory.points.push_back(p);
      } // end while

      // Once we reached the target, we set that the latest point is a knotpoint
      res.trajectory.i_knotPoints.push_back(res.trajectory.trajectory.points.size() - 1);
    } // end else straight-line segment

    // Set previous knot point
    prevKP_ = res.trajectory.trajectory.points.at(res.trajectory.trajectory.points.size() - 1);

    //std::cout<<"\nReached target at \n"<<utility_.toString(prevKP_)<<"\n";
  } // end for

  // Lastly, set newPath in case the path changed
  res.newPath = path_;

  //std::cout<<"\nReturning: "<<utility_.toString(res.trajectory)<<"\n";
  return true;
} // End trajectoryRequest callback



/** This performs a rotation using Reflexxes */
const std::vector<trajectory_msgs::JointTrajectoryPoint> MobileBase::rotate(const double start, const double goal) {
  //std::cout<<"\nIn rotate\n";
  std::vector<trajectory_msgs::JointTrajectoryPoint> result;

  // Set Selection Vector up for rotation
  setSelectionVectorRotation();

  // Set current velocity and acceleration values for x,y to 0
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = 0;

  // Set current values for orientation
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = start;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2] = 0;

  // Set target values
  reflexxesData_.inputParameters->TargetPositionVector->VecData[2] = goal;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[2] = 0;
  
  // Call Reflexxes until goal is reached
  resultValue_ = 0;
  while(!finalStateReached()) {
    trajectory_msgs::JointTrajectoryPoint p = spinOnce();
    result.push_back(p);
  }

  return result;
} // End rotate




// Returns true if the target has been reached
bool MobileBase::finalStateReached() {
  return (resultValue_ == ReflexxesAPI::RML_FINAL_STATE_REACHED);
  //return ((resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) || (timeFromStart_ >= timeCutoff_));
} // End finalStateReached



