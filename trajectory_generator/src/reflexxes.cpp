#include "reflexxes.h"


/** Constructor */
Reflexxes::Reflexxes() {
  
  // Creating all relevant objects of the Type II Reflexxes Motion Library
  rml = new ReflexxesAPI( NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );
  inputParameters = new RMLPositionInputParameters( NUMBER_OF_DOFS );
  outputParameters = new RMLPositionOutputParameters( NUMBER_OF_DOFS );
  
  // Use time synchronization so the robot drives in a straight line towards goal 
  flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;

  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  inputParameters->MaxVelocityVector->VecData[0] = 0.33;
  inputParameters->MaxVelocityVector->VecData[1] = 0.33;
  inputParameters->MaxVelocityVector->VecData[2] = PI/4;

  // Maximum acceleration
  inputParameters->MaxAccelerationVector->VecData[0] = 0.33;
  inputParameters->MaxAccelerationVector->VecData[1] = 0.33;
  inputParameters->MaxAccelerationVector->VecData[2] = PI/4;

  // As the maximum jerk values are not known, this is just to try
  inputParameters->MaxJerkVector->VecData[0] = 1;
  inputParameters->MaxJerkVector->VecData[1] = 1;
  inputParameters->MaxJerkVector->VecData[2] = PI/3;

  // Result
  resultValue = 0;

  // Set the time to cutoff generating points
  timeCutoff_ = ros::Duration(3.5);
} 

/** Destructor */
Reflexxes::~Reflexxes() {
  delete rml;
  delete inputParameters;
  delete outputParameters;
}





/** This method sets the new target of Reflexxes */
void Reflexxes::setTarget(const ramp_msgs::MotionState ms) {
  
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(i);
    inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(i);
  }  
} // End setTarget







/** This method sets the SelectionVector based on the path p */
void Reflexxes::setSelectionVector(const bool rot) {

  // If it is not a rotational trajectory, set x,y true 
  if(!rot) 
  {
    inputParameters->SelectionVector->VecData[0] = true;
    inputParameters->SelectionVector->VecData[1] = true;
    inputParameters->SelectionVector->VecData[2] = false;
  }
  else {
    inputParameters->SelectionVector->VecData[0] = false;
    inputParameters->SelectionVector->VecData[1] = false;
    inputParameters->SelectionVector->VecData[2] = true;
  }
}





/**
 * Initialize variables just after receiving a service request
 * Set-up the input parameters
 * The first degree of freedom is x position
 * The second degree of freedom is y position
 **/
void Reflexxes::setInitialConditions() {
  
  // Initialise the time
  timeFromStart_ = ros::Duration(0);
  

  // Set the position and orientation of the robot as Reflexxes input
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(i);
  }

  // Set the current velocities, in the reference frame, as Reflexxes input
  // This is the latest velocity value got from the path
  if(path_.points.at(0).motionState.velocities.size() > 0) {
    for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
      inputParameters->CurrentVelocityVector->VecData[i] = path_.points.at(0).motionState.velocities.at(i);
    }
  }
  else {//log some error
  }

  // Set the current accelerations, in the reference frame, as Reflexxes input
  // This is the latest acceleration value got from the path
  if(path_.points.at(0).motionState.accelerations.size() > 0) {
    for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
      inputParameters->CurrentAccelerationVector->VecData[i] = path_.points.at(0).motionState.accelerations.at(i);
    }
  }
  else {//log some error
  }
} // End setInitialConditions





// Compute the orientation needed to reach the target, given an initial position
double Reflexxes::computeTargetOrientation(double initial_x, double initial_y, double target_x, double target_y) {
  //We need to recalculate the target orientation
  // For that we first need to create a vector for the current position and the target position
  std::vector<double> current_position;
  current_position.push_back(initial_x);
  current_position.push_back(initial_y);

  std::vector<double> target_position;
  target_position.push_back(target_x);
  target_position.push_back(target_y);

  double angle = utility_.findAngleFromAToB(current_position, target_position);

  return angle;
}





void Reflexxes::initialize(const ramp_msgs::TrajectoryRequest::Request req) {

  // Store the path
  path_ = req.path;
  
  // Set the initial conditions of the reflexxes library
  setInitialConditions();

  // Set SelectionVector
  setSelectionVector(req.rotational);

}





const double Reflexxes::calculateR_min(const double t_min, const double A, const double B, const double C, const double D) const {

  double numerator_term_one   = ((A*A) + (B*B)) * (t_min*t_min);
  double numerator_term_two   = 2 * ((A*C)+(B*D)) * t_min;
  double numerator_term_three = (C*C) + (D*D);
  double numerator            = pow(numerator_term_one + numerator_term_two + numerator_term_three, 3); 

  double denominator          = pow((B*C) - (A*D), 2);
 
  double R_min                = sqrt( numerator / denominator );

  return R_min;
} 


const BezierConstants Reflexxes::calculateConstants(const ramp_msgs::MotionState p0, const ramp_msgs::MotionState p1, const ramp_msgs::MotionState p2) const {
  BezierConstants result;

  // A = 2(X0-2X1+X2)
  result.A = 2 * (p0.positions.at(0) - (2*p1.positions.at(0)) + p2.positions.at(0));

  // B = 2(Y0-2Y1+Y2)
  result.B = 2 * (p0.positions.at(1) - (2*p1.positions.at(1)) + p2.positions.at(1));

  // C = 2(X1-X0)
  result.C = 2 * (p1.positions.at(0) - p0.positions.at(0));

  // D = 2(Y1-Y0)
  result.D = 2 * (p1.positions.at(1) - p0.positions.at(1));



  return result;
}




/** Generates a point on a 2nd-degree Bezier curve */
const ramp_msgs::MotionState Reflexxes::BezierPoint(const double u, const double u_dot, const double u_dot_dot, const ramp_msgs::MotionState p0, const ramp_msgs::MotionState p1, const ramp_msgs::MotionState p2) {

  ramp_msgs::MotionState result;

  /* Positions */
  double u2 = u * u;
  double mu = 1-u;
  double mu2 = mu * mu;
  double x = mu2*p0.positions.at(0) + 2*mu*u*p1.positions.at(0) + u2*p2.positions.at(0);
  double y = mu2*p0.positions.at(1) + 2*mu*u*p1.positions.at(1) + u2*p2.positions.at(1);

  result.positions.push_back(x);
  result.positions.push_back(y);


  BezierConstants bc = calculateConstants(p0, p1, p2);



  return result;
}


/** This method generates motion states to move over a 2nd-degree Bezier curve */
const std::vector<ramp_msgs::MotionState> Reflexxes::BezierCurve(const ramp_msgs::MotionState p0, const ramp_msgs::MotionState p1, const ramp_msgs::MotionState p2) {
  std::vector<ramp_msgs::MotionState> result;

  double u_dot = CYCLE_TIME_IN_SECONDS;
  double u_dot_dot=0;
  for(double u=0;u<=1;u+=CYCLE_TIME_IN_SECONDS) {
    result.push_back(BezierPoint(u, u_dot, u_dot_dot, p0, p1, p2));
  }


  // A = 2(X0-2X1+X2)
  double A = 2 * (p0.positions.at(0) - (2*p1.positions.at(0)) + p2.positions.at(0));

  // B = 2(Y0-2Y1+Y2)
  double B = 2 * (p0.positions.at(1) - (2*p1.positions.at(1)) + p2.positions.at(1));

  // C = 2(X1-X0)
  double C = 2 * (p1.positions.at(0) - p0.positions.at(0));

  // D = 2(Y1-Y0)
  double D = 2 * (p1.positions.at(1) - p0.positions.at(1));

  // Time where minimum radius occurs
  double t_min = -((A*C) + (B*D)) / ((A*A) + (B*B));

  // Minimum radius
  double R_min = calculateR_min(t_min, A, B, C, D);
  
  std::cout<<"\nA: "<<A<<" B: "<<B<<" C: "<<C<<" D: "<<D;
  std::cout<<"\nt_min: "<<t_min;

  std::cout<<"\nR_min: "<<R_min;

  std::cout<<"\n 0.33 / R_min: "<<0.33/R_min;
  std::cout<<"\n"<<((0.33/R_min)<PI/4);
  


  return result;
}






/* Lambda is the time on the segments to start the curve */
const std::vector<ramp_msgs::MotionState> Reflexxes::getSegmentControlPoints(const double lambda, const ramp_msgs::MotionState x0, const ramp_msgs::MotionState x1, const ramp_msgs::MotionState x2) const {
  std::vector<ramp_msgs::MotionState> result;

  ramp_msgs::MotionState x1_prev;
  ramp_msgs::MotionState x1_next;

  /** Positions */
  x1_prev.positions.push_back( (1-lambda)*x0.positions.at(0) + lambda*x1.positions.at(0) );
  x1_prev.positions.push_back( (1-lambda)*x0.positions.at(1) + lambda*x1.positions.at(1) );
  x1_prev.positions.push_back(0);

  x1_next.positions.push_back( (1-lambda)*x1.positions.at(0) + lambda*x2.positions.at(0) );
  x1_next.positions.push_back( (1-lambda)*x1.positions.at(1) + lambda*x2.positions.at(1) );
  x1_next.positions.push_back(0);


  /** Velocities */

  BezierConstants bc = calculateConstants(x0, x1, x2);
  
  // Predict which DOF will have greater velocity
  double x_dot, y_dot;
  double x_diff = fabs(x1.positions.at(0) - x0.positions.at(0));
  double y_diff = fabs(x1.positions.at(1) - x0.positions.at(1));

  if(x_diff > y_diff) {
    if( (x_diff < 0 && y_diff > 0) || (x_diff > 0 && y_diff < 0) ) {
      x_dot =  -inputParameters->MaxVelocityVector->VecData[1];
      y_dot =   inputParameters->MaxVelocityVector->VecData[1];
    } 
    else {
      x_dot =   inputParameters->MaxVelocityVector->VecData[0];
      y_dot =  -inputParameters->MaxVelocityVector->VecData[0];
    }
  }

  x1_prev.velocities.push_back(x_dot);
  x1_prev.velocities.push_back(y_dot);
  x1_prev.velocities.push_back(0);

  x1_prev.accelerations.push_back(0);
  x1_prev.accelerations.push_back(0);
  x1_prev.accelerations.push_back(0);
  

  result.push_back(x1_prev);
  result.push_back(x1_next);

  return result;
}






const ramp_msgs::Path Reflexxes::Bezier(const ramp_msgs::Path p) {

  ramp_msgs::Path result;


  double lambda = 0.5;
  for(uint8_t i=1;i<p.points.size()-1;i++) {

    std::vector<ramp_msgs::MotionState> control_points = getSegmentControlPoints( lambda, p.points.at(i-1).motionState,
                                                                                          p.points.at(i).motionState,
                                                                                          p.points.at(i+1).motionState );

    std::vector<ramp_msgs::MotionState> curve = BezierCurve(control_points.at(0), p.points.at(i).motionState, control_points.at(1));

    // Add curve as knot points on path
    // Remove point i, start from control point 0, end at control point 2
  }




  return result;
} 






/** Execute one iteration of the Reflexxes control function */
trajectory_msgs::JointTrajectoryPoint Reflexxes::spinOnce() {

  // Calling the Reflexxes OTG algorithm
  resultValue = rml->RMLPosition(*inputParameters, outputParameters, flags);


  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  trajectory_msgs::JointTrajectoryPoint point = buildTrajectoryPoint(*outputParameters);


  // The input of the next iteration is the output of this one
  *inputParameters->CurrentPositionVector = *outputParameters->NewPositionVector;
  *inputParameters->CurrentVelocityVector = *outputParameters->NewVelocityVector;
  *inputParameters->CurrentAccelerationVector = *outputParameters->NewAccelerationVector;

  return point;
} // End spinOnce








/** This method will return a JointTrajectoryPoint given some output parameters from Reflexxes */
const trajectory_msgs::JointTrajectoryPoint Reflexxes::buildTrajectoryPoint(const RMLPositionOutputParameters my_outputParameters) {
  trajectory_msgs::JointTrajectoryPoint point;

  
  // Push on the p, v, and a vectors
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    if(inputParameters->SelectionVector->VecData[i]) {
      point.positions.push_back(my_outputParameters.NewPositionVector->VecData[i]);
      point.velocities.push_back(my_outputParameters.NewVelocityVector->VecData[i]);
      point.accelerations.push_back(my_outputParameters.NewAccelerationVector->VecData[i]);
    }
    else if(i == 2) {
      
      double p = computeTargetOrientation(inputParameters->CurrentPositionVector->VecData[0],
                                          inputParameters->CurrentPositionVector->VecData[1],
                                          path_.points.at(i_kp_).motionState.positions.at(0),
                                          path_.points.at(i_kp_).motionState.positions.at(1));
      point.positions.push_back(p);
      point.velocities.push_back(0);
      point.accelerations.push_back(0);
    }
    else {
      point.positions.push_back(inputParameters->CurrentPositionVector->VecData[i]);
      point.velocities.push_back(inputParameters->CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(inputParameters->CurrentAccelerationVector->VecData[i]);
    }
  }


  // The timeFromStart_ is the time of the previous point plus the cycle period
  point.time_from_start = timeFromStart_;
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint


/** 
 *  This method will return a JointTrajectoryPoint given some input parameters from Reflexxes 
 *  This is used to generate the first point on a trajectory
 **/
const trajectory_msgs::JointTrajectoryPoint Reflexxes::buildTrajectoryPoint(const RMLPositionInputParameters my_inputParameters) {
  trajectory_msgs::JointTrajectoryPoint point;

  
  // Push on the p, v, and a vectors
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    if(inputParameters->SelectionVector->VecData[i]) {
      point.positions.push_back(my_inputParameters.CurrentPositionVector->VecData[i]);
      point.velocities.push_back(my_inputParameters.CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(my_inputParameters.CurrentAccelerationVector->VecData[i]);
    }

    else {
      point.positions.push_back(inputParameters->CurrentPositionVector->VecData[i]);
      point.velocities.push_back(inputParameters->CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(inputParameters->CurrentAccelerationVector->VecData[i]);
    }
  }


  // The timeFromStart_ is the time of the previous point plus the cycle period
  point.time_from_start = timeFromStart_;
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint






// Service callback, the input is a path and the output a trajectory
bool Reflexxes::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req, ramp_msgs::TrajectoryRequest::Response& res) {
  //std::cout<<"\nReceived request: "<<utility_.toString(req)<<"\n";
  //std::cin.get();
  
  // Initialize Reflexxes with request
  initialize(req);
  std::cout<<"\nDone initializing\n";
  

  // Push 0 onto knot point indices
  res.trajectory.index_knot_points.push_back(0);



  double lambda = 0.5;

  // Go through every knotpoint in the path
  // (or until timeCutoff has been reached)
  for (i_kp_ = 1; i_kp_<path_.points.size()-1; i_kp_++) {
    std::cout<<"\ni_kp: "<<(int)i_kp_;
    resultValue = 0;
    
    std::vector<ramp_msgs::MotionState> control_points = 
                                getSegmentControlPoints(lambda, path_.points.at(i_kp_-1).motionState,
                                                                path_.points.at(i_kp_).motionState, 
                                                                path_.points.at(i_kp_+1).motionState);
    std::cout<<"\ncontrol_points[0]: "<<utility_.toString(control_points.at(0))<<"\n";
    std::cout<<"\ncontrol_points[1]: "<<utility_.toString(control_points.at(1))<<"\n";


    // Set target to next knot point
    setTarget(control_points.at(0));
    std::cout<<"\nTarget set\n";
    
    // Push the initial state onto trajectory
    if(i_kp_ == 1)
      res.trajectory.trajectory.points.push_back(buildTrajectoryPoint(*inputParameters));

    // We go to the next knotpoint only once we reach this one
    while (!isFinalStateReached()) {

      // Compute the motion state at t+1 and save it in the trajectory
      res.trajectory.trajectory.points.push_back(spinOnce());
    }

    // Once we reached the target, we set that the latest point is a knotpoint
    res.trajectory.index_knot_points.push_back(res.trajectory.trajectory.points.size() - 1);
  } // end for

  
  // Set the trajectory's resolution rate
  res.trajectory.resolution_rate = CYCLE_TIME_IN_SECONDS;

  //std::cout<<"\nReturning: "<<utility_.toString(res.trajectory)<<"\n";
  //std::cin.get();
  return true;
} // End trajectoryRequest callback



// Returns true if the target has been reached
bool Reflexxes::isFinalStateReached() {
  return (resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}



