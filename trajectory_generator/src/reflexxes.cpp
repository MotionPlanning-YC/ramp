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
  inputParameters->MaxVelocityVector->VecData[0] = .33;
  inputParameters->MaxVelocityVector->VecData[1] = .33;
  inputParameters->MaxVelocityVector->VecData[2] = PI/3;

  // Maximum acceleration
  inputParameters->MaxAccelerationVector->VecData[0] = 0.33;
  inputParameters->MaxAccelerationVector->VecData[1] = 0.33;
  inputParameters->MaxAccelerationVector->VecData[2] = PI/3;

  // As the maximum jerk values are not known, this is just to try
  inputParameters->MaxJerkVector->VecData[0] = 1;
  inputParameters->MaxJerkVector->VecData[1] = 1;
  inputParameters->MaxJerkVector->VecData[2] = PI/3;

  // Result
  resultValue = 0;
} 

/** Destructor */
Reflexxes::~Reflexxes() {
  delete rml;
  delete inputParameters;
  delete outputParameters;
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
                                          my_outputParameters.NewPositionVector->VecData[0],
                                          my_outputParameters.NewPositionVector->VecData[1]);
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


  // The time_from_start is the time of the previous point plus the cycle period
  point.time_from_start = time_from_start;
  time_from_start += ros::Duration(CYCLE_TIME_IN_SECONDS);


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


  // The time_from_start is the time of the previous point plus the cycle period
  point.time_from_start = time_from_start;
  time_from_start += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint








/** This method sets the new target of Reflexxes */
void Reflexxes::setTarget(const ramp_msgs::KnotPoint kp) {
  
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    inputParameters->TargetPositionVector->VecData[i] = kp.motionState.positions.at(i);
    inputParameters->TargetVelocityVector->VecData[i] = kp.motionState.velocities.at(i);
  }  
} // End setTarget







/** This method sets the SelectionVector based on the path p */
void Reflexxes::setSelectionVector(const ramp_msgs::Path p) {

  // If the x,y positions are different, this should 
  // be a translational trajectory, otherwise rotational
  if(utility.getEuclideanDist(p.points.at(0), 
              p.points.at(p.points.size()-1)) > 0.0001) 
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






// Service callback, the input is a path and the output a trajectory
bool Reflexxes::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req, ramp_msgs::TrajectoryRequest::Response& res) {
  //std::cout<<"\nReceived request: "<<utility.toString(req)<<"\n";
  //std::cin.get();


  // Store the path
  path = req.path;
  
  // Set the initial conditions of the reflexxes library
  setInitialConditions();

  // Set SelectionVector
  setSelectionVector(path);

  // Push 0 onto knot point indices
  res.trajectory.index_knot_points.push_back(0);



  // Go through every knotpoint in the path
  for (int i = 1; i<path.points.size(); i++) {
    resultValue = 0;
    
    // Set target to next knot point
    setTarget(path.points.at(i));
    
    // Push the initial state onto trajectory
    if(i==1)
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

  //std::cout<<"\nReturning: "<<utility.toString(res.trajectory)<<"\n";
  //std::cin.get();
  return true;
} // End trajectoryRequest callback






/**
 * Initialize variables just after receiving a service request
 * Set-up the input parameters
 * The first degree of freedom is x position
 * The second degree of freedom is y position
 **/
void Reflexxes::setInitialConditions() {
  
  // Initialise the time
  time_from_start = ros::Duration(0);
  

  // Set the position and orientation of the robot as Reflexxes input
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    inputParameters->CurrentPositionVector->VecData[i] = path.points[0].motionState.positions.at(i);
  }

  // Set the current velocities, in the reference frame, as Reflexxes input
  // This is the latest velocity value got from the path
  if(path.points.at(0).motionState.velocities.size() > 0) {
    for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
      inputParameters->CurrentVelocityVector->VecData[i] = path.points.at(0).motionState.velocities.at(i);
    }
  }
  else {//log some error
  }

  // Set the current accelerations, in the reference frame, as Reflexxes input
  // This is the latest acceleration value got from the path
  if(path.points.at(0).motionState.accelerations.size() > 0) {
    for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
      inputParameters->CurrentAccelerationVector->VecData[i] = path.points.at(0).motionState.accelerations.at(i);
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

  double angle = utility.findAngleFromAToB(current_position, target_position);

  return angle;
}


// Returns true if the target has been reached
bool Reflexxes::isFinalStateReached() {
  return (resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}



