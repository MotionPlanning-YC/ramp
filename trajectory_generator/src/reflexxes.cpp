#include "reflexxes.h"

// Execute one iteration of the Reflexxes control function
trajectory_msgs::JointTrajectoryPoint Reflexxes::spinOnce() {

  // Calling the Reflexxes OTG algorithm
  resultValue = rml->RMLPosition(*inputParameters, outputParameters, flags);

  // the input of the next iteration is the output of this one
  *inputParameters->CurrentPositionVector = *outputParameters->NewPositionVector;
  *inputParameters->CurrentVelocityVector = *outputParameters->NewVelocityVector;
  *inputParameters->CurrentAccelerationVector = *outputParameters->NewAccelerationVector;



  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  
  // Lets first put the new position to the trajectory point, we get it from the output of reflexxes
  trajectory_msgs::JointTrajectoryPoint point = buildTrajectoryPoint(*outputParameters);

  return point;
} // End spinOnce



/** This method will return a JointTrajectoryPoint given some output parameters from Reflexxes */
const trajectory_msgs::JointTrajectoryPoint Reflexxes::buildTrajectoryPoint(const RMLPositionOutputParameters outputParameters) {
  trajectory_msgs::JointTrajectoryPoint point;

  
  // Push on the p, v, and a vectors
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    point.positions.push_back(outputParameters.NewPositionVector->VecData[i]);
    point.velocities.push_back(outputParameters.NewVelocityVector->VecData[i]);
    point.accelerations.push_back(outputParameters.NewAccelerationVector->VecData[i]);
  }


  // The time_from_start is the time of the previous point plus the cycle period
  point.time_from_start = time_from_start;
  time_from_start += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint


/** This method will return a JointTrajectoryPoint given some input parameters from Reflexxes 
 *  This is used to generate the first point on a trajectory
 * */
const trajectory_msgs::JointTrajectoryPoint Reflexxes::buildTrajectoryPoint(const RMLPositionInputParameters inputParameters) {
  trajectory_msgs::JointTrajectoryPoint point;

  
  // Push on the p, v, and a vectors
  for(unsigned int i=0;i<NUMBER_OF_DOFS;i++) {
    point.positions.push_back(inputParameters.CurrentPositionVector->VecData[i]);
    point.velocities.push_back(inputParameters.CurrentVelocityVector->VecData[i]);
    point.accelerations.push_back(inputParameters.CurrentAccelerationVector->VecData[i]);
  }


  // The time_from_start is the time of the previous point plus the cycle period
  point.time_from_start = time_from_start;
  time_from_start += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint



//Set the target of the Reflexxes library
void Reflexxes::setTarget(float x, float y, float theta, float x_dot, float y_dot, float angular_velocity) {
  
  // Set the Target Position and Velocity vector
  inputParameters->TargetPositionVector->VecData[0] = x;
  inputParameters->TargetPositionVector->VecData[1] = y;

  // If a mobile base, the new target orientation 
  // is the orientation needed to reach the target
  inputParameters->TargetPositionVector->VecData[2] = theta;


  /*std::cout<<"\nTarget Position: ";
  std::cout<<"\n"<<inputParameters->TargetPositionVector->VecData[0];
  std::cout<<", "<<inputParameters->TargetPositionVector->VecData[1];
  std::cout<<", "<<inputParameters->TargetPositionVector->VecData[2];*/

  // Set the target velocity, which is set in the reference frame coordinates
  inputParameters->TargetVelocityVector->VecData[0] = x_dot;
  inputParameters->TargetVelocityVector->VecData[1] = y_dot;
  inputParameters->TargetVelocityVector->VecData[2] = angular_velocity;
      
  /*std::cout<<"\nTarget Velocity: ";
  std::cout<<"\n"<<inputParameters->TargetVelocityVector->VecData[0];
  std::cout<<", "<<inputParameters->TargetVelocityVector->VecData[1];
  std::cout<<", "<<inputParameters->TargetVelocityVector->VecData[2];*/
} // End setTarget







/** This method will modify a path by inserting knot points so that
 *  the path conforms to a rotate-drive-rotate style of motion */
const ramp_msgs::Path Reflexxes::modifyPath(const ramp_msgs::Path p) {
  //std::cout<<"\nIn modifyPath\n";
  ramp_msgs::Path result;

  result.points.push_back(p.points.at(0));
  for(int kp=0;kp<p.points.size()-1;kp++) {


    float trgt_theta = computeTargetOrientation( p.points.at(kp).motionState.positions.at(0),
                                        p.points.at(kp).motionState.positions.at(1),
                                        p.points.at(kp+1).motionState.positions.at(0),
                                        p.points.at(kp+1).motionState.positions.at(1));
    //std::cout<<"\nkp: "<<kp<<" trgt_theta: "<<trgt_theta;

    // The first part should be only rotation
    float diff = fabs(utility.findDistanceBetweenAngles(p.points.at(kp).motionState.positions.at(2), trgt_theta));
    if(diff > PI/18 && trgt_theta != 0) {
      ramp_msgs::KnotPoint temp;
      //std::cout<<"\np.points.at(kp).motionState.positions.at(2): "<<p.points.at(kp).motionState.positions.at(2);
      
      // Positions
      // Push on the same x,y values, but the target orientation
      temp.motionState.positions.push_back(p.points.at(kp).motionState.positions.at(0));
      temp.motionState.positions.push_back(p.points.at(kp).motionState.positions.at(1));
      temp.motionState.positions.push_back(trgt_theta);

      // Velocities
      temp.motionState.velocities.push_back(0);
      temp.motionState.velocities.push_back(0);
      temp.motionState.velocities.push_back(0);


      // Accelerations
      temp.motionState.accelerations.push_back(0);
      temp.motionState.accelerations.push_back(0);
      temp.motionState.accelerations.push_back(0);

      //result.points.insert(result.points.end(), temp);
      result.points.push_back(temp);
    } 

    // The second part should be only translation
    if( (p.points.at(kp).motionState.positions.at(0) != p.points.at(kp+1).motionState.positions.at(0)) ||
        (p.points.at(kp).motionState.positions.at(1) != p.points.at(kp+1).motionState.positions.at(1))) 
    {
      ramp_msgs::KnotPoint temp;

      // Positions
      // The x,y of next knot point, but relative orientation 
      temp.motionState.positions.push_back(p.points.at(kp+1).motionState.positions.at(0));
      temp.motionState.positions.push_back(p.points.at(kp+1).motionState.positions.at(1));
      temp.motionState.positions.push_back(trgt_theta);


      // Velocities
      temp.motionState.velocities.push_back(0);
      temp.motionState.velocities.push_back(0);
      temp.motionState.velocities.push_back(0);


      // Accelerations
      temp.motionState.accelerations.push_back(0);
      temp.motionState.accelerations.push_back(0);
      temp.motionState.accelerations.push_back(0);

      //result.points.insert(result.points.end(), temp);
      result.points.push_back(temp);
    }
  } // end for each knot points


  // Push on last knot point 
  result.points.push_back(p.points.at(p.points.size()-1));
  
  return result;
} // End modifyPath



// Service callback, the input is a path and the output a trajectory
bool Reflexxes::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res) {
  //std::cout<<"\nReceived request: "<<utility.toString(req)<<"\n";
  //std::cin.get();

  // Saves the path
  this->path = req.path;
  
  // Set the initial conditions of the reflexxes library
  setInitialConditions();

  // Modify the path if needed
  path = modifyPath(req.path);

  // Push 0 onto knot point indices
  res.trajectory.index_knot_points.push_back(0);

  // Go through every knotpoint in the path
  for (int i = 1; i<path.points.size() ; i++) {
    resultValue = 0;
    
    // Set the new knotpoint as the target
    setTarget(path.points[i].motionState.positions.at(0), 
              path.points[i].motionState.positions.at(1), 
              path.points[i].motionState.positions.at(2), 
              path.points[i].motionState.velocities.at(0), 
              path.points[i].motionState.velocities.at(1), 
              path.points[i].motionState.velocities.at(2));
    
    // Push the initial state onto trajectory
    //if(i==1)
      //res.trajectory.trajectory.points.push_back(buildTrajectoryPoint(*inputParameters));

    // We go to the next knotpoint only once we reach this one
    while (!isFinalStateReached()) {

      // Compute the motion state at t+1 and save it in the trajectory
      res.trajectory.trajectory.points.push_back(spinOnce());
    }

    // Once we reached the target, we set that the latest point is a knotpoint
    res.trajectory.index_knot_points.push_back(res.trajectory.trajectory.points.size() - 1);
  } // end for

  
  //std::cout<<"\nReturning: "<<utility.toString(res.trajectory)<<"\n";
  //std::cin.get();
  return true;
} // End trajectoryRequest callback






// Initialize variables just after receiving a service request
void Reflexxes::setInitialConditions() {
  // Set-up the input parameters
  // The first degree of freedom is x position
  // The second degree of freedom is y position
  // The third degree of freedom is the orientation
  // The target for the third dof is the orientation needed to be able to reach the goal position
  
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
float Reflexxes::computeTargetOrientation(float initial_x, float initial_y, float target_x, float target_y) {
  //We need to recalculate the target orientation
  // For that we first need to create a vector for the current position and the target position
  std::vector<float> current_position;
  current_position.push_back(initial_x);
  current_position.push_back(initial_y);

  std::vector<float> target_position;
  target_position.push_back(target_x);
  target_position.push_back(target_y);

  float angle = utility.findAngleFromAToB(current_position, target_position);

  return angle;
}


// Returns true if the target has been reached
bool Reflexxes::isFinalStateReached() {
  return (resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}




Reflexxes::Reflexxes() {

  trajectory = NULL;
  resultValue = 0;
  
  // Creating all relevant objects of the Type II Reflexxes Motion Library
  rml = new ReflexxesAPI( NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );
  inputParameters = new RMLPositionInputParameters( NUMBER_OF_DOFS );
  outputParameters = new RMLPositionOutputParameters( NUMBER_OF_DOFS );
  
  // Use time synchronization so the robot drives in a straight line towards goal 
  //flags.SynchronizationBehavior= RMLPositionFlags::NO_SYNCHRONIZATION;
  flags.SynchronizationBehavior= RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;
  
  // Set up the input parameters
  // Set the position, velocity and acceleration to be 0
  inputParameters->CurrentPositionVector->VecData[0] = 0.0;
  inputParameters->CurrentPositionVector->VecData[1] = 0.0;
  inputParameters->CurrentPositionVector->VecData[2] = 0.0;

  inputParameters->CurrentVelocityVector->VecData[0] = 0.0;
  inputParameters->CurrentVelocityVector->VecData[1] = 0.0;
  inputParameters->CurrentVelocityVector->VecData[2] = 0.0;

  inputParameters->CurrentAccelerationVector->VecData[0] = 0.0;
  inputParameters->CurrentAccelerationVector->VecData[1] = 0.0;
  inputParameters->CurrentAccelerationVector->VecData[2] = 0.0;

  // Here set up the max velocity, acceleration and jerk
  
  // Maximum velocity beeing 0.5m/s and 1 radian/s (around 60 degrees/s )
  inputParameters->MaxVelocityVector->VecData[0] = .5;
  inputParameters->MaxVelocityVector->VecData[1] = .5;
  inputParameters->MaxVelocityVector->VecData[2] =  1;

  // Maximum acceleration is 1m/s^2 and 2radian/s^2
  inputParameters->MaxAccelerationVector->VecData[0] = 1;
  inputParameters->MaxAccelerationVector->VecData[1] = 1;
  inputParameters->MaxAccelerationVector->VecData[2] = PI;

  // As the maximum jerk values are not known, this is just to try
  inputParameters->MaxJerkVector->VecData[0] = 1.0;
  inputParameters->MaxJerkVector->VecData[1] = 1.0;
  inputParameters->MaxJerkVector->VecData[2] = PI;

  // Set the Target Position and Velocity vector
  inputParameters->TargetPositionVector->VecData[0] = 0.0;
  inputParameters->TargetPositionVector->VecData[1] = 0.0;
  inputParameters->TargetPositionVector->VecData[2] = 0.0;

  inputParameters->TargetVelocityVector->VecData[0] = 0.0;
  inputParameters->TargetVelocityVector->VecData[1] = 0.0;
  inputParameters->TargetVelocityVector->VecData[2] = 0.0;

  // Select both DOF to be used
  inputParameters->SelectionVector->VecData[0] = true;
  inputParameters->SelectionVector->VecData[1] = true;
  inputParameters->SelectionVector->VecData[2] = true;

}

Reflexxes::~Reflexxes() {
  delete rml;
  delete inputParameters;
  delete outputParameters;
}
