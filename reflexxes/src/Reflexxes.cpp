#include "Reflexxes.h"

// Execute one iteration of the Reflexxes control function
trajectory_msgs::JointTrajectoryPoint Reflexxes::spinOnce()
{
  // Calling the Reflexxes OTG algorithm
  resultValue = rml->RMLPosition(*inputParameters, outputParameters, flags);

  // the input of the next iteration is the output of this one
  *inputParameters->CurrentPositionVector = *outputParameters->NewPositionVector;
  *inputParameters->CurrentVelocityVector = *outputParameters->NewVelocityVector;
  *inputParameters->CurrentAccelerationVector = *outputParameters->NewAccelerationVector;

  // Change the target orientation, as the target is the orientation needed to reach the goal
  inputParameters->TargetPositionVector->VecData[2] = computeTargetOrientation(inputParameters->CurrentPositionVector->VecData[0],inputParameters->CurrentPositionVector->VecData[1],inputParameters->TargetPositionVector->VecData[0],inputParameters->TargetPositionVector->VecData[1]);
   

  // Starting here, we build the JointTrajectoryPoint object, that will be used to build the trajectory
  
  // Lets first put the new position to the trajectory point, we get it from the output of reflexxes
  trajectory_msgs::JointTrajectoryPoint point;
  for (int i=0; i<3; i++)
  {
    point.positions.push_back(outputParameters->NewPositionVector->VecData[i]);
  }

  // WE need the angle of the velocity vector and the difference between that angle and the robot orientation
  float velocity_angle = computeTargetOrientation(0,0,outputParameters->NewVelocityVector->VecData[0],outputParameters->NewVelocityVector->VecData[1]);
  float difference_angle = velocity_angle - inputParameters->CurrentPositionVector->VecData[2];

  // Calculate the linear velocity, being the norm of the vector (Vx, Vy)
    float linear_velocity = sqrt(pow(outputParameters->NewVelocityVector->VecData[0], 2) + pow(outputParameters->NewVelocityVector->VecData[1], 2) );
  
  // Now create the velocity vector for the trajectory
  // We need to transform the velocity in x, y and theta in the reference frame to a linear and angular velocity
  point.velocities.push_back(linear_velocity * cos(difference_angle));
  point.velocities.push_back(0);
  point.velocities.push_back(outputParameters->NewVelocityVector->VecData[2] + sin(difference_angle) * linear_velocity);

  //Same with the acceleration
  float linear_acceleration = sqrt(pow(outputParameters->NewAccelerationVector->VecData[0], 2) + pow(outputParameters->NewAccelerationVector->VecData[1], 2) );
  point.accelerations.push_back(linear_acceleration);
  point.accelerations.push_back(0);
  point.accelerations.push_back(outputParameters->NewAccelerationVector->VecData[2]);

  // The time_from_start is the time of the previous point plus the cycle period
  point.time_from_start = time_from_start;
  time_from_start += ros::Duration(CYCLE_TIME_IN_SECONDS);

  return point;

}

//Set the target of the Reflexxes library
void Reflexxes::setTarget(float x, float y, float linear_velocity, float angular_velocity)
{
  // Set the Target Position and Velocity vector
  inputParameters->TargetPositionVector->VecData[0] = x;
  inputParameters->TargetPositionVector->VecData[1] = y;

  // The new target orientation is the orientation needed to reach the target
    inputParameters->TargetPositionVector->VecData[2] = computeTargetOrientation(inputParameters->CurrentPositionVector->VecData[0],inputParameters->CurrentPositionVector->VecData[1],x,y);

  // Set the target velocity, which is set in the reference frame coordinates
  inputParameters->TargetVelocityVector->VecData[0] = linear_velocity * cos(inputParameters->TargetPositionVector->VecData[2]) ;
  inputParameters->TargetVelocityVector->VecData[1] = linear_velocity * sin(inputParameters->TargetPositionVector->VecData[2] );
  inputParameters->TargetVelocityVector->VecData[2] = angular_velocity;
}



// Service callback, the input is a path and the output a trajectory
bool Reflexxes::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res)
{

  for (int i=0; i<req.v_start.size(); i++)
    ROS_ERROR("vstart(%d): %f", i, req.v_start.at(i));
  ROS_ERROR("%s", utility.toString(req.path).c_str());

  // Saves the path
  this->path = req.path;
    res.trajectory.index_knot_points.push_back(0);
  
  //set the initial conditions of the reflexxes library
  setInitialConditions(req.v_start);
    
  time_from_start = ros::Duration(0);

  // Go through every knotpoint in the path
  for (int i = 1; i<path.points.size() ; i++)
  {
    resultValue = 0;
    // Set the new knotpoint as the target
    if (i< (path.points.size()-1))
      setTarget(path.points[i].configuration.K.at(0), path.points[i].configuration.K.at(1), 0.0, 0);
    else
      setTarget(path.points[i].configuration.K.at(0), path.points[i].configuration.K.at(1),0,0);
    // We go to the next knotpoint only once we reach this one
    while (!isFinalStateReached())
    {
      // Compute the motion state at t+1 and save it in the trajectory
      res.trajectory.trajectory.points.push_back(spinOnce());
    }
    // Once we reached the target, we don't forgot to mention that this point in the trajectory is a knotpoint
    res.trajectory.index_knot_points.push_back(res.trajectory.trajectory.points.size() - 1);

  }
  return true;
}

// Initialize variables just after receiving a service request
void Reflexxes::setInitialConditions(std::vector<float> velocity)
{
  // Set-up the input parameters
  // The firt degree of freedom is x position
  // The second degree of freedom is y position
  // The third degree of freedom is the orientation
  // The target for the third dof is the orientation needed to be able to reach the goal position
  
  // Initialise the time
  time_from_start = ros::Duration(0);
  

  // Get the current orientation of the robot
  float orientation = path.points[0].configuration.K.at(2);

  // set the position and orientation of the robot as Reflexxes input
  inputParameters->CurrentPositionVector->VecData[0] = path.points[0].configuration.K.at(0);
  inputParameters->CurrentPositionVector->VecData[1] = path.points[0].configuration.K.at(1);

  current_orientation = orientation;
  inputParameters->CurrentPositionVector->VecData[2] = orientation;

  // Set the current velocities, in the reference frame, as Reflexxes input
  // This is the latest velocity value got from the path
  inputParameters->CurrentVelocityVector->VecData[0] = velocity.at(0) * cos(orientation);
  inputParameters->CurrentVelocityVector->VecData[1] = velocity.at(0) * sin(orientation);
  inputParameters->CurrentVelocityVector->VecData[2] = velocity.at(2);

  inputParameters->CurrentAccelerationVector->VecData[0] = 0.0;
  inputParameters->CurrentAccelerationVector->VecData[1] = 0.0;
  inputParameters->CurrentAccelerationVector->VecData[2] = 0.0;
}

// Compute the orientation needed to reach the target, given an initial position
float Reflexxes::computeTargetOrientation(float initial_x, float initial_y, float target_x, float target_y)
{
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
bool Reflexxes::isFinalStateReached()
{
  return (resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}


Reflexxes::Reflexxes()
{
  trajectory = NULL;
  resultValue = 0;
  // Creating all relevant objects of the Type II Reflexxes Motion Library
  rml = new ReflexxesAPI( NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );
  inputParameters = new RMLPositionInputParameters( NUMBER_OF_DOFS );
  outputParameters = new RMLPositionOutputParameters( NUMBER_OF_DOFS );
  flags.SynchronizationBehavior= RMLPositionFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
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
  inputParameters->MaxVelocityVector->VecData[0] = .3;
  inputParameters->MaxVelocityVector->VecData[1] = .3;
  inputParameters->MaxVelocityVector->VecData[2] = .6;

  // Maximum acceleration is 1m/s^2 and 2radian/s^2
  inputParameters->MaxAccelerationVector->VecData[0] = 1;
  inputParameters->MaxAccelerationVector->VecData[1] = 1;
  inputParameters->MaxAccelerationVector->VecData[2] = 3;

  // As the maximum jerk values are not known, this is just to try
  inputParameters->MaxJerkVector->VecData[0] = 1.0;
  inputParameters->MaxJerkVector->VecData[1] = 1.0;
  inputParameters->MaxJerkVector->VecData[2] = 3.0;

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

Reflexxes::~Reflexxes()
{
  delete rml;
  delete inputParameters;
  delete outputParameters;
}
