#include "Reflexxes.h"

// Execute the Reflexxes control function
geometry_msgs::Twist Reflexxes::spinOnce()
{
  // Calling the Reflexxes OTG algorithm 
  resultValue =   rml->RMLPosition(*inputParameters, outputParameters, flags);                                              
  
  // Update the current position to be the output of the reflexxes library. 
  // This is useful particularly if the odometry won't be updated before the next execution of the function
  // The acceleration is also useful as we don't get that information with the odometry
  *inputParameters->CurrentPositionVector = *outputParameters->NewPositionVector;
  *inputParameters->CurrentVelocityVector = *outputParameters->NewVelocityVector;
  *inputParameters->CurrentAccelerationVector = *outputParameters->NewAccelerationVector;

  // Create the twist message and return it
  geometry_msgs::Twist twist;
  twist.linear.x = outputParameters->NewPositionVector->VecData[0];
  twist.angular.z = outputParameters->NewPositionVector->VecData[1];
 
  return twist;

}

void Reflexxes::setTargetState(std::vector<float> target_position, float linear_velocity, float angular_velocity)
{
  // Set-up the input parameters
  // The firt degree of freedom is set up as follow:
  // The position is the distance between the robot and the goal
  // The target is 0, meaning we want the distance to be reduced
  // The second degree of freedome is as follow: 
  // The position is the orientation of the robot
  // The target is the orientation needed to be able to reach the goal position
  
  // Create a 2d vector for the position, the reason is that our position is in 2d. 
  std::vector<float> current_position;
  current_position.push_back(odometry[0]);
  current_position.push_back(odometry[1]);

  // Set the Target Position and Velocity vector
  inputParameters->TargetPositionVector->VecData[0] = 0.0;
  inputParameters->TargetPositionVector->VecData[1] = utility.findAngleFromAToB(current_position, target_position);

  inputParameters->TargetVelocityVector->VecData[0] = linear_velocity;
  inputParameters->TargetVelocityVector->VecData[1] = angular_velocity;

  // We need to recalculate the distance between the robot and the goal
  inputParameters->CurrentPositionVector->VecData[0] = utility.getEuclideanDist(current_position, target_position);

  // Save the target position for future uses
  this->target_position = target_position;
}


// Update the current Position and Velocity of the robot
void Reflexxes::updateStatus(const nav_msgs::Odometry& odometry)
{
  // Set-up the input parameters
  // The firt degree of freedom is x position
  // The second degree of freedom is y position
  // The third degree of freedom is the orientation
  // The target for the third dof is the orientation needed to be able to reach the goal position
  
  //get orientation of the robot
  float orientation = tf::getYaw(odometry.pose.pose.orientation);

  // position and orientation of the robot
  inputParameters->CurrentPositionVector->VecData[0] = odometry.pose.pose.position.x;
  inputParameters->CurrentPositionVector->VecData[1] = odometry.pose.pose.position.y;
  inputParameters->CurrentPositionVector->VecData[2] = orientation;

  //We need to recalculate the target orientation 
  // For that we first need to crate a vector for the current position
  std::vector<float> current_position;
  current_position.push_back(odometry.pose.pose.position.x);
  current_position.push_back(odometry.pose.pose.position.y);

  inputParameters->TargetPositionVector->VecData[2] = utility.findAngleFromAToB(current_position, target_position);

  // Set the current velocities
  inputParameters->CurrentVelocityVector->VecData[0] = odometry.twist.twist.linear.x * cos(orientation);
  inputParameters->CurrentVelocityVector->VecData[1] = odometry.twist.twist.linear.x * sin(orientation);
  inputParameters->CurrentVelocityVector->VecData[2] = odometry.twist.twist.angular.z;

  // We want to save the odometry for future purposes
  this->odometry = current_position;
}

// Returns true if the target has been reached
bool Reflexxes::isFinalStateReached()
{
  return (resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED); 
}


Reflexxes::Reflexxes()
{
  // Creating all relevant objects of the Type II Reflexxes Motion Library      
  rml = new ReflexxesAPI( NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );                                                           
  inputParameters = new RMLPositionInputParameters( NUMBER_OF_DOFS );                                                                     
  outputParameters = new RMLPositionOutputParameters( NUMBER_OF_DOFS );

  // Set up the input parameters
  // Set the position, velocity and acceleration to be 0
  inputParameters->CurrentPositionVector->VecData[0] = 0.0;
  inputParameters->CurrentPositionVector->VecData[1] = 0.0;
  inputParameters->CurrentPositionVector->VecData[2] = 0.0;

  inputParameters->CurrentVelocityVector->VecData[0] = 0.0;
  inputParameters->CurrentVelocityVector->VecData[1] = 0.0;
  inputParameters->CUrrentVelocityVector->VecData[2] = 0.0;

  inputParameters->CurrentAccelerationVector->VecData[0] = 0.0;
  inputParameters->CurrentAccelerationVector->VecData[1] = 0.0;
  inputParameters->CurrentAccelerationVector->VecData[2] = 0.0;

  // Here set up the max velocity, acceleration and jerk
  
  // Maximum velocity beeing 0.5m/s and 1 radian/s (around 60 degrees/s ) 
  inputParameters->MaxVelocityVector->VecData[0] = .5;
  inputParameters->MaxVelocityVector->VecData[1] = .5;
  inputParameters->MaxVelocityVector->VecData[2] = 1; 

  // Maximum acceleration is 1m/s^2 and 2radian/s^2
  inputParameters->MaxAccelerationVector->VecData[0] = 1;
  inputParameters->MaxAccelerationVector->VecData[1] = 1;
  inputParameters->MaxAccelerationVector->VecData[2] = 2; 

  // As the maximum jerk values are not known, this is just to try
  inputParameters->MaxJerkVector->VecData[0] = 1.0;
  inputParameters->MaxJerkVector->VecData[1] = 1.0;
  inputParameters->MaxJerkVector->VecData[2] = 2.0;

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
