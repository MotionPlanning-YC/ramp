#ifndef REFLEXXES_H
#define REFLEXXES_H


#include <Reflexxes/TypeII/ReflexxesAPI.h>
#include <Reflexxes/TypeII/RMLPositionFlags.h>
#include <Reflexxes/TypeII/RMLPositionInputParameters.h>
#include <Reflexxes/TypeII/RMLPositionOutputParameters.h>
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Path.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "tf/transform_datatypes.h"
#include "utility.h"

// defines
#define NUMBER_OF_DOFS 3
#define CYCLE_TIME_IN_SECONDS 0.1

class Reflexxes {

public:

  Reflexxes();
  ~Reflexxes();

  // Service callback, the input is a path and the output a trajectory
  bool trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res);
  
private:
  int resultValue;
  ReflexxesAPI *rml;
  RMLPositionInputParameters *inputParameters;          
  RMLPositionOutputParameters *outputParameters;
  RMLPositionFlags flags;
  Utility utility;
  ramp_msgs::Path path;
  ros::Duration time_from_start;
 
  // Compute the orientation needed to reach the target, given an initial position
  float computeOrientationNeededToGoal();

  // Execute one iteration of the Reflexxes control function
  trajectory_msgs::JointTrajectoryPoint spinOnce();

  // Set the target of the Reflexxes library
  void setTarget(const ramp_msgs::KnotPoint kp);

  // Returns true if the target has been reached
  bool isFinalStateReached();

  // Initialize variables just after receiving a service request
  void setInitialConditions();

  // Compute the orientation needed to reach the target, given an initial position
  float computeTargetOrientation(float initial_x, float intial_y, float target_x, float target_y);
  
  // Methods to build a trajectory point
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionOutputParameters outputParameters);
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionInputParameters inputParameters);

  // Set the selection vector for a path
  void setSelectionVector(const ramp_msgs::Path p);

};

#endif //REFLEXXES_H
