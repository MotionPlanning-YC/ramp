#ifndef MOBILE_BASE_H
#define MOBILE_BASE_H

#include "ramp_msgs/TrajectoryRequest.h"
#include "tf/transform_datatypes.h"
#include "reflexxes_data.h"
#include "bezier_curve.h"
#include "utility.h"




#define CYCLE_TIME_IN_SECONDS 0.1




class MobileBase {

public:

  MobileBase();
  ~MobileBase();

  // Service callback, the input is a path and the output a trajectory
  bool trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res);


  // Get Bezier curves over the path
  const ramp_msgs::Path Bezier(const ramp_msgs::Path p);
  
private:

  
  // Flag for result
  int resultValue;


  // Reflexxes variables 
  ReflexxesData reflexxesData_;
  
  // The path of the trajectory
  ramp_msgs::Path path_;

  // How much time has passed
  ros::Duration timeFromStart_;

  // Index of knot point we are trying to reach
  uint8_t i_kp_;

  // Time from start to stop planning trajectory points
  ros::Duration timeCutoff_;

  // Utility
  Utility utility_;



  /***** Methods *****/

  // Execute one iteration of the Reflexxes control function
  const trajectory_msgs::JointTrajectoryPoint spinOnce();

  // Set the target of the Reflexxes library
  void setTarget(const ramp_msgs::MotionState ms);

  // Set the selection vector for a path
  void setSelectionVector(const bool rot);

  // Returns true if the target has been reached
  bool finalStateReached();

  // Initialize variables just after receiving a service request
  void setInitialConditions();

  // Compute the orientation needed to reach the target, given an initial position
  double computeTargetOrientation(double initial_x, double intial_y, double target_x, double target_y);
  
  // Methods to build a trajectory point
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionInputParameters input, const RMLPositionOutputParameters output);
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionInputParameters inputParameters);
  

  void init(const ramp_msgs::TrajectoryRequest::Request req);

};

#endif 