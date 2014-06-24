#ifndef REFLEXXES_H
#define REFLEXXES_H


#include <Reflexxes/TypeII/ReflexxesAPI.h>
#include <Reflexxes/TypeII/RMLPositionFlags.h>
#include <Reflexxes/TypeII/RMLPositionInputParameters.h>
#include <Reflexxes/TypeII/RMLPositionOutputParameters.h>
#include "ramp_msgs/TrajectoryRequest.h"
#include "tf/transform_datatypes.h"
#include "utility.h"

// defines
#define NUMBER_OF_DOFS 3
#define CYCLE_TIME_IN_SECONDS 0.1


struct BezierConstants {
  double A, B, C, D;
};

class Reflexxes {

public:

  Reflexxes();
  ~Reflexxes();

  // Service callback, the input is a path and the output a trajectory
  bool trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res);

  
  // Make a point from 3 points and t
  const ramp_msgs::MotionState BezierPoint(const double u, const double u_dot, const double u_dot_dot, const ramp_msgs::MotionState p0, const ramp_msgs::MotionState p1, const ramp_msgs::MotionState p2);
  

  
  // Make a Bezier curve from 3 known points
  const std::vector<ramp_msgs::MotionState> BezierCurve(const ramp_msgs::MotionState p0, const ramp_msgs::MotionState p1, const ramp_msgs::MotionState p2);


  // Make a Bezier curve from a path
  const ramp_msgs::Path Bezier(const ramp_msgs::Path p);

  const std::vector<ramp_msgs::MotionState> getSegmentControlPoints(const double lambda, const ramp_msgs::MotionState x0, const ramp_msgs::MotionState x1, const ramp_msgs::MotionState x2) const;


  const double calculateR_min(const double t_min, const double A, const double B, const double C, const double D) const;

  const BezierConstants calculateConstants(const ramp_msgs::MotionState p0, const ramp_msgs::MotionState p1, const ramp_msgs::MotionState p2) const; 

  
private:

  
  // Flag for result
  int resultValue;

  // *********** Reflexxes variables ************ //
  ReflexxesAPI *rml;
  RMLPositionInputParameters *inputParameters;          
  RMLPositionOutputParameters *outputParameters;
  RMLPositionFlags flags;
  // ******************************************** //
  
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
  trajectory_msgs::JointTrajectoryPoint spinOnce();

  // Set the target of the Reflexxes library
  void setTarget(const ramp_msgs::MotionState ms);

  // Returns true if the target has been reached
  bool isFinalStateReached();

  // Initialize variables just after receiving a service request
  void setInitialConditions();

  // Compute the orientation needed to reach the target, given an initial position
  double computeTargetOrientation(double initial_x, double intial_y, double target_x, double target_y);
  
  // Methods to build a trajectory point
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionOutputParameters outputParameters);
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionInputParameters inputParameters);

  // Set the selection vector for a path
  void setSelectionVector(const bool rot);
  

  void initialize(const ramp_msgs::TrajectoryRequest::Request req);

};

#endif 
