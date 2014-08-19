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
  const std::vector<BezierCurve> bezier(ramp_msgs::Path& p, const bool only_curve);
  
  const double findVelocity(const uint8_t i, const double s) const;

  TrajectoryType type_;
  bool print_;
private:

  void initReflexxes();

  // Store the time it started
  ros::Time t_started_;
  
  // Flag for result
  int resultValue_;



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

  // Previous knot point - used for straight line trajectories
  trajectory_msgs::JointTrajectoryPoint prevKP_;

  // Utility
  Utility utility_;



  /***** Methods *****/
  // Initialize everything
  void init(const ramp_msgs::TrajectoryRequest::Request req);

  // Set the target of the Reflexxes library
  void setTarget(const ramp_msgs::MotionState ms);

  // Set the selection vector for a path
  void setSelectionVector();

  // Initialize variables just after receiving a service request
  void setInitialMotion();
  
  // Approximate initial state of a Bezier curve
  const ramp_msgs::MotionState getInitialState(const std::vector<ramp_msgs::MotionState> segment_points) const;


  // Insert a point to the back of the trajectory
  // and set Reflexxes to reflect the new state
  void insertPoint(const ramp_msgs::MotionState ms, ramp_msgs::TrajectoryRequest::Response& res);
  void insertPoint(const trajectory_msgs::JointTrajectoryPoint jp, ramp_msgs::TrajectoryRequest::Response& res);

  

  // Execute one iteration of the Reflexxes control function
  const trajectory_msgs::JointTrajectoryPoint spinOnce();

  // Returns true if the target has been reached
  bool finalStateReached();


  // Use Reflexxes to generate a rotation trajectory
  const std::vector<trajectory_msgs::JointTrajectoryPoint> rotate(const double start, const double goal);

  // Set the Selection Vector for rotation
  void setSelectionVectorRotation();

  // Get a valid lambda value for a curve over segment_points
  const double getControlPointLambda(const std::vector<ramp_msgs::MotionState> segment_points) const;

  // Check if a lambda value is valid for segment_points
  const bool lambdaOkay(const std::vector<ramp_msgs::MotionState> segment_points, const double lambda) const;

  // Build a JointTrajectoryPoint from Reflexxes data
  const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const ReflexxesData Data_);

  // Print Current and Next vectors
  void printReflexxesSpinInfo() const;

};

#endif 
