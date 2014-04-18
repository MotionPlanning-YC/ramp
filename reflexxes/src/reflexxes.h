#ifndef REFLEXXES_H
#define REFLEXXES_H


#include <Reflexxes/ReflexxesAPI.h>
#include <Reflexxes/RMLPositionFlags.h>
#include <Reflexxes/RMLPositionInputParameters.h>
#include <Reflexxes/RMLPositionOutputParameters.h>
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Path.h"
#include "ramp_msgs/TrajectoryRequest.h"
#include "tf/transform_datatypes.h"
#include "utility.h"

// defines
#define NUMBER_OF_DOFS 3
#define CYCLE_TIME_IN_SECONDS 0.2

class Reflexxes
{

  private:
    int resultValue;
    ReflexxesAPI *rml;
    RMLPositionInputParameters *inputParameters;          
    RMLPositionOutputParameters *outputParameters;
    RMLPositionFlags flags;
    Utility utility;
    ramp_msgs::Path path;
    ramp_msgs::Trajectory *trajectory;
    ros::Duration time_from_start;
    float current_orientation;
   
 // Compute the orientation needed to reach the target, given an initial position
    float computeOrientationNeededToGoal();

// Execute one iteration of the Reflexxes control function
    trajectory_msgs::JointTrajectoryPoint spinOnce();

//Set the target of the Reflexxes library
    void setTarget(float x, float y, float theta, float linear_velocity, float angular_velocity, bool mobile_base);

// Returns true if the target has been reached
    bool isFinalStateReached();

// Initialize variables just after receiving a service request
    void setInitialConditions();

// Compute the orientation needed to reach the target, given an initial position
    float computeTargetOrientation(float initial_x, float intial_y, float target_x, float target_y);
    
    const ramp_msgs::Path modifyPath(const ramp_msgs::Path p);


    const trajectory_msgs::JointTrajectoryPoint buildTrajectoryPoint(const RMLPositionOutputParameters inputParameters);

  public:
 
// Service callback, the input is a path and the output a trajectory
    bool trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req,ramp_msgs::TrajectoryRequest::Response& res);
    
    Reflexxes();

    ~Reflexxes();
};

#endif //REFLEXXES_H
