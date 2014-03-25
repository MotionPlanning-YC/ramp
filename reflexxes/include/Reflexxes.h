#ifndef REFLEXXES_H
#define REFLEXXES_H


#include "ReflexxesAPI.h"
#include "RMLPositionFlags.h"
#include "RMLPositionInputParameters.h"
#include "RMLPositionOutputParameters.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_datatypes.h"
#include "utility.h"

// defines
#define NUMBER_OF_DOFS 2
#define CYCLE_TIME_IN_SECONDS 0.001

class Reflexxes
{

  private:
    int resultValue;
    ReflexxesAPI *rml;
    RMLPositionInputParameters *inputParameters;          
    RMLPositionOutputParameters *outputParameters;
    RMLPositionFlags flags;
    Utility utility;
    std::vector<float> odometry;
    std::vector<float> target_position;

    float computeOrientationNeededToGoal();

  public:
    
    Reflexxes();

    ~Reflexxes();

    geometry_msgs::Twist spinOnce();

    bool isFinalStateReached();

    void updateStatus(const nav_msgs::Odometry& odometry);

    void setTargetState(std::vector<float> target_position, float linear_velocity, float angular_velocity);

};

#endif //REFLEXXES_H
