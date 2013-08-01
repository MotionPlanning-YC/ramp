
#ifndef BASE_CONFIGURATION
#define BASE_CONFIGURATION

#include "geometry_msgs/Pose.h"
#include "tf/transform_datatypes.h"
#include "math.h"

class BaseConfiguration {
  public:

    BaseConfiguration();
    BaseConfiguration(double x, double y, double theta);
    BaseConfiguration(geometry_msgs::Pose pose);
    ~BaseConfiguration();

    static double getThetaFromQuat(geometry_msgs::Quaternion quat);

    double x;
    double y;
    double theta; //The rotation of the robot on the z-axis
};

#endif
