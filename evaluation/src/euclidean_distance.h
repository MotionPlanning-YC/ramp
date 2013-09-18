#ifndef EUCLIDEAN_DISTANCE_H
#define EUCLIDEAN_DISTANCE_H
#include "utility.h"
#include "ramp_msgs/Trajectory.h"

//Modified: Add a structure
struct obstacle_struct // defines where an obstacle is
{
  float x1;
  float x2;
  float y1;
  float y2;
};

class EuclideanDistance {
  public:
    EuclideanDistance() {}


    const double perform(obstacle_struct obstacle);

    ramp_msgs::Trajectory trajectory_;

};

#endif
