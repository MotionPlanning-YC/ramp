#ifndef EUCLIDEAN_DISTANCE_H
#define EUCLIDEAN_DISTANCE_H
#include "utility.h"
#include "ramp_msgs/Trajectory.h"


class EuclideanDistance {
  public:
    EuclideanDistance() {}


    const double perform();

    ramp_msgs::Trajectory trajectory_;
    ramp_msgs::MotionState goal_;

    Utility utility_;
};

#endif
