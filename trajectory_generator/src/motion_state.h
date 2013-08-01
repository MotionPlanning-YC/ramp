#ifndef MOTION_STATE_H
#define MOTION_STATE_H

#include "utility.h"
#include "geometry_msgs/Pose2D.h"

class MotionState {
  public:

    //Note that p is also the configuration
    std::vector<float> p_;
    std::vector<float> v_;
    std::vector<float> a_;

  private:
};

#endif
