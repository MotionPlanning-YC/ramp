#ifndef MOTION_STATE_H
#define MOTION_STATE_H
#include "ramp_msgs/MotionState.h"
#include "configuration.h"
#include "utility.h"

class MotionState {
  public:

    MotionState();
    MotionState(const ramp_msgs::MotionState ms);
    MotionState(const Configuration c);

    // Motion vectors
    std::vector<float> positions_;
    std::vector<float> velocities_;
    std::vector<float> accelerations_;
    std::vector<float> jerks_;

    // Time
    double time_;

    const ramp_msgs::MotionState buildMotionStateMsg() const;
    const std::string toString() const;

  private:
};

#endif
