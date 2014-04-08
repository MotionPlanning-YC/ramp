#ifndef MOTION_STATE_H
#define MOTION_STATE_H
#include "utility.h"

class MotionState {
  public:

    // Motion vectors
    std::vector<float> positions_;
    std::vector<float> velocities_;
    std::vector<float> accelerations_;
    std::vector<float> jerks_;

    // Time
    double time_;

    const std::string toString() const;

  private:
};

#endif
