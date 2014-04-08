#ifndef KNOT_POINT_H
#define KNOT_POINT_H
#include "ramp_msgs/KnotPoint.h"
#include "motion_state.h"
#include "utility.h"

class KnotPoint {
  public:

    KnotPoint(); 
    KnotPoint(const MotionState mp);
    KnotPoint(const ramp_msgs::KnotPoint kp);
    ~KnotPoint() {}

    // Data Members
    MotionState motionState_;
    unsigned int stopTime_;

    // Methods
    const ramp_msgs::KnotPoint buildKnotPointMsg() const;
    const std::string toString() const;
    
  private:
};

#endif
