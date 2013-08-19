#ifndef RAMP_TRAJECTORY_H
#define RAMP_TRAJECTORY_H

#include "ramp_msgs/Trajectory.h"
#include "utility.h"

class RampTrajectory {
  public:
    
    RampTrajectory();
    ~RampTrajectory() {}
    
    ramp_msgs::Trajectory trajec_;
    double fitness_;

    const std::string toString() const;
  private:
    Utility u;
};

#endif
