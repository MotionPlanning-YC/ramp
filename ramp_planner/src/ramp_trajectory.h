#ifndef RAMP_TRAJECTORY_H
#define RAMP_TRAJECTORY_H

#include "ramp_msgs/Trajectory.h"
#include "path.h"
#include "utility.h"

class RampTrajectory {
  public:
    
    RampTrajectory();
    ~RampTrajectory() {}
    
    ramp_msgs::Trajectory msg_trajec_;
    double fitness_;
    bool feasible_;

    const Path getPath() const;
    const std::string fitnessFeasibleToString() const;
    const std::string toString() const;
  private:
    Utility u;
};

#endif
