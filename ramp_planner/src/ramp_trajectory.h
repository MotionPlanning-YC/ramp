#ifndef RAMP_TRAJECTORY_H
#define RAMP_TRAJECTORY_H

#include "ramp_msgs/Trajectory.h"
#include "path.h"
#include "utility.h"

class RampTrajectory {
  public:
    
    RampTrajectory(const float resRate=1.f/10.f, unsigned int id=0);
    RampTrajectory(const ramp_msgs::Trajectory msg);
    ~RampTrajectory() {}
    
    unsigned int          id_;
    ramp_msgs::Trajectory msg_trajec_;
    float                 fitness_;
    bool                  feasible_;
    float                 time_until_collision_;
    Path                  path_;
    float                 resolutionRate_;
    int                   subPopulation_;

    const trajectory_msgs::JointTrajectoryPoint getPointAtTime(const float t)       const;
    const double                                getDirection()                      const;
    const Path                                  getPath()                           const;

    const RampTrajectory                        clone()                             const;
    const bool                                  equal(const RampTrajectory& other)  const;
    
    const std::string                           fitnessFeasibleToString()           const;
    const std::string                           toString()                          const;

  private:
    Utility utility_;
};



/** Create a struct to implement < comparison between two RampTrajectories */
struct RampTrajectoryCompare {

  // Returns true if rt1 < rt2, or less "fit" than rt2
  bool operator()(const RampTrajectory& rt1, const RampTrajectory& rt2) const {
    
    // First check for feasible vs. infeasible
    if(!rt1.feasible_ && rt2.feasible_) {
      return true;
    }
    else if(rt1.feasible_ && !rt2.feasible_) {
      return false;
    }

    // Return true if rt1 has a smaller fitness value than rt2
    return rt1.fitness_ < rt2.fitness_;
  } 
};

#endif
