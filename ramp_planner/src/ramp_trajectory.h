#ifndef RAMP_TRAJECTORY_H
#define RAMP_TRAJECTORY_H

#include "ramp_msgs/RampTrajectory.h"
#include "path.h"
#include "utility.h"

class RampTrajectory {
  public:
    
    explicit RampTrajectory(const float resRate=1.f/10.f, unsigned int id=0);
    RampTrajectory(const ramp_msgs::RampTrajectory msg);
    ~RampTrajectory() {}
    
    unsigned int          id_;
    ramp_msgs::RampTrajectory msg_;
    Path                  path_;
    Path                  bezierPath_;
    int                   subPopulation_;
    float                 timeUntilCollision_;


    const bool           equal(const RampTrajectory& other)  const;
    const Path           getPath()                           const;

    const double         getDirection() const;
    
    const RampTrajectory getStraightSegment(uint8_t i) const;

    const RampTrajectory clone() const;
    
    const std::string    fitnessFeasibleToString()           const;
    const std::string    toString()                          const;

    const trajectory_msgs::JointTrajectoryPoint getPointAtTime(const float t)       const;
  private:
    Utility utility_;
};



/** Create a struct to implement < comparison between two RampTrajectories */
struct RampTrajectoryCompare {

  // Returns true if rt1 < rt2, or less "fit" than rt2
  bool operator()(const RampTrajectory& rt1, const RampTrajectory& rt2) const {
    
    // First check for feasible vs. infeasible
    if(!rt1.msg_.feasible && rt2.msg_.feasible) {
      return true;
    }
    else if(rt1.msg_.feasible && !rt2.msg_.feasible) {
      return false;
    }

    // Return true if rt1 has a smaller fitness value than rt2
    return rt1.msg_.fitness < rt2.msg_.fitness;
  } 
};

#endif
