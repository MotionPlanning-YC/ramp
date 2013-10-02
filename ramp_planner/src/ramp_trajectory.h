#ifndef RAMP_TRAJECTORY_H
#define RAMP_TRAJECTORY_H

#include "ramp_msgs/Trajectory.h"
#include "path.h"
#include "utility.h"

class RampTrajectory {
  public:
    
    RampTrajectory(unsigned int id=0);
    RampTrajectory(const ramp_msgs::Trajectory msg);
    ~RampTrajectory() {}
    
    ramp_msgs::Trajectory msg_trajec_;
    double fitness_;
    bool feasible_;
    unsigned int id_;

    const bool equal(const RampTrajectory& other) const;
    const Path getPath() const;
    const std::string fitnessFeasibleToString() const;
    const std::string toString() const;

  private:
    Utility u;
};



/** Create a struct to implement < comparison between two RampTrajectories */
struct RampTrajectoryCompare {
  //Returns true if rt1 < rt2, aka less fit than rt2
  bool operator()(const RampTrajectory& rt1, const RampTrajectory& rt2) const {
    
    //First check for feasible vs. infeasible
    if(!rt1.feasible_ && rt2.feasible_) {
      return true;
    }
    else if(rt1.feasible_ && !rt2.feasible_) {
      return false;
    }

    //Return true if rt1 has a smaller fitness value than rt2
    return rt1.fitness_ < rt2.fitness_;
  } 
};

#endif
