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
    
    ramp_msgs::RampTrajectory msg_;
    Path                      path_;
    Path                      bezierPath_;
    ros::Duration             t_nextKnotPoint_;


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

#endif
