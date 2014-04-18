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
    MotionState(const trajectory_msgs::JointTrajectoryPoint p);

    /***** Data Members *****/
    // Motion vectors
    std::vector<float> positions_;
    std::vector<float> velocities_;
    std::vector<float> accelerations_;
    std::vector<float> jerks_;

    // Time
    double time_;

    /***** Methods *****/
    double comparePosition(const MotionState& ms, bool base_theta) const;
    void transformBase(const tf::Transform t);
    
    const ramp_msgs::MotionState buildMotionStateMsg() const;
    const std::string toString() const;

  private:
    
    Utility utility;
    unsigned int mobile_base_k_;
    
    tf::Vector3 transformBasePosition(const tf::Transform t);
};

#endif
