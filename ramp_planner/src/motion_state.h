#ifndef MOTION_STATE_H
#define MOTION_STATE_H
#include "ramp_msgs/MotionState.h"
#include "utility.h"

class MotionState {
  public:

    MotionState();
    MotionState(const ramp_msgs::MotionState ms);
    MotionState(const trajectory_msgs::JointTrajectoryPoint p);

    /***** Data Members *****/
    // Motion vectors
    std::vector<double> positions_;
    std::vector<double> velocities_;
    std::vector<double> accelerations_;
    std::vector<double> jerks_;

    // Time
    double time_;

    /***** Methods *****/
    const double  comparePosition(const MotionState& ms, 
                            const bool base_theta) const;
    void    transformBase(const tf::Transform t);

    const MotionState add(const MotionState m) const;
    const MotionState subtract(const MotionState m) const; 
    const MotionState multiply(const int num) const;
    const MotionState divide(const int num) const;
    const MotionState abs() const;
    const double      norm() const;
    const double      normPosition() const;
    const double      normVelocity() const;
    const double      normAcceleration() const;
    const double      normJerk() const;
    
    
    const   ramp_msgs::MotionState buildMotionStateMsg() const;
    const   std::string toString() const;

    

  private:
    
    Utility utility_;
    unsigned int mobile_base_k_;
    
    tf::Vector3 transformBasePosition(const tf::Transform t);
};

#endif
