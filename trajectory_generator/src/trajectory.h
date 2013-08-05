#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ramp_msgs/Trajectory.h"
#include "geometry_msgs/Pose2D.h"
#include "utility.h"
#include "segment.h"

class Trajectory {
  public:

    Trajectory();
    Trajectory(const std::vector<geometry_msgs::Pose2D> kps);
    ~Trajectory();

    void buildSegments();
    const std::vector<MotionState> generate();
    const ramp_msgs::Trajectory buildTrajectoryMsg() const;
    
    //Data Members
    std::vector<geometry_msgs::Pose2D>  knot_points_;
    std::vector<Segment>                segments_;
    std::vector<MotionState>            points_;
    std::vector<float>                  t_; //Time per segment
       
    unsigned int resolutionRate_;  //The resolution rate is specified in Hz
    
  private:
    const MotionState getMotionState(const unsigned int ind_segment, const float t);
    const unsigned int k_dof_;
};

#endif
