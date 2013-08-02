#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "ros/ros.h"
#include "ramp_msgs/Trajectory.h"
#include "traj_gen_mobile_base/Segment.h"
#include "geometry_msgs/Pose2D.h"
#include "utility.h"
#include "segment.h"

class Trajectory {
  public:

    Trajectory();
    Trajectory(std::vector<geometry_msgs::Pose2D> kps);
    ~Trajectory();

    void buildSegments();

    
    std::vector<MotionState> generate();

    ramp_msgs::Trajectory buildTrajectoryMsg();
    
    //Data Members
    std::vector<geometry_msgs::Pose2D>  knot_points_;
    std::vector<Segment>                segments_;
    std::vector<MotionState>            points_;
    std::vector<float>                  t_; //Time per segment
       
    unsigned int resolutionRate_;  //The resolution rate is specified in Hz
    
  private:
    MotionState getMotionState(unsigned int ind_segment, float t);
    const unsigned int k_dof_;
};

#endif
