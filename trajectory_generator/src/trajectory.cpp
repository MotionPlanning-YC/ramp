
#include "trajectory.h"

Trajectory::Trajectory() : k_dof_(3) {}

/** This constructor pushes on all of the poses passed in */
Trajectory::Trajectory(std::vector<geometry_msgs::Pose2D> kps) : k_dof_(3) {
  for(unsigned int i=0;i<kps.size();i++) {
    knot_points_.push_back(kps.at(i));
  }
}


Trajectory::~Trajectory() {}

void Trajectory::buildSegments() {

  //Go through the knot points_,
  //Create a segment,
  //and assign the initial motions for each segment
  for(unsigned int i=0;i<knot_points_.size()-1;i++) {
    Segment temp;

    float t_s = (i==0) ? 0 : segments_.at(i-1).end_t_;
    float t_e = t_s + t_.at(i);
    temp.build(knot_points_.at(i), knot_points_.at(i+1), t_s, t_e, i);
    
    //Push the segment onto the vector
    segments_.push_back(temp);
  }
}


/** This method returns a MotionState given a segment ID and a time */
MotionState Trajectory::getMotionState(unsigned int ind_segment, float t) {
  //std::cout<<"\nIn getMotionState\n";
  
  MotionState result;

  Segment segment = segments_.at(ind_segment);

  //Position
  for(unsigned int i=0;i<k_dof_;i++) {
    result.p_.push_back(segment.a0_.at(i) + segment.a1_.at(i)*t);
  }
 
  //Velocity
  for(unsigned int i=0;i<k_dof_;i++) {
    result.v_.push_back(segment.a1_.at(i));
  }
  
  //Acceleration
  for(unsigned int i=0;i<k_dof_;i++) {
    result.a_.push_back(0);
  }

  return result;  
}



/** This function generates the set of motion states that represent the trajectory */
std::vector<MotionState> Trajectory::generate() {
  //std::cout<<"\nIn generate\n";

  //For each segment 
  for(unsigned int i=0;i<segments_.size();i++) { 
    
    //The time of the segment in seconds
    float segment_duration = segments_.at(i).end_t_ - segments_.at(i).start_t_;
    
    
    //The # of times the trajectory is calculated for the segment, determined by the resolution rate
    int total_t = segment_duration / (1.0 / resolutionRate_);
    
    //If we are at the last segment, increase the total_t by 1
    //so that in the following loop, the very last state of motion is tacked on
    if (i == segments_.size()-1) total_t++;

    //For each clock cycle, find the state of motion
    for(unsigned int clock=0;clock<total_t;clock++) {
      
      //Get the time 
      float t = clock * (1.0/resolutionRate_);
      
      //Get the motion state
      points_.push_back(getMotionState(i, t));
    }
  }

  return points_;
}




/** This function returns a JointTrajectory msg based on the trajectory
 *  points_ must not be empty */
ramp_msgs::Trajectory Trajectory::buildTrajectoryMsg() {
  ramp_msgs::Trajectory msg;

  //Push on all of the Motion States
  for(unsigned int i=0;i<points_.size();i++) {
    MotionState m = points_.at(i);
    trajectory_msgs::JointTrajectoryPoint p;
    
    //Positions
    for(unsigned int j=0;j<3;j++) {
      p.positions.push_back(m.p_.at(j));
    }
    
    //Velocities
    for(unsigned int j=0;j<3;j++) {
      p.velocities.push_back(m.v_.at(j));
    }

    //Accelerations
    for(unsigned int j=0;j<3;j++) {
      p.accelerations.push_back(m.a_.at(j));
    }

    //Set the duration for the point
    p.time_from_start = ros::Duration(i* (1/resolutionRate_));

    //Push onto the return value
    msg.trajectory.points.push_back(p);
  }

  //Find the indices of the knot points
  for(unsigned int i=0;i<segments_.size();i++) {
    unsigned int index = segments_.at(i).start_t_ * resolutionRate_;
    std::cout<<"\nindex:"<<index<<"\n";
    msg.index_knot_points.push_back(index);
  }

  //Get the last knot point
  msg.index_knot_points.push_back(segments_.at(segments_.size()-1).end_t_ * resolutionRate_);
  return msg; 
}
