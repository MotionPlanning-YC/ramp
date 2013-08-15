
#include "trajectory.h"

Trajectory::Trajectory() : k_dof_(3) {}

/** This constructor pushes on all of the poses passed in */
/*Trajectory::Trajectory(const std::vector<geometry_msgs::Pose2D> kps) : k_dof_(3) {
  for(unsigned int i=0;i<kps.size();i++) {
    knot_points_.push_back(kps.at(i));
    
    //***x***t_.push_back(i*2.0);***x***
  }
}*/


Trajectory::Trajectory(const ramp_msgs::TrajectoryRequest::Request traj_req) : k_dof_(3) {
  
  //Push all of the path configurations onto knot_points
  for(unsigned int i=0;i<traj_req.path.configurations.size();i++) {
    geometry_msgs::Pose2D temp;
    temp.x      = traj_req.path.configurations.at(i).K.at(0);
    temp.y      = traj_req.path.configurations.at(i).K.at(1);
    temp.theta  = traj_req.path.configurations.at(i).K.at(2);
    knot_points_.push_back( temp );
  }
  
  for(unsigned int i=0;i<traj_req.v.size();i++) {
    v_.push_back(traj_req.v.at(i));
  }

  resolutionRate_ = traj_req.resolutionRate;
  
}


Trajectory::~Trajectory() {}

void Trajectory::buildSegments() {

  //Go through the knot points_,
  //Create a segment,
  //and assign the initial motions for each segment
  for(unsigned int i=0;i<knot_points_.size()-1;i++) {
    Segment temp;

    //Build the segment
    temp.build(knot_points_.at(i), knot_points_.at(i+1), v_.at(i), v_.at(i), i);
    
    //Push the segment onto the vector
    segments_.push_back(temp);
  }
}


/** This method returns a MotionState given a segment ID and a time */
const MotionState Trajectory::getMotionState(const unsigned int ind_segment, const float t) {
  
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
const std::vector<MotionState> Trajectory::generate() {

  //Build the segments
  buildSegments();

  //For each segment 
  for(unsigned int i=0;i<segments_.size();i++) { 

    //The time of the segment in seconds
    float segment_duration = segments_.at(i).T_;
    
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
const ramp_msgs::Trajectory Trajectory::buildTrajectoryMsg() const {
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
    p.time_from_start = ros::Duration(i* (1.0f/resolutionRate_));

    //Push onto the return value
    msg.trajectory.points.push_back(p);
  }

  msg.index_knot_points.push_back(0);
  float acc = segments_.at(0).T_;
  //Find the indices of the knot points
  for(unsigned int i=1;i<segments_.size();i++) {
    
    unsigned int index = acc * (float)resolutionRate_; 
    acc += segments_.at(i).T_;
    
    msg.index_knot_points.push_back(index);
  }

  //Get the last knot point
  //msg.index_knot_points.push_back(segments_.at(segments_.size()-1).end_t_ * resolutionRate_);
  return msg; 
}



const std::string Trajectory::toString() const {
  std::ostringstream result;

  result<<"\nKnot Points:";
  for(unsigned int i=0;i<knot_points_.size();i++) {
    result<<"\n"<<i<<": ("<<knot_points_.at(i).x<<", "<<knot_points_.at(i).y<<", "<<knot_points_.at(i).theta<<")";
  }

  result<<"\nSegments:";
  for(unsigned int i=0;i<segments_.size();i++) {
    result<<segments_.at(i).toString();
  }

  result<<"\nResolution Rate:"<<resolutionRate_;


  result<<"\nMotion States:";
  for(unsigned int i=0;i<points_.size();i++) {
    result<<"\nTime "<< (1.0f / resolutionRate_) * i;
    result<<points_.at(i).toString();
  }

  return result.str();
}
