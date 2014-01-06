
#include "trajectory.h"

Trajectory::Trajectory() : k_dof_(3) {}


Trajectory::Trajectory(const ramp_msgs::TrajectoryRequest::Request trajec_req) : k_dof_(3) {
  
  // Push all of the path configurations onto knot_points
  for(unsigned int i=0;i<trajec_req.path.configurations.size();i++) {
    geometry_msgs::Pose2D temp;
    temp.x      = trajec_req.path.configurations.at(i).K.at(0);
    temp.y      = trajec_req.path.configurations.at(i).K.at(1);
    temp.theta  = trajec_req.path.configurations.at(i).K.at(2);
    knot_points_.push_back( temp );
  }
  
  //  Get all of the velocities
  for(unsigned int i=0;i<trajec_req.v_start.size();i++) {
    v_start_.push_back(trajec_req.v_start.at(i));
    v_end_.push_back(trajec_req.v_end.at(i));
  }

  resolutionRate_ = trajec_req.resolutionRate;
}


Trajectory::~Trajectory() {}

void Trajectory::buildSegments() {
  // std::cout<<"\nIn Trajectory::buildSegments\n";

  // Go through the knot points_,
  // Create a segment,
  // and assign the initial motions for each segment
  for(unsigned int i=0;i<knot_points_.size()-1;i++) {
    Segment temp;

    // Build the segment
    temp.build(knot_points_.at(i), knot_points_.at(i+1), v_start_.at(i), v_end_.at(i), i);
    
    // Push the segment onto the vector
    segments_.push_back(temp);
  }

}


/** 
  * This method returns a MotionState given a segment ID and a time 
  * The method is passed a vector of dof's to compute 
  */
const MotionState Trajectory::getMotionState(const unsigned int ind_segment, const float t) {
  //std::cout<<"\nin Trajectory::getMotionState\n";
  
  MotionState result;

  Segment segment = segments_.at(ind_segment);
  
  for(unsigned int i=0;i<k_dof_;i++) {

    // If k is x or y and it is still time to be pre-rotate (rotate towards segment goal)
    // Push on the initial x,y value
    if(i < 2 && t <= segment.T_rotate_pre_ && segment.T_rotate_pre_ > 0) {
      result.p_.push_back(segment.a0_.at(i));
      result.v_.push_back(0);
    }

    // If k is x or y and it is time to drive towards the goal
    // Push on the next x,y value
    // Need to translate t by the time for pre-rotating
    else if(i < 2 && t <= (segment.T_loc_ + segment.T_rotate_pre_)) {
      result.p_.push_back(segment.a0_.at(i) + (segment.a1_.at(i) * (t - segment.T_rotate_pre_)));
      result.v_.push_back(fabs(segment.a1_.at(i)));
    }

    // If k is x or y and it is past time to pre-rotate or drive 
    // Push on the last value of x,y
    else if(i < 2) {
      result.p_.push_back(segment.a0_.at(i) + segment.a1_.at(i)*segment.T_loc_);
      result.v_.push_back(0);
    }

    // If k is theta and it is time to pre-rotate
    // Push on the value of angle 
    else if(i == 2 && t <= segment.T_rotate_pre_ && segment.T_rotate_pre_ > 0) {
      result.p_.push_back(segment.a0_.at(i) + segment.a1_.at(i) * t);
      result.v_.push_back(segment.a1_.at(i));
    }
    
    // If k is theta and it is time to post-rotate
    // Push on the next value of theta
    // Need to translate t by the time for pre-rotating and driving straight
    else if(i == 2 && t > (segment.T_rotate_pre_ + segment.T_loc_)) {
      result.p_.push_back(segment.angle_pre + segment.a1_.at(i+1) * (t - segment.T_loc_ - segment.T_rotate_pre_));
      result.v_.push_back(segment.a1_.at(i+1));
    }

    // If k is theta and it is not time to post-rotate
    // Push on the initial value of theta
    else {
      result.p_.push_back(segment.angle_pre);
      result.v_.push_back(0);
    }
    
    // 0 acceleration
    result.a_.push_back(0);
  }

  return result;  
} //End generate



/** This function generates the set of motion states that represent the trajectory */
const std::vector<MotionState> Trajectory::generate() {
  //std::cout<<"\nIn Trajectory::generate()\n";

  // Build the segments
  buildSegments();

  // For each segment 
  for(unsigned int i=0;i<segments_.size();i++) { 
    //std::cout<<"\nSegment "<<i<<"\n";

    Segment segment = segments_.at(i);
    /*std::cout<<"\nSegment "<<i<<":";
    std::cout<<"\nangle_pre: "<<segment.angle_pre<<" (radians)";
    std::cout<<", "<<segment.angle_pre * 180 / M_PI<<" (degrees)";
    std::cout<<"\nT_rotate_pre_: "<<segment.T_rotate_pre_;
    std::cout<<"\nT_loc: "<<segment.T_loc_;
    std::cout<<"\nT_rotate_post_: "<<segment.T_rotate_post_;*/

    // The time of the segment in seconds
    unsigned int segment_duration = segments_.at(i).T_min_;
    //std::cout<<"\nsegment_duration:"<<segment_duration;
    
    // The # of times the trajectory is calculated for the segment, determined by the resolution rate
    unsigned int total_t = segment_duration / (1.0 / resolutionRate_);
    //std::cout<<"\ntotal_t:"<<total_t;
    
    // If we are at the last segment, increase the total_t by 1
    // so that in the following loop, the very last state of motion is tacked on
    if (i == segments_.size()-1) total_t++;

    // For each clock cycle, find the state of motion
    for(unsigned int clock=0;clock<total_t;clock++) {
      
      // Get the time 
      float t = clock * (1.0/resolutionRate_);
      
      // Get the motion state
      points_.push_back(getMotionState(i, t));

    } // end for each clock
  } // end for each segment

  return points_;
}




/** This function returns a JointTrajectory msg based on the trajectory
 *  points_ must not be empty */
const ramp_msgs::Trajectory Trajectory::buildTrajectoryMsg() const {
  ramp_msgs::Trajectory msg;

  // Push on all of the Motion States
  for(unsigned int i=0;i<points_.size();i++) {
    MotionState m = points_.at(i);
    trajectory_msgs::JointTrajectoryPoint p;
    
    // Positions
    for(unsigned int j=0;j<3;j++) {
      p.positions.push_back(m.p_.at(j));
    }
    
    // Velocities
    for(unsigned int j=0;j<3;j++) {
      p.velocities.push_back(m.v_.at(j));
    }

    // Accelerations
    for(unsigned int j=0;j<3;j++) {
      p.accelerations.push_back(m.a_.at(j));
    }

    // Set the duration for the point
    p.time_from_start = ros::Duration(i * (1.0f/resolutionRate_));

    // Push onto the return value
    msg.trajectory.points.push_back(p);
  }

  
  // Find the indices of the knot points
  float acc=0;
  for(unsigned int i=0;i<=segments_.size()-1;i++) {
    
    unsigned int index = acc * (float)resolutionRate_; 
    
    acc += segments_.at(i).T_min_;
    
    msg.index_knot_points.push_back(index);
  }

  // Push the last knot point index on.
  msg.index_knot_points.push_back(points_.size()-1);

  return msg; 
} //End buildTrajectoryMsg



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
