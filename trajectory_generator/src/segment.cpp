#include "segment.h"


Segment::Segment() : k_dof_(3), plan_post(0), T_rotate_pre_(0), T_rotate_post_(0), T_loc_(0), T_min_(0) {
  max_v_.push_back(0.33f);
  max_v_.push_back(0.33f);

  // Theta velocity is higher
  max_v_.push_back(PI/4.0f);
}

Segment::Segment(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float v_start, const float v_end, const unsigned int ind) : k_dof_(3), plan_post(0), T_rotate_pre_(0), T_rotate_post_(0), T_loc_(0), T_min_(0)
{
  max_v_.push_back(0.33f);
  max_v_.push_back(0.33f);
  max_v_.push_back(PI/4.0f);
  build(kp_start, kp_end, v_start, v_end, ind);
}

Segment::~Segment() {}


/** This function assigns the Segment's members and calls buildWork() */
void Segment::build(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float v_start, const float v_end, const unsigned int ind) {
  // std::cout<<"\nIn Segment::build\n";

  start_.p_.push_back(kp_start.x);
  start_.p_.push_back(kp_start.y);
  start_.p_.push_back(kp_start.theta);

  end_.p_.push_back(kp_end.x);
  end_.p_.push_back(kp_end.y);
  end_.p_.push_back(kp_end.theta); 
  
  index_    = ind;
  v_start_  = v_start;
  v_end_    = v_end;

  buildWork();
} // End build



/** This function initializes the Segment's start and end states of motion, and the segment's a0 and a1 coefficients for interpolation */
void Segment::buildWork() {
  // std::cout<<"\nIn Segment::buildWork()\n";

  // Set the intial velocities and accelerations of the segment
  for(unsigned int j=0;j<k_dof_;j++) {
    start_.v_.push_back(v_start_);
    start_.a_.push_back(0);
  }
  
  // Set the ending velocities and accelerations of the segment
  for(unsigned int j=0;j<k_dof_;j++) {
    end_.v_.push_back(v_end_);
    end_.a_.push_back(0);
  }


  // Set a0 coefficient for each DOF
  // For linear segments, a0 is simply the starting position
  for(unsigned int j=0;j<k_dof_;j++) {
    a0_.push_back(start_.p_.at(j));
  }
  

  // Calculate the minimum time required to perform this segment
  // This sets the T_ variable which we need to set a1
  calculateMinTime();

  
  /** Calculate a1 (slope) for each DOF. a1 depends on the corresponding T_ */
  // a1 for x
  if(T_loc_ == 0) {
    a1_.push_back(0);
  }
  else {
    a1_.push_back( (end_.p_.at(0) - start_.p_.at(0)) / T_loc_ );
  }

  // a1 for y
  if(T_loc_ == 0) {
    a1_.push_back(0);
  }
  else {
    a1_.push_back( (end_.p_.at(1) - start_.p_.at(1)) / T_loc_ );
  }

  // a1 for pre_rotation
  if(T_rotate_pre_ > 0) {
    a1_.push_back( pre_angle_dist / T_rotate_pre_ );
  }
  else {
    a1_.push_back(0);
  }

  // a1 for post_rotation
  if(plan_post) {
    if(T_rotate_post_ > 0) {
      a1_.push_back( post_angle_dist / T_rotate_post_ );
    }
    else {
      a1_.push_back(0);
    }
  }
} // End buildWork






/** 
  * This method calculates the minimum time needed to compute the trajectory 
  * The members T_rotate_pre_, T_loc_, T_rotate_post_, and T_min_ are set
  */
const void Segment::calculateMinTime() {
  // std::cout<<"\nIn Segment::calculateMinTime\n";
  // std::cout<<"\n\nend_.p_.at(0):"<<end_.p_.at(0)<<" start_.p_.at(0):"<<start_.p_.at(0);
  // std::cout<<"\nend_.p_.at(1):"<<end_.p_.at(1)<<" start_.p_.at(1):"<<start_.p_.at(1);


  //==============================================================================
  // Find pre_angle - the angle we should have to drive towards the goal
  pre_angle = u.findAngleFromAToB(start_.p_, end_.p_);
  //std::cout<<"\npre_angle: "<<pre_angle;

  // Find angle dist between pre_angle and starting orientation
  pre_angle_dist = u.findDistanceBetweenAngles(pre_angle, start_.p_.at(k_dof_-1));
  //std::cout<<"\npre_angle_dist: "<<pre_angle_dist;

  // Calculate time needed to rotate towards goal
  // if the difference is > 12 degrees
  if( fabs(pre_angle_dist) > (PI/15)) {
    T_rotate_pre_ = ceil(fabs( pre_angle_dist / max_v_.at(k_dof_-1)));
  } 
  else {
    T_rotate_pre_ = 0;
  }
  //==============================================================================



  //==============================================================================
  // Set T_loc_ - the time needed to go straight towards the goal
  T_loc_ = ceil(u.positionDistance(start_.p_, end_.p_) / max_v_.at(0));
  //==============================================================================



  //==============================================================================
  // Find angle dist between goal orientation and pre_angle 
  if(plan_post) {
    post_angle_dist = u.findDistanceBetweenAngles(end_.p_.at(k_dof_-1), pre_angle);
    
    // Calculate time needed to rotate to goal orientation 
    // if the difference is > 12 degrees
    if( fabs(post_angle_dist) > (PI/15))
      T_rotate_post_ = ceil(fabs( post_angle_dist / max_v_.at(k_dof_-1)));
    else {
      T_rotate_post_ = 0;
    }
  }
  //==============================================================================



  // Set min_T_
  T_min_ = T_loc_ + T_rotate_pre_ + T_rotate_post_;
} //End calculateMinTime



/** This method returns a string to display information about the Segment */
const std::string Segment::toString() const {
  std::ostringstream result;

  result<<"\n\n====================================";
  result<<"\nSegment "<<index_<<":"; 

  result<<"\nstarting orientation: "<<start_.p_.at(2);
  result<<"\ngoal orientation: "<<end_.p_.at(2);
  result<<"\npre_angle: "<<pre_angle;
  result<<"\npre_angle_dist: "<<pre_angle_dist;
  result<<"\npost_angle: "<<post_angle;
  result<<"\npost_angle_dist: "<<post_angle_dist;

  result<<"\nT_rotate_pre_: "<<T_rotate_pre_;
  result<<"\nT_loc_: "<<T_loc_;
  result<<"\nT_rotate_post_: "<<T_rotate_post_;
  result<<"\nT_min_: "<<T_min_;

  result<<"\n\nstart: "<<start_.toString();
  result<<"\n\nend: "<<end_.toString();
  
  result<<"\n\na1 coefficients: ("<<a1_.at(0);
  for(unsigned int i=1;i<a1_.size();i++) {
    result<<", "<<a1_.at(i);  
  }
  result<<")";

  result<<"\na0 coefficients: ("<<a0_.at(0);
  for(unsigned int i=1;i<a0_.size();i++) {
    result<<", "<<a0_.at(i);
  }
  result<<")";
  result<<"\n====================================";
  
  return result.str();
}
