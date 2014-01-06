#include "segment.h"


Segment::Segment() : k_dof_(3) {
  max_v_.push_back(0.33f);
  max_v_.push_back(0.33f);
  max_v_.push_back(0.33f);
}

Segment::Segment(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float v_start, const float v_end, const unsigned int ind) : k_dof_(3) 
{
  max_v_.push_back(0.33f);
  max_v_.push_back(0.33f);
  max_v_.push_back(0.33f);
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
  
  index_     = ind;
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
  
  // Need to take angle_pre into account
  

  // a1 for x
  a1_.push_back( (end_.p_.at(0) - start_.p_.at(0)) / T_loc_ );
  // a1 for y
  a1_.push_back( (end_.p_.at(1) - start_.p_.at(1)) / T_loc_ );

  // a1 for pre_rotation
  if(T_rotate_pre_ > 0)
    a1_.push_back( (angle_pre - start_.p_.at(2)) / T_rotate_pre_ );
  else 
    a1_.push_back(0);

  // a1 for post_rotation
  if(T_rotate_post_ > 0)
    a1_.push_back( (end_.p_.at(2) - angle_pre) / T_rotate_post_ );
  else
    a1_.push_back(0);

} // End buildWork



/** This method calculates the minimum time needed to compute the trajectory 
  * The members T_rotate_pre_, T_loc_, T_rotate_post_, and T_min_ are set
  */
const void Segment::calculateMinTime() {
  // std::cout<<"\nIn Segment::calculateMinTime\n";

  //std::cout<<"\n\nend_.p_.at(0):"<<end_.p_.at(0)<<" start_.p_.at(0):"<<start_.p_.at(0);
  //std::cout<<"\nend_.p_.at(1):"<<end_.p_.at(1)<<" start_.p_.at(1):"<<start_.p_.at(1);

  // Find Euclidean distance between [x,y] of start and goal
  float d_x = end_.p_.at(0) - start_.p_.at(0);
  float d_y = end_.p_.at(1) - start_.p_.at(1);
  float euc_dist = sqrt( pow(d_x,2) + pow(d_y,2) );
  std::cout<<"\nd_x: "<<d_x<<" d_y:"<<d_y<<" euc_dist:"<<euc_dist;

  // Calculate time needed to rotate towards goal
  angle_pre = asin(d_y / euc_dist);
  std::cout<<"\nangle_pre: "<<angle_pre;
  angle_pre = angle_pre * M_PI;
  float angle_dist = angle_pre - start_.p_.at(k_dof_-1);
  if( fabs(angle_dist) > 0.1) {
    T_rotate_pre_ = ceil(fabs(angle_dist / max_v_.at(k_dof_-1)));
  } 
  else
    T_rotate_pre_ = 0;
  std::cout<<"\nangle_pre:"<<angle_pre<<" angle_dist:"<<angle_dist;

  // Then add to T_loc_ the time to go straight towards the goal
  T_loc_ = ceil(euc_dist / max_v_.at(0));

  // Now find the time required to rotate to desired goal orientation
  angle_dist = end_.p_.at(k_dof_-1) - angle_pre;
  if( fabs(angle_dist) > 0.15)
    T_rotate_post_ = ceil(fabs( angle_dist / max_v_.at(k_dof_-1)));
  else
    T_rotate_post_ = 0;
  //T_rotate_post_ = ceil(fabs( (end_.p_.at(k_dof_-1) - angle_pre) / max_v_.at(k_dof_-1)));

  // Set min_T_
  T_min_ = T_loc_ + T_rotate_pre_ + T_rotate_post_;

  //std::cout<<"\nT_rotate_pre_:"<<T_rotate_pre_<<" T_loc_:"<<T_loc_<<" T_rotate_post_:"<<T_rotate_post_;

} //End calculateMinTime



/** This method returns a string to display information about the Segment */
const std::string Segment::toString() const {
  std::ostringstream result;

  
  result<<"\nindex_: "<<index_; 

  result<<"\nT_rotate_pre_:"<<T_rotate_pre_;
  result<<"\nT_loc_:"<<T_loc_;
  result<<"\nT_rotate_post_:"<<T_rotate_post_;
  result<<"\nT_min_:"<<T_min_;

  result<<"\n\nstart:"<<start_.toString();
  result<<"\n\nend:"<<end_.toString();
  
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
  
  return result.str();
}
