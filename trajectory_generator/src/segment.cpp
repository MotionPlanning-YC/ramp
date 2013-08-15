#include "segment.h"


Segment::Segment() : k_dof_(3) {
  max_v_.push_back(1.0f);
  max_v_.push_back(1.0f);
  max_v_.push_back(5.0f);
}

Segment::Segment(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float t_start, const float t_end, const unsigned int ind) : k_dof_(3) 
{
  max_v_.push_back(1.0f);
  max_v_.push_back(1.0f);
  max_v_.push_back(5.0f);
  build(kp_start, kp_end, t_start, t_end, ind);
}

Segment::~Segment() {}


/** This function assigns the Segment's members and calls buildWork() */
void Segment::build(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float v_start, const float v_end, const unsigned int ind) {

  start_.p_.push_back(kp_start.x);
  start_.p_.push_back(kp_start.y);
  start_.p_.push_back(kp_start.theta);

  end_.p_.push_back(kp_end.x);
  end_.p_.push_back(kp_end.y);
  end_.p_.push_back(kp_end.theta); 
  
  index     = ind;
  v_start_  = v_start;
  v_end_    = v_end;

  buildWork();
}


const float Segment::calculateMinTime() {

  T_ = abs((end_.p_.at(0) - start_.p_.at(0)) / max_v_.at(0));
  
  //Compute the execution time for each k
  for(unsigned int i=1;i<k_dof_;i++) {
    
    float t = (end_.p_.at(i) - start_.p_.at(i)) / max_v_.at(i);
    
    if(T_ < t)
      T_ = t;
  }

  return T_;
}

/** This function initializes the Segment's start and end states of motion, and the segment's a0 and a1 coefficients for interpolation */
void Segment::buildWork() {

  //The initial velocities and accelerations of each segment are 0
  for(unsigned int j=0;j<k_dof_;j++) {
    start_.v_.push_back(v_start_);
    start_.a_.push_back(0);
  }
  
  //The goal velocities and accelerations of each segment are 0
  for(unsigned int j=0;j<k_dof_;j++) {
    end_.v_.push_back(v_end_);
    end_.a_.push_back(0);
  }


  //Set a0 coefficient for each DOF
  //For linear segments, a0 is simply the starting position
  for(unsigned int j=0;j<k_dof_;j++) {
    a0_.push_back(start_.p_.at(j));
  }
  
  calculateMinTime();

  //Compute a1 for each DOF
  //For linear segments, a1 is simply the slope
  for(unsigned int j=0;j<k_dof_;j++) {
    a1_.push_back( (end_.p_.at(j) - start_.p_.at(j)) / T_);
  }
}



const std::string Segment::toString() const {
  std::ostringstream result;
  
  result<<"\nIndex: "<<index; 

  result<<"\nT:"<<T_;
  
  result<<"\na1 coefficients: ("<<a1_.at(0);
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
