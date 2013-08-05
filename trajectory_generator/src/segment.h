#ifndef SEGMENT
#define SEGMENT

#include "utility.h"
#include "motion_state.h"

class Segment {
  public:
  
    Segment();
    
    //TODO: Put all these parameters in a struct?
    Segment(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float t_start, const float t_end, const unsigned int ind);
    ~Segment();

    void build(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float t_start, const float t_end, const unsigned int ind);

    //State of motion describing the start 
    MotionState start_;

    //State of motion describing the end 
    MotionState end_;

    //The slope for each DOF in the segment
    std::vector<double> a1_; //the slope
    std::vector<double> a0_; //constant

    //Starting and ending times
    float start_t_;
    float end_t_;
    
    //The segment's index in whichever trajectory it is in
    int index; 

  private:
    void buildWork();
    unsigned int k_dof_;
};
#endif 
