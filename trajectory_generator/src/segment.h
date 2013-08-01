#ifndef SEGMENT
#define SEGMENT

#include "utility.h"
#include "motion_state.h"

class Segment {
  public:
  
    Segment();
    
    //TODO: Put all these parameters in a struct
    Segment(geometry_msgs::Pose2D kp_start, geometry_msgs::Pose2D kp_end, float t_start, float t_end, unsigned int ind);
    ~Segment();

    void build(geometry_msgs::Pose2D kp_start, geometry_msgs::Pose2D kp_end, float t_start, float t_end, unsigned int ind);

    //State of motion describing the start 
    MotionState start_;

    //State of motion describing the end 
    MotionState end_;

    //The slope for each DOF in the segment
    std::vector<double> a1_; //the slope
    std::vector<double> a0_; //constant

    float start_t_;
    float end_t_;


    int index; 
  private:
    void buildWork();
    unsigned int k_dof_;
};
#endif 
