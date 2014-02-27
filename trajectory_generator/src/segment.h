#ifndef SEGMENT
#define SEGMENT

#include "utility.h"
#include "motion_state.h"

class Segment {
  public:
  
    Segment();
    
    // TODO: Put all these parameters in a struct?
    Segment(const ramp_msgs::KnotPoint kp_start, const ramp_msgs::KnotPoint kp_end, const float v_start, const float v_end, const unsigned int ind);
    ~Segment();

    /** Methods */
    void build(const ramp_msgs::KnotPoint kp_start, const ramp_msgs::KnotPoint kp_end, const float v_start, const float v_end, const unsigned int ind);

    const std::string toString() const;
    
    
    /** Data members */

    // State of motion describing the start 
    MotionState start_;

    // State of motion describing the end 
    MotionState end_;

    // The slope for each DOF in the segment
    std::vector<double> a1_; // the slope
    std::vector<double> a0_; // constant

    // The velocities at the bounding knot points of the segment
    float v_start_;
    float v_end_;

    // Minimum times required to execute the segment
    // These values may need to change to float in the future
    int T_min_;
    int T_loc_;
    int T_rotate_pre_;
    int T_rotate_post_;
    int T_stop_;
    float pre_angle;
    float post_angle;
    float pre_angle_dist;
    float post_angle_dist;
     
    // The segment's index in whichever trajectory it is in
    int index_; 
    

    bool plan_post;
  private:
    Utility u;
    float k_dof_;
    std::vector<float> max_v_;
    const void calculateMinTime();
    void buildWork();
};
#endif 
