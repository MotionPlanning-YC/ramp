#ifndef SEGMENT
#define SEGMENT

#include "utility.h"
#include "motion_state.h"

class Segment {
  public:
  
    Segment();
    
    // TODO: Put all these parameters in a struct?
    Segment(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float v_start, const float v_end, const unsigned int ind);
    ~Segment();

    /** Methods */
    void build(const geometry_msgs::Pose2D kp_start, const geometry_msgs::Pose2D kp_end, const float v_start, const float v_end, const unsigned int ind);

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

    // Minimum time required to execute the segment
    // This value may need to change to a float in the future
    //std::vector<unsigned int> T_;
    unsigned int T_min_;
    unsigned int T_loc_;
    int T_rotate_pre_;
    int T_rotate_post_;
    float angle_pre;
     
    // The segment's index in whichever trajectory it is in
    int index_; 
    

  private:
    float k_dof_;
    std::vector<float> max_v_;
    const void calculateMinTime();
    void buildWork();
};
#endif 
