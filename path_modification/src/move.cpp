#include "move.h"



Move::Move(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Move::perform() 
{
  //ROS_INFO("dir_: %f", dir_);
  //ROS_INFO("Before: %s", utility_.toString(path_).c_str()); 

  if(path_.points.size() > 2) 
  {
    double dir = dir_;
    double min = PI/2.f;
    double max = (3.f*PI)/2.f;

    double displacement = ((float)rand() / (float)RAND_MAX);
    //ROS_INFO("displacement: %f", displacement);
    
    displacement *= (max-min);
    
    //ROS_INFO("displacement: %f", displacement);

    displacement += PI/2.f;

    
    dir = utility_.displaceAngle(dir, displacement);
    //ROS_INFO("dir_: %f displacement: %f dir: %f", dir_, displacement, dir);

    // Randomly choose a knot point to change
    double dist = (double)rand() / RAND_MAX;
    //ROS_INFO("dist: %f", dist);
    

    // Create point
    double x = cos(dir) * dist;
    double y = sin(dir) * dist;
    
    ramp_msgs::KnotPoint kp;
    kp.motionState.positions.push_back(x);
    kp.motionState.positions.push_back(y);
    kp.motionState.positions.push_back(dir);

    path_.points.at(1) = kp;
  } // end if points.size()>2

  //ROS_INFO("After: %s", utility_.toString(path_).c_str()); 
  return path_;
}
