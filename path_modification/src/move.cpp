#include "move.h"



Move::Move(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Move::perform() 
{
  ROS_INFO("Before: %s", utility_.toString(path_).c_str()); 

  if(path_.points.size() > 2) 
  {

    // Randomly choose a knot point to change
    double dist = (double)rand() / RAND_MAX;
    ROS_INFO("dist: %f", dist);
    

    // Create point
    double x = cos(dir_) * dist;
    double y = sin(dir_) * dist;
    ramp_msgs::KnotPoint kp;
    kp.motionState.positions.push_back(x);
    kp.motionState.positions.push_back(y);

    path_.points.at(1) = kp;

    
    /*while(!checkConstraints_.validKPForPath(kp, tempPath))
    {
      // Generate new, random values for the positions
      kp.motionState.positions.clear();
      for(unsigned int i=0;i<path_.points.at(0).motionState.positions.size();i++) 
      {
        
        // Generate a random value for each K in the specified range
        double  min = utility_.standardRanges_.at(i).min;
        double  max = utility_.standardRanges_.at(i).max;
        float temp = (min == 0 && max == 0) ? 0 :      
              ( min + (float)rand() / ((float)RAND_MAX / (max - min)) );

        // Push the new value onto positions
        kp.motionState.positions.push_back(temp);
      } // end for
    } // end while*/
  } // end if points.size()>2

  ROS_INFO("After: %s", utility_.toString(path_).c_str()); 
  return path_;
}
