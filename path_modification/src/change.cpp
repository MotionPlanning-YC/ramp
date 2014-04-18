#include "change.h"



Change::Change(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Change::perform() {

  if(path_.points.size() > 2) {

    // Randomly choose a knot point to change
    unsigned int i_knotPoint = rand() % (path_.points.size()-2) + 1;
    //std::cout<<"\ni_knotPoint: "<<i_knotPoint;

    
    // Clear the knot point's positions
    path_.points.at(i_knotPoint).motionState.positions.clear();

    // Generate new, random values for the positions
    for(unsigned int i=0;i<path_.points.at(0).motionState.positions.size();i++) {
      
      // Generate a random value for each K in the specified range
      double  min = utility.standardRanges_.at(i).min;
      double  max = utility.standardRanges_.at(i).max;
      float temp = (min == 0 && max == 0) ? 0 :      
            ( min + (float)rand() / ((float)RAND_MAX / (max - min)) );

      // Push the new value onto positions
      path_.points.at(i_knotPoint).motionState.positions.push_back(temp);
    }
  } // end if points.size()>2

  return path_;
}
