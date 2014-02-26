#include "change.h"


Change::Change(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Change::perform() {

  if(path_.points.size() > 2) {

    // Randomly choose a knot point to change
    unsigned int i_knotPoint = rand() % (path_.points.size()-2) + 1;
    //std::cout<<"\ni_knotPoint: "<<i_knotPoint;

    // Generate a new, random configuration
    ramp_msgs::KnotPoint kp;
    for(unsigned int i=0;i<path_.points.at(0).configuration.K.size();i++) {
      
      // Generate a random value for each K in the specified range
      double min = path_.points.at(0).configuration.ranges.at(i).min;
      double max = path_.points.at(0).configuration.ranges.at(i).max;
      
      float temp = (min == 0 && max == 0) ? 0 :      
            ( min + (float)rand() / ((float)RAND_MAX / (max - min)) );

      kp.configuration.K.push_back(temp);
      kp.configuration.ranges.push_back(path_.points.at(0).configuration.ranges.at(i));
    }


    // Remove the knot point
    path_.points.erase(path_.points.begin()+i_knotPoint);

    // Insert the new configuration 
    path_.points.insert(path_.points.begin()+i_knotPoint, kp);

  }

  return path_;
}
