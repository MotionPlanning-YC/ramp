#include "change.h"


Change::Change(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Change::perform() {

  if(path_.configurations.size() > 2) {

    //Randomly choose a knot point to change
    unsigned int i_knotPoint = rand() % (path_.configurations.size()-2) + 1;

    //Generate a new, random configuration
    ramp_msgs::Configuration c;
    for(unsigned int i=0;i<path_.configurations.at(0).K.size();i++) {
      
      //Generate a random value for each K in the specified range
      double min = path_.configurations.at(0).ranges.at(i).min;
      double max = path_.configurations.at(0).ranges.at(i).max;
      
      float temp = ( min + (float)rand() / ((float)RAND_MAX / (max - min)) );

      c.K.push_back(temp);
      c.ranges.push_back(path_.configurations.at(0).ranges.at(i));
    }


    //Remove the knot point
    path_.configurations.erase(path_.configurations.begin()+i_knotPoint);

    //Insert the new configuration 
    path_.configurations.insert(path_.configurations.begin()+i_knotPoint, c);

  }

  return path_;
}
