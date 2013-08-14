#include "insert.h"


Insert::Insert(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Insert::perform() {

  //Randomly choose the two adjacent knot points
  //One index is generated randomly and the second will be the next knot point,
  //unless the generated index is the last knot point index
  unsigned int i_knotPoint1 = rand() % path_.configurations.size(); 
  unsigned int i_knotPoint2 = (i_knotPoint1 == path_.configurations.size()-1) ? i_knotPoint1-1 : i_knotPoint1+1;

  //If the last index was chosen, swap the values so i_knotPoint1 < i_knotPoint2
  //this just makes it simpler 
  if(i_knotPoint2 < i_knotPoint1) {
    unsigned int swap = i_knotPoint1;
    i_knotPoint1 = i_knotPoint2;
    i_knotPoint2 = swap;
  }

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

  //Insert the configuration 
  path_.configurations.insert(path_.configurations.begin()+i_knotPoint2, c);
  
 return path_; 
}
