#include "swap.h"


Swap::Swap(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Swap::perform() {

  if(path_.configurations.size() > 3) {
    unsigned int i_knotPoint1 = rand() % (path_.configurations.size()-2) + 1;
    unsigned int i_knotPoint2; 
    
    //Make sure the two configurations are different
    do {
      i_knotPoint2 = rand() % (path_.configurations.size()-2) + 1;
    } while(i_knotPoint1 == i_knotPoint2);

    //Swap the configurations
    ramp_msgs::Configuration temp = path_.configurations.at(i_knotPoint1);
    path_.configurations.at(i_knotPoint1) = path_.configurations.at(i_knotPoint2);
    path_.configurations.at(i_knotPoint2) = temp;

  }

  return path_;
}
