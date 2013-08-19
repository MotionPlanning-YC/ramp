#include "delete.h"


Delete::Delete(const ramp_msgs::Path p) : path_(p) {}


const ramp_msgs::Path Delete::perform() {

  if(path_.configurations.size() > 2) {
 
    //Randomly choose a knot point to delete 
    //Cannot delete the start or goal, so adjust the range a bit, range= [1,(size-2)]
    unsigned int i_knotPoint = rand() % (path_.configurations.size()-2) + 1; 

    //Delete the knot point
    path_.configurations.erase(path_.configurations.begin()+i_knotPoint);

  }

  return path_;
}
