#include "insert.h"


Insert::Insert(ramp_msgs::Path p) : path_(p) {}


ramp_msgs::Path Insert::perform() {

  //Randomly choose the two adjacent knot points
  //One index is generated randomly and the second will be the next knot point,
  //unless the generated index is the last knot point index
  unsigned int i_kp1 = rand() % path_.configurations.size(); 
  unsigned int i_kp2 = (i_kp1 == path_.configurations.size()-1) ? i_kp1-1 : i_kp1+1;

  //If the last index was chosen, swap the values so i_kp1 < i_kp2
  //this just makes it simpler 
  if(i_kp2 < i_kp1) {
    unsigned int swap = i_kp1;
    i_kp1 = i_kp2;
    i_kp2 = swap;
  }


  //Generate a new, random knot point
}
