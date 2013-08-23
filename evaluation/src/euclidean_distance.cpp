#include "euclidean_distance.h"


/** This method sums the euclidean distances over all the path segments */
const double EuclideanDistance::perform() {
  double result=0;

  //For each coniguration,
  for(unsigned int i=0;i<path_.configurations.size()-1;i++) {

    double diff=0;
    //For each DOF
    for(unsigned int k=0;k<3;k++) {

      //squared difference of the values
      diff += pow( (path_.configurations.at(i+1).K.at(k) - path_.configurations.at(i).K.at(k)), 2 );
    }

    result += sqrt(diff);
  }
  
  return result;
}
