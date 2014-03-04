#include "euclidean_distance.h"


/** This method sums the euclidean distances over all the path segments */
const double EuclideanDistance::perform() {
  double result=0;
  trajectory_msgs::JointTrajectory trajec = trajectory_.trajectory;

  //For each configuration,
  for(unsigned int i=0;i<trajectory_.index_knot_points.size()-1;i++) {


    unsigned int i1 = trajectory_.index_knot_points.at(i);
    unsigned int i2 = trajectory_.index_knot_points.at(i+1);

    double diff=0;
    //For each DOF
    for(unsigned int k=0;k<3;k++) {
      
      //squared difference of the values
      diff += pow( (trajec.points.at(i2).positions.at(k) - trajec.points.at(i1).positions.at(k)), 2 );

    }
    
    result += sqrt(diff);
  }
  
  return result;
}
