#include "orientation.h"

Orientation::Orientation() : Q_(10000.) {}


const double Orientation::perform() {
  double result = 0.;

  // Add the change in orientation needed to move on this trajectory
  // Check there is more than 1 points
  if(trajectory_.i_knotPoints.size() > 1) 
  {
    double thetaNec = utility_.findAngleFromAToB(trajectory_.trajectory.points.at(0),
      trajectory_.trajectory.points.at( trajectory_.i_knotPoints.at(1) ));   
    double deltaTheta = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );
    ROS_INFO("thetaNec: %f currentTheta_: %f deltaTheta: %f", thetaNec, currentTheta_, deltaTheta);

    double normalize = PI;
    deltaTheta /= normalize;

    // Normalize
    result += deltaTheta;
  }


  return result;
}


const double Orientation::getPenalty() const {
  double result = 0.;

  if(trajectory_.i_knotPoints.size() > 1) 
  {
    double thetaNec = utility_.findAngleFromAToB(trajectory_.trajectory.points.at(0),
      trajectory_.trajectory.points.at( trajectory_.i_knotPoints.at(1) ));   
    double deltaTheta = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );

    if(deltaTheta > PI/2) 
    {
      result += (Q_ / deltaTheta); 
    }
    //else if(deltaTheta > PI/4) {
      //result += ( (Q_/2.) / deltaTheta );
    //}
  } // end if > 1 knot point

  return result;
}
