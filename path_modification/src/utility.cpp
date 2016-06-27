#include "utility.h"

Utility::Utility() {
  for(unsigned int i=0;i<3;i++) {
    ramp_msgs::Range temp;
    temp.min = (i < 2 ? 0   : -PI);
    temp.max = (i < 2 ? 3.5 :  PI);
    standardRanges_.push_back(temp);
  }
}



/** This method returns distance between orientations a1 and a2. The distance is in the range [-PI, PI]. */
const double Utility::findDistanceBetweenAngles(const double a1, const double a2) const 
{
  double result;
  double difference = a2 - a1;
  
  // If difference > pi, the result should be in [-PI,0] range
  if(difference > PI) 
  {
    difference = fmodf(difference, PI);
    result = difference - PI;
  }

  // If difference < -pi, the result should be in [0,PI] range
  else if(difference < -PI) 
  {
    result = difference + (2*PI);
  }

  // Else, the difference is fine
  else 
  {
    result = difference;
  }

  return result;
} // End findDistanceBetweenAngles



const double Utility::displaceAngle(const double a1, double a2) const {

  a2 = fmodf(a2, 2*PI);

  if(a2 > PI) 
  {
    a2 = fmodf(a2,PI) - PI;
  }

  return findDistanceBetweenAngles(-a1, a2);
} // End displaceAngle




const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath:";
  for(unsigned int i=0;i<path.points.size();i++) {
    result<<"\n  "<<i<<": (";

    result<<path.points.at(i).motionState.positions.at(0);
    for(unsigned int k=1;k<path.points.at(i).motionState.positions.size();k++) {
      result<<", "<<path.points.at(i).motionState.positions.at(k);
    }
    result<<")";
  }

  return result.str();
}

