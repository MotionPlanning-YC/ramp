#include "time.h"


const double Time::perform() {
  double result;

  if(trajectory_.trajectory.points.size() > 0) {
    result = trajectory_.trajectory.points.at( trajectory_.trajectory.points.size()-1 ).time_from_start.toSec();
  }
  else
    result = 0;

  return result;
}
