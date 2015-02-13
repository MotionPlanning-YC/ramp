#include "time.h"


const double Time::perform() {
  double result = 0.;

  // Get the total time to move along the trajectory
  if(trajectory_.trajectory.points.size() > 0) {
    result = trajectory_.trajectory.points.at( trajectory_.trajectory.points.size()-1 ).time_from_start.toSec();
  }

  return result;
}
