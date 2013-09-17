#include "time.h"


const double Time::perform() {
  double result;

  result = trajectory_.trajectory.points.at( trajectory_.trajectory.points.size()-1 ).time_from_start.toSec();

  return result;
}
