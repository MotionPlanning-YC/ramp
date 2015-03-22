#include "time.h"


const double Time::perform() {
  double result = 0.;

  // Get the total time to move along the trajectory
  if(trajectory_.trajectory.points.size() > 0) 
  {
    result = trajectory_.trajectory.points.at( trajectory_.trajectory.points.size()-1 ).time_from_start.toSec();
  }

  // Add how long in the future the trajectory is being executed
  result += trajectory_.t_start.toSec();

  // Normalize the value based on timeCutoff in trajectory_generator
  double normalize = 50.;
  result /= normalize;

  return result;
}
