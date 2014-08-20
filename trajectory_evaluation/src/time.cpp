#include "time.h"


const double Time::perform() {
  double result = 0.;
  double max_ang_speed = PI/3.;

  // Get the total time to move along the trajectory
  if(trajectory_.trajectory.points.size() > 0) {
    result = trajectory_.trajectory.points.at( trajectory_.trajectory.points.size()-1 ).time_from_start.toSec();
  }


  /* 
  // Estimate time for any turns
  trajectory_msgs::JointTrajectoryPoint a, b, c;

  int num_turns = trajectory_.index_knot_points.size()-1;
  for(uint8_t i=0;i<num_turns;i++) {


    if(i > 0) {

      a = trajectory_.trajectory.points.at( trajectory_.index_knot_points.at(i-1) );
      b = trajectory_.trajectory.points.at( trajectory_.index_knot_points.at(i) );
      c = trajectory_.trajectory.points.at( trajectory_.index_knot_points.at(i+1) );
      double theta = utility_.findAngleFromAToB(a, b);
      double phi   = utility_.findAngleFromAToB(b, c);
      double beta  = utility_.findDistanceBetweenAngles(theta, phi);
      
      result += pow(beta / max_ang_speed, 2);
    }
    else {
      a = trajectory_.trajectory.points.at( trajectory_.index_knot_points.at(i) );
      b = trajectory_.trajectory.points.at( trajectory_.index_knot_points.at(i+1) );
      double theta = utility_.findAngleFromAToB(a, b);
      double beta = utility_.findDistanceBetweenAngles(theta, a.positions.at(2));

      result += pow(beta / max_ang_speed, 2);
    }
  } // end for


  // Add on time to rotate from last theta to the goal
  trajectory_msgs::JointTrajectoryPoint last = trajectory_.trajectory.points.at( trajectory_.trajectory.points.size()-1 );
  double phi = utility_.findAngleFromAToB(last.positions, goal_.positions);
  double beta = utility_.findDistanceBetweenAngles(phi, last.positions.at(2));

  result += pow(beta / max_ang_speed, 2);
  */

  return result;
}
