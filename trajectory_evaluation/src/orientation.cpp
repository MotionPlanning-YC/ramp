#include "orientation.h"

Orientation::Orientation() : Q_(1000. / (50.*PI)) {}


const double Orientation::perform() 
{
  double result = 0.;

  // Add the change in orientation needed to move on this trajectory
  // Check if there is more than 1 point
  if(trajectory_.i_knotPoints.size() > 1) 
  {

    // Check for rotation at beginning
    trajectory_msgs::JointTrajectoryPoint a = trajectory_.trajectory.points.at(0);
    trajectory_msgs::JointTrajectoryPoint b = trajectory_.trajectory.points.at(trajectory_.i_knotPoints.at(1));
    //ROS_INFO("a: %s\nb: %s", utility_.toString(a).c_str(), utility_.toString(b).c_str());
    if( fabs(utility_.positionDistance(a.positions, b.positions)) < 0.01)
    {
      if(trajectory_.i_knotPoints.size() > 2)
      {
        //ROS_INFO("Rotation at beginning, setting b to knot point 2");
        b = trajectory_.trajectory.points.at(trajectory_.i_knotPoints.at(2));
        //ROS_INFO("b: %s", utility_.toString(b).c_str());
      }
      //else
      //{
        //ROS_WARN("Only two knot points and the points are equal.");
        //ROS_WARN("Point 0: %s\nPoint at KP 1: %s", utility_.toString(a).c_str(), utility_.toString(b).c_str());
      //}
    }


    double thetaNec = utility_.findAngleFromAToB(a, b);   
    double deltaTheta = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );
    //ROS_INFO("thetaNec: %f currentTheta_: %f deltaTheta: %f", thetaNec, currentTheta_, deltaTheta);

    double normalize = PI;
    deltaTheta /= normalize;


    // Normalize
    result += deltaTheta;
  }


  return result;
}


const double Orientation::getPenalty() const 
{
  double result = 0.;

  if(trajectory_.i_knotPoints.size() > 1) 
  {
    double thetaNec = utility_.findAngleFromAToB(trajectory_.trajectory.points.at(0),
      trajectory_.trajectory.points.at( trajectory_.i_knotPoints.at(1) ));   
    double deltaTheta = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );

    double mag_linear = sqrt( pow(trajectory_.trajectory.points.at(0).velocities.at(0), 2) + 
        pow(trajectory_.trajectory.points.at(0).velocities.at(1), 2) );


    // If delta theta is too high, add a penalty
    if(mag_linear > 0 && deltaTheta >= PI/2.f) 
    {
      double normalize = PI;
      deltaTheta /= normalize;
      result += (Q_ * normalize);
    }
  } // end if > 1 knot point*/


  //ROS_INFO("trajectory.size(): %i", (int)trajectory_.trajectory.points.size());
  if(trajectory_.trajectory.points.size() > 2)
  {
    trajectory_msgs::JointTrajectoryPoint p = trajectory_.trajectory.points.at(2);
    double v = sqrt( pow(p.velocities.at(0), 2) + pow(p.velocities.at(1), 2) );
    double w = p.velocities.at(2);

    if(fabs(v) < 0.0001 && fabs(w) > 0.01)
    {
      result += 1000;
    }
  }

  return result;
}
