#include "orientation.h"

Orientation::Orientation() : Q_(1000. / PI) {}


const double Orientation::getDeltaTheta() const
{
  double result = 0.f;
  if(trajectory_.i_knotPoints.size() > 1) 
  {

    trajectory_msgs::JointTrajectoryPoint a = trajectory_.trajectory.points.at(0);
    trajectory_msgs::JointTrajectoryPoint b = trajectory_.trajectory.points.at(trajectory_.i_knotPoints.at(1));
    //ROS_INFO("a: %s\nb: %s", utility_.toString(a).c_str(), utility_.toString(b).c_str());
    
    double thetaNec = utility_.findAngleFromAToB(a, b);
    
    if(trajectory_.t_start.toSec() < 0.01)
    {
      result          = fabs( utility_.findDistanceBetweenAngles(currentTheta_, thetaNec) );
    }
    else
    {
      result          = fabs( utility_.findDistanceBetweenAngles(theta_at_cc_, thetaNec) );
    }

    //ROS_INFO("thetaNec: %f result: %f", thetaNec, result);
  }

  return result;
}


const double Orientation::perform() 
{
  double result = 0.;

  // Add the change in orientation needed to move on this trajectory
  // Check if there is more than 1 point
  if(trajectory_.i_knotPoints.size() > 1) 
  {

    trajectory_msgs::JointTrajectoryPoint a = trajectory_.trajectory.points.at(0);
    trajectory_msgs::JointTrajectoryPoint b = trajectory_.trajectory.points.at(trajectory_.i_knotPoints.at(1));
    //ROS_INFO("a: %s\nb: %s", utility_.toString(a).c_str(), utility_.toString(b).c_str());
    
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

    //ROS_INFO("thetaNec: %f deltaTheta: %f mag_linear: %f", thetaNec, deltaTheta, mag_linear);

    // If delta theta is too high, add a penalty
    //if(mag_linear > 0 && deltaTheta >= PI/2.f) 
    //{
      //ROS_INFO("Adding penalty for deltaTheta: %f", deltaTheta);
      double normalize = PI;
      deltaTheta /= normalize;
      result += (Q_ * normalize);
    //}
  } // end if > 1 knot point


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
