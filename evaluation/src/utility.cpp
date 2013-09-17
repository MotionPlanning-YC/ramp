#include "utility.h"



const std::string Utility::toString(const ramp_msgs::Trajectory traj) const {
  std::ostringstream result;

  result<<"\n Knot Points:";

  for(unsigned int i=0;i<traj.index_knot_points.size();i++) {
    
    result<<"\n   "<<i<<":";
    unsigned int index = traj.index_knot_points.at(i);

    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(index);
    


    result<<"\n       Positions: ("<<p.positions.at(0);
    for(unsigned int k=1;k<p.positions.size();k++) {
      result<<", "<<p.positions.at(k);
    }
    result<<")";

  }


  result<<"\n Points:";
  for(unsigned int i=0;i<traj.trajectory.points.size();i++) {
    result<<"\n\n   Point "<<i<<":";
    
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(i);
  
    //Positions
    result<<"\n       Positions: ("<<p.positions.at(0);
    for(unsigned int k=1;k<p.positions.size();k++) {
      result<<", "<<p.positions.at(k);
    }
    result<<")";
  
    //Velocities
    result<<"\n       Velocities: ("<<p.velocities.at(0);
    for(unsigned int k=1;k<p.velocities.size();k++) {
      result<<", "<<p.velocities.at(k);
    }
    result<<")";
    
    //Accelerations
    result<<"\n       Accelerations: ("<<p.accelerations.at(0);
    for(unsigned int k=1;k<p.accelerations.size();k++) {
      result<<", "<<p.accelerations.at(k);
    }
    result<<")";
    
    result<<"\n Time From Start: "<<p.time_from_start;

  }

  return result.str();
}
