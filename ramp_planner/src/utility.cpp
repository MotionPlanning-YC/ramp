#include "utility.h"

const std::string Utility::toString(const ramp_msgs::Trajectory traj) const {
  std::ostringstream result;

  result<<"\nTrajectory "<<traj.id;
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

  return result.str();
}
