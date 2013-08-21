#include "utility.h"

const std::string Utility::toString(const geometry_msgs::Pose2D p) const {
  std::ostringstream result;
  result<<"\nPose2D: ("<<p.x<<", "<<p.y<<", "<<p.theta<<")";
  return result.str();
}

const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath:";
  for(unsigned int i=0;i<path.configurations.size();i++) {
    result<<"\n  "<<i<<": (";

    result<<path.configurations.at(i).K.at(0);
    for(unsigned int k=1;k<path.configurations.at(i).K.size();k++) {
      result<<", "<<path.configurations.at(i).K.at(k);
    }
    result<<")";

  }

  return result.str();
}

const std::string Utility::toString(const ramp_msgs::TrajectoryRequest::Request tr) const {
  std::ostringstream result;
  result<<"\nTrajectory Request:";
  result<<"\nPath:"<<tr.path;


  result<<"\nv_start: ("<<tr.v_start.at(0);
  for(unsigned int i=0;i<tr.v_start.size();i++) {
    result<<", "<<tr.v_start.at(i);
  }
  result<<")";

  result<<"\nv_end: ("<<tr.v_end.at(0);
  for(unsigned int i=0;i<tr.v_end.size();i++) {
    result<<", "<<tr.v_end.at(i);
  }
  result<<")";
  
  result<<"\nresolutionRate: "<<tr.resolutionRate;

  return result.str();
}


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

  }

  return result.str();
}
