#include "utility.h"

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


  result<<"\nv: ("<<tr.v.at(0);
  for(unsigned int i=0;i<tr.v.size();i++) {
    result<<", "<<tr.v.at(i);
  }
  result<<")";

  result<<"\nresolutionRate: "<<tr.resolutionRate;

  return result.str();
}
