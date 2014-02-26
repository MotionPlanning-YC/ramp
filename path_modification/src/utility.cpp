#include "utility.h"


const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath:";
  for(unsigned int i=0;i<path.points.size();i++) {
    result<<"\n  "<<i<<": (";

    result<<path.points.at(i).configuration.K.at(0);
    for(unsigned int k=1;k<path.points.at(i).configuration.K.size();k++) {
      result<<", "<<path.points.at(i).configuration.K.at(k);
    }
    result<<")";
  }

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::Configuration c) const {
  std::ostringstream result;

  result<<"\n  (";

  result<<c.K.at(0);
  for(unsigned int k=1;k<c.K.size();k++) {
    result<<", "<<c.K.at(k);
  }
  result<<")";

  return result.str();
}
