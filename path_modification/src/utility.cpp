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
