#include "path.h"

Path::Path() {}

Path::Path(Configuration start, Configuration goal) : start_(start), goal_(goal) 
{
  all_.push_back(start);
  all_.push_back(goal); 
}

Path::~Path() {}

/** This method inserts the configuration c into the path at location path_size-1 */
void Path::Add(const Configuration c) {
  all_.insert(all_.end()-1, c); 
}


const ramp_msgs::Path Path::buildPathMsg() const {
  ramp_msgs::Path result;

  //Push all of the configurations onto the Path msg
  for(unsigned int i=0;i<all_.size();i++) {

    //Build the configuration msg
    ramp_msgs::Configuration c = all_.at(i).buildConfigurationMsg();
    
    //Push the msg onto K
    result.configurations.push_back(c);
  }

  return result;
}

const std::string Path::toString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<all_.size();i++) {
    result<<"\nConfiguration "<<i<<":";
    result<<"\n"<<all_.at(i).toString();
  }
  
  return result.str();
}
