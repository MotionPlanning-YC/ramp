#include "path.h"

Path::Path() {}

Path::Path(KnotPoint start, KnotPoint goal) : start_(start), goal_(goal) {
  all_.push_back(start);
  all_.push_back(goal); 
}

Path::Path(std::vector<KnotPoint> all) {
  start_ = all.at(0);
  goal_ = all.at(all.size()-1);
  

  for(unsigned int i=0;i<all.size();i++) {
    all_.push_back(all.at(i));
  }
}

Path::Path(ramp_msgs::Path p) {

  KnotPoint s(p.points.at(0));
  start_ = s;

  KnotPoint g(p.points.at(p.points.size()-1));
  goal_ = g;

  for(unsigned int i=0;i<p.points.size();i++) {
    KnotPoint c(p.points.at(i));
    all_.push_back(c);
  }
}

Path::~Path() {}

/** This method inserts the configuration c into the path at location path_size-1 */
void Path::Add(const KnotPoint kp) {
  all_.insert(all_.end()-1, kp); 
}


/** This method inserts the configuration c into the path at location path_size-1 */
void Path::Add(const Configuration c) {
  KnotPoint kp(c);
  Add(kp);
}


const unsigned int Path::size() const { return all_.size(); }


const ramp_msgs::Path Path::buildPathMsg() const {
  ramp_msgs::Path result;

  //Push all of the configurations onto the Path msg
  for(unsigned int i=0;i<all_.size();i++) {

    //Build the configuration msg
    ramp_msgs::KnotPoint kp = all_.at(i).buildKnotPointMsg();
    
    //Push the msg onto K
    result.points.push_back(kp);
  }

  return result;
}

const std::string Path::toString() const {
  std::ostringstream result;

  for(unsigned int i=0;i<all_.size();i++) {
    result<<"\n  "<<i<<": "<<all_.at(i).toString();
  }
  
  return result.str();
}
