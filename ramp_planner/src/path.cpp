#include "path.h"

Path::Path() {}


Path::Path(MotionState start, MotionState goal) : start_(start), goal_(goal) {
  all_.push_back(start);
  all_.push_back(goal); 
}


Path::Path(std::vector<MotionState> all) {
  start_ = all.at(0);
  goal_ = all.at(all.size()-1);
  

  for(unsigned int i=0;i<all.size();i++) {
    all_.push_back(all.at(i));
  }
}

Path::Path(ramp_msgs::Path p) {

  MotionState s(p.points.at(0));
  start_ = s;

  MotionState g(p.points.at(p.points.size()-1));
  goal_ = g;

  for(unsigned int i=0;i<p.points.size();i++) {
    MotionState mp(p.points.at(i));
    all_.push_back(mp);
  }
}

Path::~Path() {}


void Path::Add(const MotionState mp) {
  all_.insert(all_.end()-1, mp);
}

/** This method inserts the configuration c into the path at location path_size-1 */
void Path::Add(const Configuration c) {
  MotionState mp(c);
  Add(mp);
}

const unsigned int Path::size() const { return all_.size(); }


const ramp_msgs::Path Path::buildPathMsg() const {
  ramp_msgs::Path result;

  //Push all of the configurations onto the Path msg
  for(unsigned int i=0;i<all_.size();i++) {

    //Build the motion state msg
    ramp_msgs::MotionState mp = all_.at(i).buildMotionStateMsg();
    
    //Push the msg onto K
    result.points.push_back(mp);
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
