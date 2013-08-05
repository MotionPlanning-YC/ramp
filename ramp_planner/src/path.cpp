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

const std::string Path::toString() const {
  std::ostringstream result;

  std::cout<<"\nall_.size():"<<all_.size();
  for(unsigned int i=0;i<all_.size();i++) {
    std::cout<<"\ni:"<<i;
    result<<"\nConfiguration "<<i<<":";
    result<<"\n"<<all_.at(i).toString();
  }
  
  return result.str();
}
