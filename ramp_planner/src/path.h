#ifndef PATH_H
#define PATH_H
#include "configuration.h"
#include "ramp_msgs/Path.h"

class Path {
  public:

    Path();
    Path(Configuration start, Configuration goal);
    Path(std::vector<Configuration> all);
    Path(ramp_msgs::Path p);
    ~Path();
    
    //Data members
    Configuration start_;
    Configuration goal_;
    std::vector<Configuration> all_;
    
    //Methods
    void Add(const Configuration c);
    const unsigned int size() const;
    const ramp_msgs::Path buildPathMsg() const; 
    const std::string toString() const;
};

#endif
