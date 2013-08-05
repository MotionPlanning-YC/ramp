#ifndef PATH_H
#define PATH_H
#include "configuration.h"

class Path {
  public:

    Path();
    Path(Configuration start, Configuration goal);
    ~Path();
    
    Configuration start_;
    Configuration goal_;
    std::vector<Configuration> all_;
    
    void Add(const Configuration c);
    
    const std::string toString() const;
};

#endif
