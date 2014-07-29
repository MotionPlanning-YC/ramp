#ifndef PATH_H
#define PATH_H
#include "knot_point.h"
#include "ramp_msgs/Path.h"

class Path {
  public:

    Path();
    Path(const KnotPoint start, const KnotPoint goal);
    Path(const MotionState start, const MotionState goal);
    Path(const std::vector<KnotPoint> all);
    Path(const ramp_msgs::Path p);
    ~Path();
    
    //Data members
    KnotPoint start_;
    KnotPoint goal_;
    std::vector<KnotPoint> all_;
    
    
    //Methods
    void Add(const KnotPoint kp);
    void Add(const MotionState kp);
    const unsigned int size() const;
    const ramp_msgs::Path buildPathMsg() const; 
    const std::string toString() const;
};

#endif
