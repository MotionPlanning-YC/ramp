#ifndef KNOT_POINT_H
#define KNOT_POINT_H
#include "ramp_msgs/KnotPoint.h"
#include "configuration.h"
#include "utility.h"

class KnotPoint {
  public:

    KnotPoint(); 
    KnotPoint(const Configuration c);
    KnotPoint(const ramp_msgs::KnotPoint kp);
    ~KnotPoint() {}

    // Data Members
    Configuration configuration_;
    unsigned int stop_time_;

    // Methods
    const ramp_msgs::KnotPoint buildKnotPointMsg() const;
    const std::string toString() const;
    
  private:
};

#endif
