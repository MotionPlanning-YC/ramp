#ifndef KNOT_POINT_H
#define KNOT_POINT_H
#include "configuration.h"
#include "utility.h"

class KnotPoint {
  public:

    KnotPoint() {}
    ~KnotPoint() {}

    Configuration configuration_;
    unsigned int stop_time_;

  private:
};

#endif
