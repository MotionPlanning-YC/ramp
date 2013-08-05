#ifndef RANGE_H
#define RANGE_H
#include "utility.h"

class Range {
  public:
    Range();
    Range(const float min, const float max);
    ~Range();

    /** This method returns a random value in the range */
    const float random();
    
    float min_;
    float max_;
};

#endif
