#include "range.h"

Range::Range() {}

Range::Range(const float min, const float max) : min_(min), max_(max) {}

Range::Range(ramp_msgs::Range r) {
  min_ = r.min;
  max_ = r.max;
}

Range::~Range() {}


const float Range::random() {
  if(min_ == 0 && max_ == 0)
    return 0;

  return ( min_ + (float)rand() / ((float)RAND_MAX / (max_ - min_)) ); 
}


const ramp_msgs::Range Range::buildRangeMsg() const {
  ramp_msgs::Range result;
  
  result.min = min_;
  result.max = max_;

  return result;
}
