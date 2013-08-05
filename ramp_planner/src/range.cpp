#include "range.h"

Range::Range() {}

Range::Range(const float min, const float max) : min_(min), max_(max) {}

Range::~Range() {}


const float Range::random() {
 return ( min_ + (float)rand() / ((float)RAND_MAX / (max_ - min_)) ); 
}
