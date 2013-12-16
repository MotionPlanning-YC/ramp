#include "utility.h"


Utility::Utility() {
  ramp_msgs::Range range0; 
  range0.min = 0;
  range0.max = 3.5;

  ramp_msgs::Range range1; 
  range1.min = 0;
  range1.max = 3.5;

  ramp_msgs::Range range2; 
  range2.min = 0;
  range2.min = 0;

  standardRanges.push_back(range0);
  standardRanges.push_back(range1);
  standardRanges.push_back(range2);
}
