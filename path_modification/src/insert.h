#ifndef INSERT_H
#define INSERT_H
#include "utility.h"
#include "ramp_msgs/Path.h"

class Insert {
  public:
    Insert() {}
    Insert(const ramp_msgs::Path p); 

    const ramp_msgs::Path perform();

    ramp_msgs::Path path_;
  private:
    Utility utility;
};

#endif
