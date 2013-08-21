#ifndef CHANGE_H
#define CHANGE_H
#include "ramp_msgs/Path.h"
#include "utility.h"

class Change {
  public:
    Change() {}
    Change(const ramp_msgs::Path p);

    const ramp_msgs::Path perform();

    ramp_msgs::Path path_;

  private:

};

#endif
