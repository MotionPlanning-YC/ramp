#ifndef COLLISION_DETECTION
#define COLLISION_DETECTION
#include "utility.h"


class CollisionDetection {
  public:
    CollisionDetection() {}

    const bool perform() const;

    ramp_msgs::Trajectory trajectory_;
    std::vector<obstacle_struct> obstacle_list;
};

#endif
