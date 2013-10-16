#ifndef DYNAMIC_OBJECT_H
#define DYNAMIC_OBJECT_H
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Object.h"
#include <vector>

class DynamicObject {
  public:
    DynamicObject();
    DynamicObject(nav_msgs::Odometry p);
    ~DynamicObject(); 

    //Index 0: odom_t-1
    //Index 1: odom_t
    std::vector<nav_msgs::Odometry> odom_;

    //Time of last update
    ros::Time last_updated_;
    
    void update(nav_msgs::Odometry o);
    ramp_msgs::Object buildObjectMsg();

  private:
};

#endif
