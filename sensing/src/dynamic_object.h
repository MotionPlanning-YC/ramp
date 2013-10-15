#ifndef DYNAMIC_OBJECT_H
#define DYNAMIC_OBJECT_H
#include "geometry_msgs/Pose.h"
#include "ramp_msgs/Object.h"
#include <vector>

class DynamicObject {
  public:
    DynamicObject();
    DynamicObject(geometry_msgs::Pose p);
    ~DynamicObject(); 

    geometry_msgs::Pose current_pose_;
    ros::Time last_updated_;
    std::vector<float> trajectory_;
    
    void updatePose(geometry_msgs::Pose p, ros::Time t);
    ramp_msgs::Object buildObjectMsg();
};

#endif
