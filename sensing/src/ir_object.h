#ifndef IR_OBJECT_H
#define IR_OBJECT_H
#include "corobot_msgs/SensorMsg.h"
#include "ramp_msgs/Configuration.h"

class IrObject {
  public:
    IrObject();
    IrObject(const corobot_msgs::SensorMsg msg, const ramp_msgs::Configuration current);
    ~IrObject() {}

    void build(const corobot_msgs::SensorMsg msg, const ramp_msgs::Configuration current);

    float x1;
    float x2;
    float y1;
    float y2;
  private:

    const float OBSTACLE_SIZE;
    
};

#endif
