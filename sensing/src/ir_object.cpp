#include "ir_object.h"

IrObject::IrObject() : OBSTACLE_SIZE(0.3) {}

IrObject::IrObject(const corobot_msgs::SensorMsg msg, const ramp_msgs::Configuration current) : OBSTACLE_SIZE(0.3){
  build(msg, current);
}

void IrObject::build(const corobot_msgs::SensorMsg msg, const ramp_msgs::Configuration current) {

  x1 = current.K.at(0) + msg.value;
  x2 = x1 + OBSTACLE_SIZE;
  y1 = current.K.at(1) - (OBSTACLE_SIZE/2.0f);
  y2 = y1 + OBSTACLE_SIZE;

}
