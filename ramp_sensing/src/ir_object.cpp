#include "ir_object.h"

IrObject::IrObject() : OBSTACLE_SIZE(0.3) {}

IrObject::IrObject(const corobot_msgs::SensorMsg msg, const ramp_msgs::Configuration current) : OBSTACLE_SIZE(0.3){
  build(msg, current);
}

void IrObject::build(const corobot_msgs::SensorMsg msg, const ramp_msgs::Configuration current) {

  if (current.K.size() >=3)
  {
      double theta = current.K.at(2); 
      
      x1 = current.K.at(0) + ( (msg.value) * cos(theta)) - ( (OBSTACLE_SIZE/2) * sin(theta) );
      x2 = x1 + OBSTACLE_SIZE;
      
      y1 = current.K.at(1) + ( (msg.value) * sin(theta)) - ( (OBSTACLE_SIZE/2) * cos(theta) );
      y2 = y1 + OBSTACLE_SIZE;
  }

}

const ramp_msgs::IRObject IrObject::buildIRObjectMsg() const {
  ramp_msgs::IRObject result;
  result.x1 = x1;
  result.x2 = x2;
  result.y1 = y1;
  result.y2 = y2;
  return result;
}
