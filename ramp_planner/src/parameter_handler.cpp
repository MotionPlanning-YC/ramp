#include "parameter_handler.h"

ParameterHandler::ParameterHandler() {}


void ParameterHandler::setImminentCollision(bool ic) { 
  ros::param::set("imminent_collsion", ic);
}
