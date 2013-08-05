
#include "base_configuration.h"


BaseConfiguration::BaseConfiguration() {}

BaseConfiguration::BaseConfiguration(const double x, const double y, const double theta) {
  this->x = x;
  this->y = y;
  this->theta = theta;
}



/** Sets the members based on the pose */
BaseConfiguration::BaseConfiguration(const geometry_msgs::Pose pose) {
  x = pose.position.x;
  y = pose.position.y;

  theta = getThetaFromQuat(pose.orientation) * 180 / M_PI;
}

BaseConfiguration::~BaseConfiguration() {}


const double BaseConfiguration::getThetaFromQuat(geometry_msgs::Quaternion quat) {

  return tf::getYaw(quat);

}
