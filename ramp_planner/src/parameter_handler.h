#ifndef PARAMETER_HANDLER_H
#define PARAMETER_HANDLER_H
#include "ros/ros.h"


class ParameterHandler {
  public:
    ParameterHandler(const ros::NodeHandle& h);

    bool set(const std::string name, 

  private:
    ros::NodeHandle  handle_;
};

#endif
