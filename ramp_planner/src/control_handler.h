#ifndef CONTROL_HANDLER_H
#define CONTROL_HANDLER_H
#include "ros/ros.h"
#include "ramp_msgs/Trajectory.h"
#include "ramp_msgs/Population.h"

class ControlHandler {
  public:
    ControlHandler(const ros::NodeHandle& h);

    void send(ramp_msgs::Trajectory bestTrajec);
    void sendPopulation(ramp_msgs::Population population);

  private:
    ros::NodeHandle handle_;
    ros::Publisher pub_bestTrajec_;
    ros::Publisher pub_population_;
};

#endif
