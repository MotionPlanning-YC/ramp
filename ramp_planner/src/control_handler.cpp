#include "control_handler.h"

ControlHandler::ControlHandler(const ros::NodeHandle& h) : handle_(h) {
  pub_bestTrajec_ = handle_.advertise<ramp_msgs::Trajectory>("bestTrajec", 1000);
}


void ControlHandler::send(ramp_msgs::Trajectory bestTrajec) {
  pub_bestTrajec_.publish(bestTrajec);
}
