#include "control_handler.h"

ControlHandler::ControlHandler(const ros::NodeHandle& h) : handle_(h) {
  pub_bestTrajec_ = handle_.advertise<ramp_msgs::RampTrajectory>("bestTrajec", 1000);
  pub_population_ = handle_.advertise<ramp_msgs::Population>("population", 1000);
  
  pub_imminent_collision_ = handle_.advertise<std_msgs::Bool>("imminent_collision", 1000);
}


void ControlHandler::send(ramp_msgs::RampTrajectory bestTrajec) 
{
  pub_bestTrajec_.publish(bestTrajec);
}

void ControlHandler::sendPopulation(ramp_msgs::Population population) 
{
  pub_population_.publish(population);
}


void ControlHandler::sendIC(std_msgs::Bool value)
{
  pub_imminent_collision_.publish(value);
}
