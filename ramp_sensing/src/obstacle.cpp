#include "obstacle.h"

Obstacle::Obstacle() {
  nav_msgs::Odometry temp;
  odom_t      = temp;
  odom_t_prev = temp;
}

Obstacle::Obstacle(const nav_msgs::Odometry o) 
{
  odom_t      = o;
  odom_t_prev = o;
}

Obstacle::~Obstacle() {}


void Obstacle::update(const nav_msgs::Odometry o) 
{

  //Set new odometry infomation
  odom_t_prev = odom_t;
  odom_t      = o;

  //Update time
  last_updated_ = ros::Time::now();
}


const ramp_msgs::Obstacle Obstacle::buildObstacleMsg() {

  ramp_msgs::Obstacle result;

  result.odom_t      = odom_t;
  result.odom_t_prev = odom_t_prev;
  
  return result;
}
