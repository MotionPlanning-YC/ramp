#include "obstacle.h"

Obstacle::Obstacle() {
  nav_msgs::Odometry temp;
  temp.pose.pose.orientation.w = 1.;
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
  /*ROS_INFO("Obstacle odometry passed in:\nPosition: (%f, %f, %f)", 
      o.pose.pose.position.x, 
      o.pose.pose.position.y, 
      tf::getYaw(o.pose.pose.orientation));*/

  //Set new odometry infomation
  odom_t_prev = odom_t;
  odom_t      = o;

  //Update time
  last_updated_ = ros::Time::now();
}


const ramp_msgs::Obstacle Obstacle::buildObstacleMsg() 
{
  ramp_msgs::Obstacle result;

  

  result.odom_t      = odom_t;
  result.odom_t_prev = odom_t_prev;
  
  /*ROS_INFO("Building Obstacle msg:\nPosition: (%f, %f, %f)", 
      result.odom_t.pose.pose.position.x, 
      result.odom_t.pose.pose.position.y, 
      tf::getYaw(result.odom_t.pose.pose.orientation));*/
  
  return result;
}
