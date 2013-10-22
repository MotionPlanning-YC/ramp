#ifndef OBSTACLE_H
#define OBSTACLE_H
#include "nav_msgs/Odometry.h"
#include "ramp_msgs/Obstacle.h"
#include <vector>

class Obstacle {
  public:
    Obstacle();
    Obstacle(const nav_msgs::Odometry p);
    ~Obstacle(); 

    /** Data Members */

    //Hold odometry information for t and t-1
    nav_msgs::Odometry odom_t;
    nav_msgs::Odometry odom_t_prev;

    //Time of last update
    ros::Time last_updated_;
    

    /** Methods */

    void update(const nav_msgs::Odometry o);
    const ramp_msgs::Obstacle buildObstacleMsg();

  private:
};

#endif
