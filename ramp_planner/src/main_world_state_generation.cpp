#include "ros/ros.h"
#include "utility.h"
#include "knot_point.h"
#include "ramp_msgs/ObstacleList.h"
#include "nav_msgs/Odometry.h"
#include "range.h"
#include "ramp_trajectory.h"
#include "ramp_msgs/TrajectorySrv.h"
#include "tf/transform_datatypes.h"



/*
 * p_x and p_y are the position values,                 range: [0, 3.5]
 * v_mag is the magnitude of the linear velocity,       range: [0, 0.25]
 * v_direction is the direction of the linear velocity, range: [0, pi]
 * w is the angular velocity (not a vector),            range: [-pi/4, pi/4]
 */
const ramp_msgs::Obstacle buildObstacleMsg(const double& p_x, const double& p_y, const double& v_mag, const double& v_direction, const double& w)
{
  ROS_INFO("p_x: %f p_y: %f, v_mag: %f v_direction: %f w: %f", p_x, p_y, v_mag, v_direction, w);

  ramp_msgs::Obstacle result;

  // odom_msg describes the obstacle's position and velocity
  nav_msgs::Odometry odom_msg;

  // Set the x,y position
  odom_msg.pose.pose.position.x = p_x;
  odom_msg.pose.pose.position.y = p_y; 
  odom_msg.pose.pose.position.z = 0;

  // Set orientation
  odom_msg.pose.pose.orientation  = tf::createQuaternionMsgFromYaw(v_direction);
  
  // For linear velocity, calculate x and y components
  double v_x = v_mag*cos(v_direction);
  double v_y = v_mag*sin(v_direction);

  // Set velocities
  odom_msg.twist.twist.linear.x   = v_x;
  odom_msg.twist.twist.linear.y   = v_y;
  odom_msg.twist.twist.angular.z  = w;

  // Set odom_msg and return result
  result.odom_t = odom_msg;
  return result;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "generate_world_state");
  ros::NodeHandle handle;
  Utility utility;

  srand(time(NULL));

  // Set the number of obstacles (given by command line argument)
  const int NUM_OBSTACLES = argc > 1 ? atoi(argv[1]) : 1;
  ROS_INFO("NUM_OBSTACLES: %i", NUM_OBSTACLES);
    

  // Create Ranges for each dimension to generate random values
  Range x_pos(0, 3.5);
  Range y_pos(0, 3.5);
  Range v_mag(0, 0.33);
  Range v_direction(0, PI);
  Range w(-PI/4.f, PI/4.f);


  /*
   * Populate an ObstacleList object
   * Create x number of random obstacles, x=NUM_OBSTACLES
   */
  ramp_msgs::ObstacleList obs;
  for(uint8_t i=0;i<NUM_OBSTACLES;i++)
  {
    // Build Obstacle msg
    ramp_msgs::Obstacle ob = buildObstacleMsg( x_pos.random(), y_pos.random(), v_mag.random(), v_direction.random(), w.random() );

    // Push new Obstacle msg onto ObstacleList obstacles vector
    obs.obstacles.push_back(ob); 
  }

  ROS_INFO("obs.obstacles.size(): %i", (int)obs.obstacles.size());

  // Create Publisher for the ObstacleList
  ros::Publisher pub_obs = handle.advertise<ramp_msgs::ObstacleList>("obstacles", 100);

  // Sleep briefly
  ros::Duration d(0.5f);
  d.sleep();

  // Publish the list of obstacles
  pub_obs.publish(obs);

  ROS_INFO("Exiting normally");
  return 0;
}
