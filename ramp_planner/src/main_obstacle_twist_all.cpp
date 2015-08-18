#include <ros/ros.h>
#include "utility.h"
#include "geometry_msgs/Twist.h"


int num_obs;
std::vector< std::string > ob_odoms;
std::vector< std::string > ob_vels;
std::vector< ros::Publisher > ob_pubs;

void getObstacleParams(const ros::NodeHandle handle)
{
  if(handle.hasParam("/ramp/num_of_obstacles"))
  {
    handle.getParam("/ramp/num_of_obstacles", num_obs);
    ROS_INFO("num_of_obstacles: %i", num_obs);
  }
  else
  {
    ROS_ERROR("Cannot find /ramp/num_of_obstacles");
  }



  if(handle.hasParam("/ramp/obstacle_odoms"))
  {
    handle.getParam("/ramp/obstacle_odoms", ob_odoms);
    ROS_INFO("ob_odoms.size(): %i", (int)ob_odoms.size());
    for(int i=0;i<ob_odoms.size();i++)
    {
      ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
    }
  }


  if(handle.hasParam("/ramp/obstacle_vels"))
  {
    handle.getParam("/ramp/obstacle_vels", ob_vels);
    ROS_INFO("ob_vels.size(): %i", (int)ob_vels.size());
    for(int i=0;i<ob_vels.size();i++)
    {
      ROS_INFO("ob_vels[%i]: %s", i, ob_vels.at(i).c_str());
    }
  }
}



void publishToAllObs(const geometry_msgs::Twist twist)
{
  for(uint8_t i=0;i<ob_pubs.size();i++)
  {
    ob_pubs.at(i).publish(twist);
  }
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "obstacle");
  ros::NodeHandle handle;

  getObstacleParams(handle);
  ROS_INFO("Obtained obstacle rosparams, please review and hit enter to continue");
  std::cin.get();
  
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>(ob_vels.at(i), 1000);
    ob_pubs.push_back(pub_twist);
  }

  Utility u;


  ros::Rate r(15);
  ros::Duration d(3.);
  geometry_msgs::Twist twist;
  
  twist.linear.x = 0.33f;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = 0.f;

  ROS_INFO("Press Enter to begin obstacle movement");

  std::cin.get();
  

  bool cc_started = false;
  while(!cc_started)
  {
    handle.getParam("/ramp/cc_started", cc_started);
    //ROS_INFO("/ramp/cc_started: %s", cc_started ? "True" : "False");
    r.sleep();
    ros::spinOnce();
  }

  ROS_INFO("Starting obstacle motion!");

  // Drive forward
  ros::Time t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    publishToAllObs(twist);
    r.sleep();
  }


  // Self-rotate
  /*twist.linear.x = 0;
  twist.angular.z = 0.44;

  d = ros::Duration(1.5);
  t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    pub_twist.publish(twist);

    r.sleep();
  }*/
  


  // Translate+rotate
  twist.linear.x = 0.33;
  twist.angular.z = -0.52;

  d = ros::Duration(1.5);
  t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    publishToAllObs(twist);
    r.sleep();
  }


  // Self rotate
  twist.linear.x = 0.33;
  twist.angular.z = 0.52;

  d = ros::Duration(2.);
  t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    publishToAllObs(twist);
    r.sleep();
  }


  // Translate+rotate
  twist.linear.x = 0.33;
  twist.angular.z = -0.52;

  d = ros::Duration(2.5);
  t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    publishToAllObs(twist);
    r.sleep();
  }


  // Translate+rotate
  twist.linear.x = 0.33;
  twist.angular.z = 0.52;

  d = ros::Duration(1.5);
  t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    publishToAllObs(twist);
    r.sleep();
  }


  // Translate+rotate
  twist.linear.x = 0.33;
  twist.angular.z = -0.52;

  d = ros::Duration(1.5);
  t = ros::Time::now();
  while(ros::Time::now() - t < d)
  {
    publishToAllObs(twist);
    r.sleep();
  }

  
  twist.linear.x = 0.;
  twist.angular.z = 0.;
  publishToAllObs(twist);
  publishToAllObs(twist);
  publishToAllObs(twist);
  publishToAllObs(twist);
  publishToAllObs(twist);


  ROS_INFO("Obstacle node done");
  return 0;
}
