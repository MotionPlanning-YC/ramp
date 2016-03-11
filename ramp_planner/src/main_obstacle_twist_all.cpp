#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "corobot_msgs/MotorCommand.h"


int num_obs;
std::vector< std::string > ob_odoms;
std::vector< std::string > ob_vels;
std::vector< ros::Publisher > ob_pubs;
std::vector< double > ob_delays;
std::vector< ros::Duration > dur_delays;
std::vector< ros::Timer > ob_timers;
ros::Time node_start;

ros::Publisher corobot_pub;

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
    //for(int i=0;i<ob_odoms.size();i++)
    for(int i=0;i<num_obs;i++)
    {
      ROS_INFO("ob_odoms[%i]: %s", i, ob_odoms.at(i).c_str());
    }
  }


  if(handle.hasParam("/ramp/obstacle_vels"))
  {
    handle.getParam("/ramp/obstacle_vels", ob_vels);
    ROS_INFO("ob_vels.size(): %i", (int)ob_vels.size());
    //for(int i=0;i<ob_vels.size();i++)
    for(int i=0;i<num_obs;i++)
    {
      ROS_INFO("ob_vels[%i]: %s", i, ob_vels.at(i).c_str());
    }
  }


  if(handle.hasParam("/ramp/obstacle_delays"))
  {
    handle.getParam("/ramp/obstacle_delays", ob_delays);

    //for(int i=0;i<ob_delays.size();i++)
    for(int i=0;i<num_obs;i++)
    {
      ROS_INFO("ob_delay[%i]: %f", i, ob_delays.at(i));
      dur_delays.push_back(ros::Duration(ob_delays.at(i)));
    }
  }
}




void turn(const int index, const double v, const double w, const double t)
{
  ros::Rate r(15);
  ros::Duration d(t);
  geometry_msgs::Twist twist;
  
  twist.linear.x = v;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = w;
 
  // Drive
  ros::Time start = ros::Time::now();
  while(ros::Time::now() - start < d)
  {
    ob_pubs.at(index).publish(twist);
    r.sleep();
  } // end while
}




void SLike(const int index, const double v, const double w, const double t)
{
  double t_each = t/4.f;

  turn(index, v, w, t_each);
  turn(index, v, -w, t_each);
  turn(index, v, w, t_each);
  turn(index, v, -w, t_each);
}


void driveStraight(const int index, const double v, const double t)
{
  //ROS_INFO("In driveStraight");

  ros::Rate r(25);
  ros::Duration d(t);
  geometry_msgs::Twist twist;
  
  twist.linear.x = v;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = 0.f;
 
  // Drive forward
  ros::Time start = ros::Time::now();
  while(ros::ok() && ros::Time::now() - start < d)
  {
    ob_pubs.at(index).publish(twist);
    r.sleep();
  } // end while
}


void publishToOb(const ros::TimerEvent e, const int index)
{
  ROS_INFO("index: %i", index);
  ROS_INFO("Elapsed time: %f", (ros::Time::now() - node_start).toSec());


  ros::Rate r(10);
  ros::Duration d(1.5);
  geometry_msgs::Twist twist;
  
  twist.linear.x = 0.28f;
  twist.linear.y = 0.f;
  twist.linear.z = 0.f;
  twist.angular.x = 0.f;
  twist.angular.y = 0.f;
  twist.angular.z = 0.f;

  /*
   * Set motion for Obstacle 1
   */
  if(index == 0)
  {
    
    
    driveStraight(index, 0.33, 5);
    
    //d = ros::Duration(4);

    //twist.angular.z = (index == 1) ? 0.64 : 0.8;
    // Drive forward
    /*ros::Time t = ros::Time::now();
    while(ros::Time::now() - t < d)
    {
      //ob_pubs.at(index).publish(twist);
      r.sleep();
    } // end while*/
  } // end Obstacle 1


  else
  {
 
    // Drive forward
    ros::Time t = ros::Time::now();
    while(ros::ok() && ros::Time::now() - t < d)
    {
      ob_pubs.at(index).publish(twist);
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
    //twist.angular.z = -0.68;

    d = ros::Duration(1.75);
    t = ros::Time::now();
    while(ros::ok() && ros::Time::now() - t < d)
    {
      twist.angular.z = (index == 1) ? -0.64 : -0.8;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    // Self rotate
    twist.linear.x = 0.28;
    //twist.angular.z = 0.8;

    d = ros::Duration(2.5);
    t = ros::Time::now();
    while(ros::ok() && ros::Time::now() - t < d)
    {
      twist.angular.z = (index == 1) ? 0.64 : 0.8;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    // Self rotate
    twist.linear.x = 0.28;
    //twist.angular.z = 0.8;

    d = ros::Duration(3);
    t = ros::Time::now();
    while(ros::ok() && ros::Time::now() - t < d)
    {
      twist.angular.z = (index == 1) ? -0.8 : -0.64;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }


    // Self rotate
    twist.linear.x = 0.28;
    //twist.angular.z = 0.8;

    d = ros::Duration(2.25);
    t = ros::Time::now();
    while(ros::ok() && ros::Time::now() - t < d)
    {
      twist.angular.z = (index == 1) ? 0.64 : 0.8;
      ob_pubs.at(index).publish(twist);
      r.sleep();
    }

  } // end if turtlebot

  
  // Stop
  twist.linear.x = 0.;
  twist.angular.z = 0.;
  for(int i=0;i<10;i++)
  {
    ob_pubs.at(index).publish(twist);
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
  ros::Rate r(5);
  ros::Duration d(3.);
  geometry_msgs::Twist twist;


  getObstacleParams(handle);
  ROS_INFO("Obtained obstacle rosparams, please review and hit enter to continue");
  std::cin.get();
  
  // Create publishers
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    ros::Publisher pub_twist = handle.advertise<geometry_msgs::Twist>(ob_vels.at(i), 10);
    ob_pubs.push_back(pub_twist);
  }
  corobot_pub = handle.advertise<corobot_msgs::MotorCommand>("PhidgetMotor", 10);

  ROS_INFO("Waiting for /ramp/cc_started=true...");

  // Wait for ramp to start moving the robot
  bool cc_started = false;
  while(!cc_started)
  {
    handle.getParam("/ramp/cc_started", cc_started);
    //ROS_INFO("/ramp/cc_started: %s", cc_started ? "True" : "False");
    r.sleep();
    ros::spinOnce();
  }

  //ROS_INFO("Press Enter to begin obstacle movement");

  //std::cin.get();

  node_start = ros::Time::now();
 

  // Start timers
  for(uint8_t i=0;i<ob_odoms.size();i++)
  {
    ros::Timer temp = handle.createTimer(ros::Duration(ob_delays.at(i)), boost::bind(publishToOb, _1, i), true, true);
    ob_timers.push_back(temp);
  }

  

  ROS_INFO("Starting obstacle motion!");



  ROS_INFO("Obstacle node done");
  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
