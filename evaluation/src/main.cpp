#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"
#include "ramp_msgs/Update.h"
#include "ramp_msgs/ObstacleList.h"
#include "tf/transform_datatypes.h"

Evaluate ev;
Utility u;


/** Srv callback to evaluate a trajectory */
bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  //std::cout<<"\nIn handling requests!\n";
  ev.setRequest(req);
  res.fitness = ev.performFitness();
  res.feasible = !ev.performCollisionDetection();
  //std::cout<<"\nfitness: "<<res.fitness<<"\n";
  //std::cout<<"\nfeasible: "<<(res.feasible ? "true" : "false")<<"\n";
  
  return true;
} //End handleRequest


/** Subscribe to the object_list topic to get the latest list information about objects, update the collision detection's obstacle list */
void obstacleListCb(const ramp_msgs::ObstacleList& ol) {
  ev.collision_.obstacleList_ = ol;
} //End objectListCb


int main(int argc, char** argv) {

  /** ***Testing*** */
  nav_msgs::Odometry odom;
  nav_msgs::Odometry odom_prev;

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0.3);

  odom_prev.pose.pose.position.x = 0;
  odom_prev.pose.pose.position.y = 0;
  odom_prev.pose.pose.position.z = 0;
  odom_prev.twist.twist.linear.x = 1;
  odom_prev.twist.twist.linear.y = 1;
  odom_prev.twist.twist.linear.z = 0;
  odom_prev.twist.twist.angular.x = 0;
  odom_prev.twist.twist.angular.y = 0;
  odom_prev.twist.twist.angular.z = 0;
  odom_prev.pose.pose.orientation = q;
  
  odom.pose.pose.position.x = 1;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.twist.twist.linear.x = 1;
  odom.twist.twist.linear.y = 1;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  odom.pose.pose.orientation = q;

  ramp_msgs::Obstacle ob;
  ob.odom_t = odom;
  ob.odom_t_prev = odom_prev;
  ros::Duration d(2); 
  ramp_msgs::Trajectory t = ev.collision_.getTrajectoryRequest(ob, d); 

  /** Testing */
  std::cout<<"\nGot trajectory t\n";
  std::cout<<"\n"<<u.toString(t)<<"\n";

  odom_prev.pose.pose.position.x = -1;
  odom_prev.pose.pose.position.y = 0;
  odom_prev.pose.pose.position.z = 0;
  odom_prev.twist.twist.linear.x = 0;
  odom_prev.twist.twist.linear.y = 1;
  odom_prev.twist.twist.linear.z = 0;
  odom_prev.twist.twist.angular.x = 0;
  odom_prev.twist.twist.angular.y = 0;
  odom_prev.twist.twist.angular.z = 0;
  odom_prev.pose.pose.orientation = q;
  
  odom.pose.pose.position.x = 1;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 1;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  odom.pose.pose.orientation = q;

  ob.odom_t = odom;
  ob.odom_t_prev = odom_prev;
  ramp_msgs::Trajectory t_2 = ev.collision_.getTrajectoryRequest(ob, d); 

  //std::cout<<"\nGot trajectory t_2\n";
  //std::cout<<"\n"<<u.toString(t_2)<<"\n";

  ev.collision_.trajectory_ = t;
  ev.collision_.obstacleList_.obstacles.push_back(ob);

  std::cout<<"\nev.collision_:"<<ev.collision_.perform();
  /** End Testing */
  
  ros::init(argc, argv, "evaluation");
  ros::NodeHandle handle;


  ros::ServiceServer service = handle.advertiseService("evaluation", handleRequest);
  ros::Subscriber sub_obj_list = handle.subscribe("object_list", 1000, obstacleListCb);

 

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
