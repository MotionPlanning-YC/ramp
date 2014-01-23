#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"
#include "ramp_msgs/Update.h"
#include "ramp_msgs/ObstacleList.h"
#include "tf/transform_datatypes.h"

Evaluate ev;
CollisionDetection cd;
Utility u;


/** Srv callback to evaluate a trajectory */
bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  //std::cout<<"\nIn handleRequest!\n";
  ev.setRequest(req);
  
  // Do collision detection
  cd.trajectory_  = req.trajectory;
  CollisionDetection::QueryResult qr = cd.perform();
  res.feasible = !qr.collision_;

  // Do fitness
  res.fitness = ev.performFitness(qr);

  //std::cout<<"\nfitness: "<<res.fitness<<"\n";
  //std::cout<<"\nfeasible: "<<(res.feasible ? "true" : "false")<<"\n";
  
  return true;
} //End handleRequest


/** Subscribe to the object_list topic to get the latest list information about objects, update the collision detection's obstacle list */
//void obstacleListCb(const ramp_msgs::ObstacleList& ol) {
void obstacleCb(const ramp_msgs::Obstacle& ol) {
  cd.obstacle_ = ol;
} //End objectListCb


int main(int argc, char** argv) {

  ros::init(argc, argv, "evaluation");
  ros::NodeHandle handle;

  int id;
  handle.getParam("evaluation/robot_id", id);
  cd.id = id;
  std::cout<<"\nid: "<<cd.id;
  cd.init(handle, id);

  std::cout<<"\nAfter init\n";
  
  ros::ServiceServer service    = handle.advertiseService("evaluation", handleRequest);
  ros::Subscriber sub_obj_list  = handle.subscribe("object_list", 1000, obstacleCb);

  /** ***Testing*** */
  nav_msgs::Odometry odom;
  nav_msgs::Odometry odom_prev;

  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);

  odom_prev.pose.pose.position.x = 1;
  odom_prev.pose.pose.position.y = 2.f;
  odom_prev.pose.pose.position.z = 0;
  odom_prev.twist.twist.linear.x = 0.33;
  odom_prev.twist.twist.linear.y = 0;
  odom_prev.twist.twist.linear.z = 0;
  odom_prev.twist.twist.angular.x = 0;
  odom_prev.twist.twist.angular.y = 0;
  odom_prev.twist.twist.angular.z = 0;
  odom_prev.pose.pose.orientation = q;
  
  odom.pose.pose.position.x = 1.06;
  odom.pose.pose.position.y = 2.f;
  odom.pose.pose.position.z = 0;
  odom.twist.twist.linear.x = 0.33;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  odom.pose.pose.orientation = q;

  ramp_msgs::Obstacle ob;
  ob.odom_t = odom;
  ob.odom_t_prev = odom_prev;

  //ros::Duration d(10); 
  //ramp_msgs::Trajectory t = cd.getPredictedTrajectory(ob, d); 

  //std::cout<<"\nGot trajectory t\n";
  //std::cout<<"\n"<<u.toString(t)<<"\n";

  /*q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, PI);
  
  odom_prev.pose.pose.position.x = 3;
  odom_prev.pose.pose.position.y = 1.75f;
  odom_prev.pose.pose.position.z = 0;
  odom_prev.twist.twist.linear.x = -0.33;
  odom_prev.twist.twist.linear.y = 0;
  odom_prev.twist.twist.linear.z = 0;
  odom_prev.twist.twist.angular.x = 0;
  odom_prev.twist.twist.angular.y = 0;
  odom_prev.twist.twist.angular.z = 0;
  odom_prev.pose.pose.orientation = q;
  
  odom.pose.pose.position.x = 2.94;
  odom.pose.pose.position.y = 0;
  odom.pose.pose.position.z = 0;
  odom.twist.twist.linear.x = -0.33;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  odom.pose.pose.orientation = q;*/

  //ob.odom_t = odom;
  //ob.odom_t_prev = odom_prev;
  //ros::Duration d2(12);
  //ramp_msgs::Trajectory t_2 = cd.getPredictedTrajectory(ob, d2); 

  //std::cout<<"\nGot trajectory t_2\n";
  //std::cout<<"\n"<<u.toString(t_2)<<"\n";

  //cd.trajectory_ = t;
  //cd.obstacleList_.obstacles.push_back(ob);

  //std::cout<<"\nev.collision_:"<<cd.perform();
  /** End Testing */


  std::vector<float> p;
  p.push_back(0);
  p.push_back(2.f);
  p.push_back(0);
  std::vector<float> c = cd.getCenter(p, 0);
  std::cout<<"\ncenter: ["<<c.at(0)<<", "<<c.at(1)<<"]";


 
  /*std::cout<<"\n"<<u.displaceAngle(PI/2, 5*PI/4);
  std::cout<<"\n"<<u.displaceAngle(PI/2, 6*PI/4);
  std::cout<<"\n"<<u.displaceAngle(PI/2, 7*PI/4);*/

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
