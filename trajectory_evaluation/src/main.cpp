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
  //cd.trajectory_  = req.trajectory;
  //CollisionDetection::QueryResult qr = cd.perform();
  //std::cout<<"\nqr.collision_: "<<qr.collision_;
  CollisionDetection::QueryResult qr;
  qr.collision_ = 0;
  
  // Set response
  res.feasible = !qr.collision_;
  res.time_until_collision = qr.time_until_collision_;

  // Do fitness
  res.fitness = ev.performFitness(qr);
  
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
  cd.init(handle);

  std::cout<<"\nAfter init\n";
  
  ros::ServiceServer service    = handle.advertiseService("evaluation", handleRequest);
  ros::Subscriber sub_obj_list  = handle.subscribe("object_list", 1000, obstacleCb);

  /** ***Testing*** */

  // Make trajectory 1
  /*ramp_msgs::Configuration c1;
  c1.K.push_back(0);
  c1.K.push_back(2);
  c1.K.push_back(0.3218f);
  c1.ranges = u.ranges_;
  
  ramp_msgs::Configuration c2;
  c2.K.push_back(3);
  c2.K.push_back(3);
  c2.K.push_back(0.3218f);
  c2.ranges = u.ranges_;

  ramp_msgs::Path p1;
  p1.configurations.push_back(c1);
  p1.configurations.push_back(c2);

  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p1;
  tr.request.v_start.push_back(0.33f);
  tr.request.v_end.push_back(0.33f);
  tr.request.resolutionRate = 5;

  cd.h_traj_req_->request(tr);
  ramp_msgs::Trajectory t1 = tr.response.trajectory;


  // Make trajectory 2 - in odometry space
  ramp_msgs::Configuration c3;
  c3.K.push_back(0);
  c3.K.push_back(0.f);
  c3.K.push_back(0);
  c3.ranges = u.ranges_;
  
  ramp_msgs::Configuration c4;
  c4.K.push_back(-3);
  c4.K.push_back(1);
  c4.K.push_back(-0.3218);
  c4.ranges = u.ranges_;

  ramp_msgs::Path p2;
  p2.configurations.push_back(c3);
  p2.configurations.push_back(c4);

  tr.request.path = p2;

  cd.h_traj_req_->request(tr);
  ramp_msgs::Trajectory t2 = tr.response.trajectory;

  std::cout<<"\nt1: "<<u.toString(t1);
  std::cout<<"\nt2: "<<u.toString(cd.transformT(t2));


  // Do collision detection against two trajectories
  cd.trajectory_ = t1;
  CollisionDetection::QueryResult qr = cd.query(t2);
  std::cout<<"\nqr.collision: "<<qr.collision_;
  std::cout<<"\nqr.i_obstacle: "<<qr.i_obstacle;
  std::cout<<"\nqr.time_until_collision: "<<qr.time_until_collision_;
  t1.feasible = !qr.collision_;
  //t1.feasible = 1;*/
  


  // Publish two trajectories to trajectory_visualization
  //ros::Publisher p_t1 = handle.advertise<ramp_msgs::Population>("/population1", 1000);
  //ros::Publisher p_t2 = handle.advertise<ramp_msgs::Population>("/population2", 1000);

  /*ramp_msgs::Population pop1;
  pop1.population.push_back(t1);
  pop1.best_id = 0;
  pop1.robot_id = 1;


  ramp_msgs::Population pop2;
  pop2.population.push_back(cd.transformT(t2));
  pop2.best_id = 0;
  pop2.robot_id = 2;
  
  ros::Rate r(1);
  while(ros::ok()) {
    p_t1.publish(pop1);
    p_t2.publish(pop2);
    ros::spinOnce();
    r.sleep();
  }
  std::cout<<"\nDone publishing!\n";*/









  /*nav_msgs::Odometry odom;
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
  ob.odom_t_prev = odom_prev;*/

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


  /*std::vector<float> p;
  p.push_back(3);
  p.push_back(1.75f);
  p.push_back(0);
  std::vector<float> c = cd.getCenter(p, PI/6);
  std::cout<<"\ncenter: ["<<c.at(0)<<", "<<c.at(1)<<"]";*/


 
  /*std::cout<<"\n"<<u.displaceAngle(PI/2, 5*PI/4);
  std::cout<<"\n"<<u.displaceAngle(PI/2, 6*PI/4);
  std::cout<<"\n"<<u.displaceAngle(PI/2, 7*PI/4);*/

  std::cout<<"\nSpinning...\n";
  ros::spin();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
