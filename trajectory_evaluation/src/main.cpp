#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Obstacle.h"

Evaluate ev;
CollisionDetection cd;
Utility u;
bool received_ob = false;


/** Srv callback to evaluate a trajectory */
bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{
  ROS_INFO("Robot %i Evaluating trajectory: %s", cd.id_, u.toString(req.trajectory).c_str());

  ev.setRequest(req);
  
  // Make a QueryResult object
  CollisionDetection::QueryResult qr;

  // If there are obstacles, do collision detection
  if(received_ob) {
    cd.trajectory_  = req.trajectory;
    qr = cd.perform();
  }
  // Else, set collision to false
  else {
    qr.collision_ = 0;
    qr.t_firstCollision_ = 9999.0f; 
  }

  // Set response
  res.feasible = !qr.collision_;
  res.t_firstCollision = ros::Duration(qr.t_firstCollision_);

  // Do fitness
  res.fitness = ev.performFitness(qr);

  ROS_INFO("Robot %i Done evaluating, fitness: %f feasible: %s t_firstCollision: %f", cd.id_, res.fitness, res.feasible ?  
      "True" : "False", res.t_firstCollision.toSec());
  return true;
} //End handleRequest


/** Subscribe to the object_list topic to get the latest list information about objects, update the collision detection's obstacle list */
void obstacleCb(const ramp_msgs::Obstacle& ol) 
{
  ROS_INFO("Received Obstacle Info");
  cd.obstacle_ = ol;
  received_ob = true;
} //End objectCb


/*void temp_odomCb(const nav_msgs::Odometry& msg) {
  ramp_msgs::Obstacle ob;
  ob.odom_t     = msg;
  cd.obstacle_  = ob;

  received_ob = true;
}*/


int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_evaluation");
  ros::NodeHandle handle;

  int id;
  handle.getParam("trajectory_evaluation/robot_id", cd.id_);
  std::cout<<"\nTrajectory Evaluation id: "<<cd.id_;
  cd.init(handle);

 
  ros::ServiceServer service    = handle.advertiseService("trajectory_evaluation", handleRequest);
  ros::Subscriber sub_obj_list  = handle.subscribe("object_list", 1000, obstacleCb);
  //ros::Subscriber sub_ob_odom  = handle.subscribe("odom", 1000, temp_odomCb);

  //cd.pub_population = handle.advertise<ramp_msgs::Population>("/robot_1/population", 1000);

  /** ***Testing*** */

  // Make trajectory 1
  /*ramp_msgs::KnotPoint kp1;
  ramp_msgs::Configuration c1;
  c1.K.push_back(0);
  c1.K.push_back(2);
  c1.K.push_back(0);
  c1.ranges = u.ranges_;
  kp1.configuration = c1;
  kp1.stop_time = 0;
 
  ramp_msgs::KnotPoint kp2;
  ramp_msgs::Configuration c2;
  c2.K.push_back(1);
  c2.K.push_back(2);
  c2.K.push_back(0);
  c2.ranges = u.ranges_;
  kp2.configuration = c2;
  kp2.stop_time = 0;

  ramp_msgs::Path p1;
  p1.points.push_back(kp1);
  p1.points.push_back(kp2);

  ramp_msgs::TrajectoryRequest tr;
  tr.request.path = p1;
  tr.request.v_start.push_back(0.33f);
  tr.request.v_end.push_back(0.33f);
  tr.request.resolutionRate = 5;

  cd.h_traj_req_->request(tr);
  ramp_msgs::Trajectory t1 = tr.response.trajectory;
 
  std::cout<<"\nt1: "<<u.toString(t1);

  //ramp_msgs::Trajectory t1_ob = cd.transformT_ob(t1);
  //std::cout<<"\nt1_ob: "<<u.toString(t1_ob);
  

  // Make trajectory 2 - in odometry space
  ramp_msgs::KnotPoint kp3;
  ramp_msgs::Configuration c3;
  c3.K.push_back(0);
  c3.K.push_back(0.f);
  c3.K.push_back(0);
  c3.ranges = u.ranges_;
  kp3.configuration = c3;
  kp3.stop_time = 0;
  
  ramp_msgs::KnotPoint kp4;
  ramp_msgs::Configuration c4;
  c4.K.push_back(0.f);
  c4.K.push_back(1.f);
  c4.K.push_back(0.f);
  c4.ranges = u.ranges_;
  kp4.configuration = c4;
  kp4.stop_time = 0;

  ramp_msgs::Path p2;
  p2.points.push_back(kp3);
  p2.points.push_back(kp4);

  tr.request.path = p2;

  cd.h_traj_req_->request(tr);
  ramp_msgs::Trajectory t2 = tr.response.trajectory;

  //std::cout<<"\nt2: "<<u.toString(t2);


  // Do collision detection against two trajectories
  cd.trajectory_ = t1;
  CollisionDetection::QueryResult qr = cd.query(t2);
  std::cout<<"\nqr.collision: "<<qr.collision_;
  std::cout<<"\nqr.i_obstacle: "<<qr.i_obstacle;
  std::cout<<"\nqr.t_firstCollision: "<<qr.t_firstCollision_;
  t1.feasible = !qr.collision_;*/
  //t1.feasible = 1;
  


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


  ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();


  std::cout<<"\nExiting Normally\n";
  return 0;
}
