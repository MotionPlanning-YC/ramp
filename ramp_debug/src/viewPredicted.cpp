#include "ros/ros.h"
#include <iostream>
#include "utility.h"
#include "ramp_msgs/Obstacle.h"
#include "ramp_msgs/Population.h"
#include "trajectory_request_handler.h"
#include "motion_type.h"
using namespace std;

ros::Publisher  pub_population;
ros::Subscriber sub_odometry;

Utility u;
ramp_msgs::Obstacle obstacle;
TrajectoryRequestHandler* h_traj_req_;

std::vector<float> gr_angulars;
std::vector<float> tgr_linears;

bool odom_recv = false;


tf::Transform T_w_r;


/** This method determines what type of motion an obstacle has */
const MotionType findMotionType(const ramp_msgs::Obstacle ob) { 
  MotionType result;

  // Find the linear and angular velocities
  tf::Vector3 v_linear;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);

  tf::Vector3 v_angular;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

  // Find magnitude of velocity vectors
  float mag_linear_t  = sqrt( tf::tfDot(v_linear, v_linear)   );
  float mag_angular_t = sqrt( tf::tfDot(v_angular, v_angular) );


  // Translation only
  // normally 0.0066 when idle
  if(mag_linear_t >= 0.15 && mag_angular_t < 0.1) {
    result = MotionType::Translation;
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.25) {
    result = MotionType::Rotation;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.1) {
    result = MotionType::TranslationAndRotation;

    // Find v(t-1)
    /*tf::Vector3 v_linear_prev;
    tf::vector3MsgToTF(ob.odom_t_prev.twist.twist.linear, v_linear_prev);
    
    // Find angle between v(t) and v(t-1)
    double theta = tf::tfAngle(v_linear, v_linear_prev);

    // Check if v(t) and v(t-1) have similar directions
    if(theta > 0.01) {
      result = MotionType::TranslationAndSelfRotation;
      tgr_linears.push_back(mag_linear_t);
    } //end if
    else {
      result = MotionType::GlobalRotation;
      gr_angulars.push_back(mag_angular_t);
    } //end global rotation else*/
  } //end else if

  // Else, there is no motion
  else {
    result = MotionType::None;
  }

  return result;
} // End findMotionType





/** This method returns the predicted trajectory for an obstacle for the future duration d 
 * TODO: Remove Duration parameter and make the predicted trajectory be computed until robot reaches bounds of environment */
const ramp_msgs::Trajectory getPredictedTrajectory(const ramp_msgs::Obstacle ob, const ros::Duration d) {
  ramp_msgs::Trajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(obstacle);

  
  // The starting point 
  //trajectory_msgs::JointTrajectoryPoint start;
  ramp_msgs::KnotPoint start;
  start.configuration.ranges = u.ranges_;



  // Positions
  start.configuration.K.push_back(ob.odom_t.pose.pose.position.x);
  start.configuration.K.push_back(ob.odom_t.pose.pose.position.y);
  start.configuration.K.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));


  ramp_msgs::TrajectoryRequest tr;
  std::vector<ramp_msgs::KnotPoint> cs;
  
  // start will be the starting configuration in the path
  cs.push_back(start);

  /** Find the ending configuration for the predicted trajectory based on motion type */
  // If translation
  if(motion_type == MotionType::Translation) {

    // Get the Goal configuration
    ramp_msgs::KnotPoint end;
    end.configuration.ranges = u.ranges_;
    
    end.configuration.K.push_back(start.configuration.K.at(0) + (ob.odom_t.twist.twist.linear.x * d.toSec()));
    end.configuration.K.push_back(start.configuration.K.at(1) + (ob.odom_t.twist.twist.linear.y * d.toSec()));
    end.configuration.K.push_back(start.configuration.K.at(2));
    

    // Now we have starting and ending configurations
    // Build a Path
    cs.push_back(end);
  } // end if translation




  // If translation and rotation
  else if(motion_type == MotionType::TranslationAndRotation) {
    // Find the linear and angular velocities
    tf::Vector3 v_linear;
    tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);

    tf::Vector3 v_angular;
    tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

    // Find magnitude of velocity vectors
    float v = sqrt( tf::tfDot(v_linear, v_linear)   );
    float w = sqrt( tf::tfDot(v_angular, v_angular) );
    float r = v / w;
    std::cout<<"\nv: "<<v<<" w: "<<w<<" r: "<<r;

    std::vector<float> a;
    a.push_back(0);
    a.push_back(0);

    std::vector<float> b;
    b.push_back(ob.odom_t.pose.pose.position.x);
    b.push_back(ob.odom_t.pose.pose.position.y);
    
    // This is the theta from robot origin to robot position
    float polar_theta_r = u.findAngleFromAToB(a, b);
    // This is the radius from robot origin to robot position
    float polar_r_r = sqrt(pow(start.configuration.K.at(0),2) + pow(start.configuration.K.at(1), 2));
    std::cout<<"\npolar_theta_r: "<<polar_theta_r;
    std::cout<<"\npolar_r_r: "<<polar_r_r;


    //float polar_theta_w = u.findAngleFromAToB(a, b);
    //float r_w = sqrt(pow(start.configuration.K.at(0),2) + pow(start.configuration.K.at(1), 2));

    /*std::cout<<"\npolar_theta_w: "<<polar_theta_w;
    std::cout<<"\nr_w: "<<r_w;
    std::cout<<"\n"<<r_w<<"*cos("<<polar_theta_w<<"): "<<r_w*cos(polar_theta_w);
    std::cout<<"\n"<<r_w<<"*sin("<<polar_theta_w<<"): "<<r_w*sin(polar_theta_w);*/

    // Generate intermediate points
    for(float i=0.25f;i<d.toSec();i+=0.25f) {
      ramp_msgs::KnotPoint temp;
      temp.configuration.ranges = u.ranges_;

      // Get the new polar coodinates theta value in robot frame 
      float theta_prime_r = u.displaceAngle(polar_theta_r, w*i);
      //float theta_prime_w = u.displaceAngle(polar_theta_w, w*i);

      // Convert from polar to cartesian in robot frame
      float x_prime_r = polar_r_r * cos(theta_prime_r);
      float y_prime_r = polar_r_r * sin(theta_prime_r);
      float theta_r = u.displaceAngle(start.configuration.K.at(2), w*i);
      std::cout<<"\nx_prime_r: "<<x_prime_r<<" y_prime_r: "<<y_prime_r<<" theta_r: "<<theta_r;

      // Now convert to world coordinates
      tf::Vector3 p_r(x_prime_r, y_prime_r, 0);
      tf::Vector3 p_w = T_w_r * p_r;

      // Push the values onto temp
      temp.configuration.K.push_back(p_w.getX());
      temp.configuration.K.push_back(p_w.getY());
      temp.configuration.K.push_back(u.displaceAngle(theta_r, tf::getYaw(T_w_r.getRotation())));
      
      //temp.configuration.K.push_back(x);
      //temp.configuration.K.push_back(y);
      //temp.configuration.K.push_back(theta);
      
      cs.push_back(temp);
    }
  }
  



  // If rotation
  // Since our robot models are circles, rotation is the same as no movement
  else if(motion_type == MotionType::Rotation || motion_type == MotionType::None) {
    cs.push_back(start);
  } // end if self-rotation, none


  std::cout<<"\nPath: ";
  for(int i=0;i<cs.size();i++) {
    std::cout<<"\n("<<cs.at(i).configuration.K.at(0)<<", "<<cs.at(i).configuration.K.at(1)<<", "<<cs.at(i).configuration.K.at(2);
  }


  tf::Vector3 start_w(start.configuration.K.at(0), start.configuration.K.at(1), 0);
  start_w = T_w_r * start_w;
  cs.at(0).configuration.K.at(0) = start_w.getX();
  cs.at(0).configuration.K.at(1) = start_w.getY();
  cs.at(0).configuration.K.at(2) = u.displaceAngle(start.configuration.K.at(2), tf::getYaw(T_w_r.getRotation()));


  // Now build a Trajectory Request 
  ramp_msgs::Path p = u.getPath(cs);
  tr.request.path = p;
  tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
  tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
  tr.request.v_start.push_back(ob.odom_t.twist.twist.angular.z);
  tr.request.resolutionRate = 5;

  // Get trajectory
  if(h_traj_req_->request(tr)) {
    result = tr.response.trajectory;
  }

  return result;
} //End getPredictedTrajectory




void odometryCallback(const nav_msgs::Odometry& msg) {
  obstacle.odom_t_prev = obstacle.odom_t;
  obstacle.odom_t = msg;

  odom_recv = true;
  
  MotionType motion_type = findMotionType(obstacle);
  if(motion_type == MotionType::Translation) {
    std::cout<<"\nmotion_type == Translation";
  }
  else if(motion_type == MotionType::Rotation) {
    std::cout<<"\nmotion_type == Rotation";
  }
  else if(motion_type == MotionType::TranslationAndRotation) {
    std::cout<<"\nmotion_type == Translation + Rotation";
  }
  else if(motion_type == MotionType::None) {
    std::cout<<"\nmotion_type == None";
  }
}




void getAndSendTrajectory() {

  if(odom_recv) {
    ros::Duration d(10);
    ramp_msgs::Trajectory t = getPredictedTrajectory(obstacle, d);

    ramp_msgs::Population pop;
    pop.population.push_back(t);
    pop.best_id  = 0;
    pop.robot_id = 1;

    pub_population.publish(pop);
  }
}


void fakeOdom() {
  obstacle.odom_t.pose.pose.position.x = 0.5;
  obstacle.odom_t.pose.pose.position.y = 0.86602;
  tf::quaternionTFToMsg(tf::createQuaternionFromYaw(2.356f), obstacle.odom_t.pose.pose.orientation);
  obstacle.odom_t.twist.twist.linear.x = 0.25;
  obstacle.odom_t.twist.twist.linear.y = 0.25;
  obstacle.odom_t.twist.twist.angular.z = 0.25;
  odom_recv = true;
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "viewPredicted");
  ros::NodeHandle handle;

  h_traj_req_ = new TrajectoryRequestHandler(handle);

  pub_population = handle.advertise<ramp_msgs::Population>("population", 1000);
  sub_odometry = handle.subscribe("odometry", 1000, odometryCallback);
  

  T_w_r.setOrigin(tf::Vector3(0, 0, 0));
  T_w_r.setRotation(tf::createQuaternionFromYaw(0));

  ros::Rate r(1);
  std::cout<<"\nWaiting for obstacle's odometry to be published..\n";
//  fakeOdom();
  while(ros::ok()) {
    ros::spinOnce();
    getAndSendTrajectory();
    r.sleep();
  }



  /* Show the max and average velocities */
  float max_w = gr_angulars.at(0);
  float w_sum = max_w;
  for(unsigned int i=1;i<gr_angulars.size();i++) {
    if(gr_angulars.at(i) > max_w) {
      max_w = gr_angulars.at(i);
    }
    w_sum += gr_angulars.at(i);
  }

  float max_v = tgr_linears.at(0);
  float v_sum = max_v;
  for(unsigned int i=1;i<tgr_linears.size();i++) {
    if(tgr_linears.at(i) > max_v) {
      max_v = tgr_linears.at(i);
    }
    v_sum += tgr_linears.at(i);
  }

  float avg_w = w_sum / gr_angulars.size();
  float avg_v = v_sum / tgr_linears.size();

  std::cout<<"\nmax angular velocity when driving straight: "<<max_w;
  std::cout<<"\naverage angular velocity when driving straight: "<<avg_w;
  std::cout<<"\nmax linear velocity when self-rotating: "<<max_v;
  std::cout<<"\naverage linear velocity when self-rotating: "<<avg_v;
  cout<<"\nExiting Normally\n";
  return 0;
}
