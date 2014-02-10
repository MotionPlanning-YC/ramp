#include "ros/ros.h"
#include <iostream>
#include "utility.h"
#include "ramp_msgs/Obstacle.h"
#include "ramp_msgs/Population.h"
#include "trajectory_request_handler.h"
using namespace std;

ros::Publisher  pub_population;
ros::Subscriber sub_odometry;

Utility u;
ramp_msgs::Obstacle obstacle;
TrajectoryRequestHandler* h_traj_req_;

std::vector<float> gr_angulars;
std::vector<float> tgr_linears;


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
  if(mag_linear_t >= 0.15 && mag_angular_t < 0.25) {
    result = MotionType::Translation;
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.25) {
    result = MotionType::SelfRotation;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.25) {

    // Find v(t-1)
    tf::Vector3 v_linear_prev;
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
    } //end global rotation else
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
  ramp_msgs::Configuration start;
  start.ranges = u.ranges_;

  // If translation
  if(motion_type == MotionType::Translation) {

    // Positions
    start.K.push_back(ob.odom_t.pose.pose.position.x);
    start.K.push_back(ob.odom_t.pose.pose.position.y);
    start.K.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

    // Get the Goal configuration
    ramp_msgs::Configuration end;
    end.ranges = u.ranges_;
    
    end.K.push_back(start.K.at(0) + (ob.odom_t.twist.twist.linear.x * d.toSec()));
    end.K.push_back(start.K.at(1) + (ob.odom_t.twist.twist.linear.y * d.toSec()));
    end.K.push_back(start.K.at(2));
    

    // Now we have starting and ending configurations
    // Build a Path
    std::vector<ramp_msgs::Configuration> cs;
    cs.push_back(start);
    cs.push_back(end);
    ramp_msgs::Path p = u.getPath(cs);
    
    // Now build a Trajectory Request 
    ramp_msgs::TrajectoryRequest tr;
    tr.request.path = p;
    tr.request.v_start.push_back(0.33f);
    tr.request.v_end.push_back(0.33f);
    tr.request.resolutionRate = 5;

    // Get trajectory
    if(h_traj_req_->request(tr)) {
      result = tr.response.trajectory;
    }
  } // end if translation


  else if(motion_type == MotionType::SelfRotation || motion_type == MotionType::None) {

    // Positions
    start.K.push_back(ob.odom_t.pose.pose.position.x);
    start.K.push_back(ob.odom_t.pose.pose.position.y);
    start.K.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));
    
    std::vector<ramp_msgs::Configuration> cs;
    cs.push_back(start);
    cs.push_back(start);
    

    // Now we have configurations
    // Build a Path
    ramp_msgs::Path p = u.getPath(cs);
    
    // Now build a Trajectory Request 
    ramp_msgs::TrajectoryRequest tr;
    tr.request.path = p;

    for(unsigned int i=0;i<p.configurations.size()-1;i++) {
      tr.request.v_start.push_back(PI/4);
      tr.request.v_end.push_back(PI/4);
    }

    tr.request.resolutionRate = 5;

    // Get trajectory
    if(h_traj_req_->request(tr)) {
      result = tr.response.trajectory;
    }
  } // end if self-rotation, none


  return result;
} //End getPredictedTrajectory




void odometryCallback(const nav_msgs::Odometry& msg) {
  obstacle.odom_t_prev = obstacle.odom_t;
  obstacle.odom_t = msg;
  
  MotionType motion_type = findMotionType(obstacle);
  if(motion_type == MotionType::Translation) {
    std::cout<<"\nmotion_type == Translation";
  }
  else if(motion_type == MotionType::SelfRotation) {
    std::cout<<"\nmotion_type == Self-Rotation";
  }
  else if(motion_type == MotionType::TranslationAndSelfRotation) {
    std::cout<<"\nmotion_type == Translation + Self-Rotation";
  }
  else if(motion_type == MotionType::GlobalRotation) {
    std::cout<<"\nmotion_type == Global Rotation";
  }
  else if(motion_type == MotionType::None) {
    std::cout<<"\nmotion_type == None";
  }
}




void getAndSendTrajectory() {
  ros::Duration d(10);
  ramp_msgs::Trajectory t = getPredictedTrajectory(obstacle, d);

  ramp_msgs::Population pop;
  pop.population.push_back(t);
  pop.best_id  = 0;
  pop.robot_id = 1;

  pub_population.publish(pop);
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "viewPredicted");
  ros::NodeHandle handle;

  h_traj_req_ = new TrajectoryRequestHandler(handle);

  pub_population = handle.advertise<ramp_msgs::Population>("population", 1000);
  sub_odometry = handle.subscribe("odometry", 1000, odometryCallback);
  


  ros::Rate r(1);
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
