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

// Track the maximum angular velocity in global rotation
// because when we move straight, it gives us global rotation
// but we want translation, so we must know the threshold to check the angular V
float max_angular_gr = -9999.f;

// Track the maximum linear velocity in translation + global rotation
// because when we turn, it gives us translation + global rotation
// but we want self-rotation, so we must know the threshold to check the linear V
float max_linear_tgr = -9999.f;


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
  std::cout<<"\nmag_linear_t: "<<mag_linear_t;
  std::cout<<"\nmag_angular_t: "<<mag_angular_t;
  std::cout<<"\nmax_angular_gr: "<<max_angular_gr;
  std::cout<<"\nmax_linear_tgr: "<<max_linear_tgr;



  // Translation only
  // normally 0.0066 when idle
  if(mag_linear_t >= 0.001 && mag_angular_t < 0.001) {
    /*if(id == 1)
      std::cout<<"\nRobot 2 has ";
    else
      std::cout<<"\nRobot 1 has ";
    std::cout<<"Motion Type == Translation";*/
    result = MotionType::Translation;
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.001 && mag_angular_t >= 0.001) {
    /*if(id == 1)
      std::cout<<"\nRobot 2 has ";
    else
      std::cout<<"\nRobot 1 has ";
    std::cout<<"Motion Type == Self-Rotation";*/
    result = MotionType::SelfRotation;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.001 && mag_angular_t >= 0.001) {

    // Find v(t-1)
    tf::Vector3 v_linear_prev;
    tf::vector3MsgToTF(ob.odom_t_prev.twist.twist.linear, v_linear_prev);
    
    // Find angle between v(t) and v(t-1)
    double theta = tf::tfAngle(v_linear, v_linear_prev);

    // Check if v(t) and v(t-1) have similar directions
    if(theta > 0.01) {
      result = MotionType::TranslationAndSelfRotation;
      if(mag_linear_t > max_linear_tgr) {
        max_linear_tgr = mag_linear_t;
      }
    } //end if
    else {
      result = MotionType::GlobalRotation;
      if(mag_angular_t > max_angular_gr) {
        max_angular_gr = mag_angular_t;
      }
    } //end global rotation else
  } //end else if

  // Else, there is no motion
  else {
    /*if(id == 1)
      std::cout<<"\nRobot 2 has ";
    else
      std::cout<<"\nRobot 1 has ";
    std::cout<<"Motion Type == None";*/
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


  std::cout<<"\nResulting trajectory size: "<<result.trajectory.points.size();
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
  ros::Duration d(5);
  //ramp_msgs::Trajectory t = getPredictedTrajectory(obstacle, d);

  /*ramp_msgs::Population pop;
  pop.population.push_back(t);
  pop.best_id  = 0;
  pop.robot_id = 1;

  pub_population.publish(pop);*/
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "viewPredicted");
  ros::NodeHandle handle;

  h_traj_req_ = new TrajectoryRequestHandler(handle);

  pub_population = handle.advertise<ramp_msgs::Population>("population", 1000);
  sub_odometry = handle.subscribe("odometry", 1000, odometryCallback);
  
  ros::Duration d(0.25);
  /*while(ros::ok()) {
    getAndSendTrajectory();
    d.sleep();
    ros::spinOnce();
  }*/

  ros::spin();
  cout<<"\nExiting Normally\n";
  return 0;
}
