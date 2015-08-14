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

Utility utility;
ramp_msgs::Obstacle obstacle;
TrajectoryRequestHandler* h_traj_req_;

std::vector<float> gr_angulars;
std::vector<float> tgr_linears;

bool odom_recv = false;


tf::Transform T_w_b;


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
    //std::cout<<"\nMotion type = Translation\n";
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.1) {
    result = MotionType::Rotation;
    //std::cout<<"\nMotion Type = Rotation\n";
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.1) {
    result = MotionType::TranslationAndRotation;
    //std::cout<<"\nMotion Type = TranslationAndRotation\n";
  } //end else if

  // Else, there is no motion
  else {
    result = MotionType::None;
    //std::cout<<"\nMotion Type = None\n";
  }

  return result;
} // End findMotionType




const ramp_msgs::Path getObstaclePath(const ramp_msgs::Obstacle ob, const MotionType mt) {
  ramp_msgs::Path result;

  std::vector<ramp_msgs::KnotPoint> path;

  ros::Duration predictionTime_(5);

  // Create and initialize the first point in the path
  ramp_msgs::KnotPoint start;
  start.motionState.positions.push_back(ob.odom_t.pose.pose.position.x);
  start.motionState.positions.push_back(ob.odom_t.pose.pose.position.y);
  start.motionState.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));
  std::cout<<"\nodom.pose.x: "<<ob.odom_t.pose.pose.position.x;
  std::cout<<"\nodom.pose.y: "<<ob.odom_t.pose.pose.position.y;
  std::cout<<"\nodom.twist.theta: "<<ob.odom_t.twist.twist.angular.z;

  start.motionState.velocities.push_back(ob.odom_t.twist.twist.linear.x);
  start.motionState.velocities.push_back(ob.odom_t.twist.twist.linear.y);
  start.motionState.velocities.push_back(ob.odom_t.twist.twist.angular.z);

  tf::Vector3 p_st(start.motionState.positions.at(0), start.motionState.positions.at(1), 0); 
  tf::Vector3 p_st_tf = T_w_b * p_st;
  std::cout<<"\np_st.x: "<<p_st.getX()<<" p_st.y: "<<p_st.getY()<<" p_st_tf.x: "<<p_st_tf.getX()<<" p_st_tf.y: "<<p_st_tf.getY();
  start.motionState.positions.at(0) = p_st_tf.getX();
  start.motionState.positions.at(1) = p_st_tf.getY();

  
  
  
 
  std::cout<<"\nBefore changing velocities, start: "<<utility.toString(start);
  double teta = utility.findAngleToVector(start.motionState.positions);
  double phi = start.motionState.positions.at(2);
  double v = start.motionState.velocities.at(0);
  std::cout<<"\nteta: "<<teta<<" phi: "<<phi<<" v: "<<v;
  std::cout<<"\nsin(teta): "<<sin(teta)<<" cos(teta): "<<cos(teta);
  std::cout<<"\ncos("<<phi<<"): "<<cos(phi)<<" sin(phi): "<<sin(phi);
  start.motionState.velocities.at(0) = v*cos(phi);
  start.motionState.velocities.at(1) = v*sin(phi);
  std::cout<<"\nAfter changing, start: "<<utility.toString(start);


  if(v < 0) {
    start.motionState.positions.at(2) = utility.displaceAngle(start.motionState.positions.at(2), PI);
  }

  // Push the first point onto the path
  path.push_back(start);

  /** Find the ending configuration for the predicted trajectory based on motion type */
  // If translation
  if(mt == MotionType::Translation) {

    // Create the Goal Knotpoint
    ramp_msgs::KnotPoint goal;

    double theta = start.motionState.positions.at(2);
    double delta_x = cos(phi)*ob.odom_t.twist.twist.linear.x;
    double delta_y = sin(phi)*ob.odom_t.twist.twist.linear.x;
    std::cout<<"\ntheta: "<<theta<<" delta_x: "<<delta_x<<" delta_y: "<<delta_y;
   

    // Get the goal position in the base frame
    tf::Vector3 ob_goal_b(start.motionState.positions.at(0) + (delta_x * predictionTime_.toSec()), 
                          start.motionState.positions.at(1) + (delta_y * predictionTime_.toSec()),
                          0);

    // Convert the goal position to world coordinates
    //tf::Vector3 goal_w = T_w_b * ob_goal_b;
    
    // Push on the world coordinates
    //goal.motionState.positions.push_back(goal_w.getX());
    //goal.motionState.positions.push_back(goal_w.getY());
    goal.motionState.positions.push_back(ob_goal_b.getX());
    goal.motionState.positions.push_back(ob_goal_b.getY());
    goal.motionState.positions.push_back(start.motionState.positions.at(2));


    goal.motionState.velocities.push_back(start.motionState.velocities.at(0));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(1));
    goal.motionState.velocities.push_back(start.motionState.velocities.at(2));


    std::cout<<"\nGoal: "<<utility.toString(goal);
    
    // Push goal onto the path
    path.push_back(goal);
  } // end if translation




  result = utility.getPath(path);
  return result; 
}






/** This method returns the predicted trajectory for an obstacle for the future duration d 
 * TODO: Remove Duration parameter and make the predicted trajectory be computed until robot reaches bounds of environment */
const ramp_msgs::RampTrajectory getPredictedTrajectory(const ramp_msgs::Obstacle ob, const ros::Duration d) {
  ramp_msgs::RampTrajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(ob);
  

  // Now build a Trajectory Request 
  ramp_msgs::TrajectoryRequest tr;
    tr.request.path = getObstaclePath(ob, motion_type);
    tr.request.type = PREDICTION;

  // Get trajectory
  if(h_traj_req_->request(tr)) {
    result = tr.response.trajectory;
  }

  return result;
} //End getPredictedTrajectory





void odometryCallback(const nav_msgs::Odometry& msg) {
  obstacle.odom_t = msg;

  odom_recv = true;
}




void getAndSendTrajectory() {

  if(odom_recv) {
    ros::Duration d(10);
    ramp_msgs::RampTrajectory t = getPredictedTrajectory(obstacle, d);

    ramp_msgs::Population pop;
    pop.population.push_back(t);
    pop.best_id  = 0;
    pop.robot_id = 1;

    std::cout<<"\nPublishing population\n";
    std::cout<<"\nTrajec size: "<<t.trajectory.points.size();
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
  sub_odometry = handle.subscribe("odom", 1000, odometryCallback);
 

  T_w_b.setOrigin(tf::Vector3(0, 1, 0));
  T_w_b.setRotation(tf::createQuaternionFromYaw(0));

  ros::Rate r(10);
  std::cout<<"\nWaiting for obstacle's odometry to be published..\n";
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
