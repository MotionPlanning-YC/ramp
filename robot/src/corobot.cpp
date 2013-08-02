
#include "corobot.h"

const std::string Corobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string Corobot::TOPIC_STR_ODOMETRY="odometry";
const std::string Corobot::TOPIC_STR_TWIST="twist";


Corobot::Corobot() {
  configuration_.x = 0;
  configuration_.y = 0;
  configuration_.theta = 0;
}


Corobot::~Corobot() {}



void Corobot::updateState(const nav_msgs::Odometry::ConstPtr& msg) {
  
  //Push the latest theta
  thetas_.push_back(tf::getYaw(msg->pose.pose.orientation));
  
  //If we have enough thetas to average,
  if(thetas_.size() == POSE_COUNT_THRESHOLD) {
    
    //Average theta    
    double avg_theta = 0;
    for(unsigned int i=0;i<POSE_COUNT_THRESHOLD;i++) {
      avg_theta += thetas_.at(i) / POSE_COUNT_THRESHOLD;
    }
    
    //Set configuration
    configuration_.x = msg->pose.pose.position.x;
    configuration_.y = msg->pose.pose.position.y;
    configuration_.theta = avg_theta;
     
    //Clear vector
    thetas_.clear();
  }
}

/** Publishes the MotorCommand msg. The Corobot will drive based on the msg. */
void Corobot::drive(corobot_msgs::MotorCommand msg) const {
  pub_phidget_motor_.publish(msg);
}

void Corobot::stop() const {
  corobot_msgs::MotorCommand msg;
  msg.leftSpeed  = 0;
  msg.rightSpeed = 0;
  msg.secondsDuration = 0;
  msg.acceleration = ACCELERATION_CONSTANT;

  //Send Command
  drive(msg);
}


void Corobot::driveStraight(unsigned int speed) const {

  corobot_msgs::MotorCommand msg;

  //Set the speed
  msg.leftSpeed = speed;
  msg.rightSpeed = speed;
  
  //The time should be indefinite, so just make it very high
  msg.secondsDuration = 1000;

  //acceleration does not matter with corobot
  msg.acceleration = ACCELERATION_CONSTANT;

  //Send command
  drive(msg);
}


void Corobot::turn(unsigned int speed, bool cwise) const {


  corobot_msgs::MotorCommand msg;

  //Set the speed
  if(cwise) {
    msg.leftSpeed = speed;
  }
  else 
    msg.leftSpeed = -speed;

  msg.rightSpeed = -msg.leftSpeed;
  
  //The time should be indefinite, so just make it very high
  msg.secondsDuration = 1000;

  //acceleration does not matter with corobot
  msg.acceleration = ACCELERATION_CONSTANT;
  
  //Send Command
  drive(msg);
}


void Corobot::turn(float speed, float angle) const {
 geometry_msgs::Twist v;

 v.linear.x = 0;
 v.angular.z = speed;
 
 //Need to set up stopping the turn once the desired angle has been turned 
 pub_twist_.publish(v);
}


void Corobot::updateTrajectory(const ramp_msgs::Trajectory msg) {
  std::cout<<"\nIn updateTrajectory\n";
  std::cout<<"\ntrajectory.points.size():"<<trajectory_.trajectory.points.size();
}



float Corobot::getSpeedToWaypoint(trajectory_msgs::JointTrajectoryPoint waypoint1, trajectory_msgs::JointTrajectoryPoint waypoint2) {

  
  std::cout<<"\nwaypoint1:";
  for(unsigned int i=0;i<3;i++) {
    std::cout<<waypoint1.positions.at(i)<<", ";
  }
  
  std::cout<<"\nwaypoint2:";
  for(unsigned int i=0;i<3;i++) {
    std::cout<<waypoint2.positions.at(i)<<", ";
  }

  //Get the velocity vector, index 0 is x component, index 1 is y component
  float v[2];
  v[0] = waypoint2.positions.at(0) - waypoint1.positions.at(0);
  v[1] = waypoint2.positions.at(1) - waypoint1.positions.at(1);

  float mag_v = sqrt( pow(v[0],2) + pow(v[1],2) );
  std::cout<<"\nmag_v:"<<mag_v; 
  float time = waypoint2.time_from_start.toSec() - waypoint1.time_from_start.toSec();
  std::cout<<"\ntime:"<<time;
  float speed = mag_v / time;
  std::cout<<"\nspeed:"<<speed;
  return speed;
}


void Corobot::moveOnTrajectory() {
  
  //Get the number of waypoints
  int num = trajectory_.trajectory.points.size();

  //Build a vector of the time_from_starts
  std::vector<ros::Time> end_times;
  ros::Time start = ros::Time::now() + ros::Duration(1.0);

  for(unsigned int i=0;i<num-1;i++) {
    end_times.push_back(start + trajectory_.trajectory.points.at(i+1).time_from_start );
  } 
  
  ros::Rate r(25);
  
  geometry_msgs::Twist twist;
  twist.linear.x = 0.5;
  
  //For each waypoint
  for(unsigned int i=0;i<num-1;i++) {
    
    //Send the twist msg at some rate r
    while(ros::ok() && ros::Time::now() < end_times.at(i)) {
      pub_twist_.publish(twist); 
      r.sleep();
    }
  }
}


