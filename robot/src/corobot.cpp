
#include "corobot.h"

const std::string Corobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string Corobot::TOPIC_STR_ODOMETRY="odometry";
const std::string Corobot::TOPIC_STR_TWIST="twist";

const float timeNeededToTurn = 3.0; 

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
void Corobot::drive(const corobot_msgs::MotorCommand msg) const {
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


void Corobot::driveStraight(const unsigned int speed) const {

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


void Corobot::turn(const unsigned int speed, const bool cwise) const {


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


void Corobot::turn(const float speed, const float angle) const {
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



const float Corobot::getSpeedToWaypoint(const trajectory_msgs::JointTrajectoryPoint waypoint1, const trajectory_msgs::JointTrajectoryPoint waypoint2) const {

/*  
  for(unsigned int i=0;i<3;i++) {
    std::cout<<waypoint1.positions.at(i)<<", ";
  }
  
  for(unsigned int i=0;i<3;i++) {
    std::cout<<waypoint2.positions.at(i)<<", ";
  }
*/
  //Get the velocity vector, index 0 is x component, index 1 is y component
  float v[2];
  v[0] = waypoint2.positions.at(0) - waypoint1.positions.at(0);
  v[1] = waypoint2.positions.at(1) - waypoint1.positions.at(1);

  float mag_v = sqrt( pow(v[0],2) + pow(v[1],2) );
  float time = waypoint2.time_from_start.toSec() - waypoint1.time_from_start.toSec();
  float speed = mag_v / time;
  return speed;
}

const float Corobot::getAngularSpeed(const float direction1, const float direction2) const {

  float delta_angle = direction2 - direction1;

  return delta_angle / timeNeededToTurn;
}

/**
 * Get the angle orientation of a trajectory
 */ 
float Corobot::getTrajectoryOrientation(const trajectory_msgs::JointTrajectoryPoint waypoint1, const trajectory_msgs::JointTrajectoryPoint waypoint2) const
{
    float x_dif = waypoint2.positions.at(0) - waypoint1.positions.at(0); // difference in x between the waypoint 2 and 1
    float y_dif = waypoint2.positions.at(1) - waypoint1.positions.at(1); // difference in y between the waypoint 2 and 1
    return (x_dif)/(sqrt(x_dif*x_dif + y_dif*y_dif)); // Orientation of this trajectory in the X/Y axes

}

/** 
 * Calculate all the necessary values to move the robot: the linear and angular velocities as well as ending times
 */
void Corobot::calculateSpeedsAndTime ()
{
  int i_knot_points = 1; // Index for going through knotpoints. We don't need the index of the current knot point but the next one
  float past_orientation = 0;
  float current_orientation = 0;
  int num = trajectory_.trajectory.points.size(); //Get the number of waypoints
  ros::Time start = ros::Time::now() + ros::Duration(1.0);
  
  // WE go through all the waypoints
  for(unsigned int i=0;i<num-1;i++) {
    // Calculate the ending time for each waypoints
    end_times.push_back(start + trajectory_.trajectory.points.at(i+1).time_from_start );
    //Culatate the linear speed between each waypoints
    speeds.push_back(getSpeedToWaypoint(trajectory_.trajectory.points.at(i), trajectory_.trajectory.points.at(i+1)));
    
    //calculate the orientation of each trajectory
    current_orientation = getTrajectoryOrientation(trajectory_.trajectory.points.at(i), trajectory_.trajectory.points.at(i+1));
    //caculate the angular speed needed before to satisfy the correct direction for the next waypoint
    angular_speeds_waypoints.push_back(getAngularSpeed(past_orientation, current_orientation));
    past_orientation = current_orientation;
    
    //calculate the angular speed for each knot points
    if ( i+1 == trajectory_.index_knot_points.at(i_knot_points))
    {
      angular_speeds_knotpoints.push_back(getAngularSpeed(trajectory_.trajectory.points.at(trajectory_.index_knot_points[i_knot_points-1]).positions.at(2), trajectory_.trajectory.points.at(i+1).positions.at(2)));

      i_knot_points++;
    }
  } 
}

void Corobot::moveOnTrajectory() 
{
  int num = trajectory_.trajectory.points.size(); //Get the number of waypoints
  int i_knot_points = 1; // Index for going through knotpoints. We don't need the index of the current knot point but the next one 
  ros::Rate r(25);
  geometry_msgs::Twist twist;
  ros::Duration delay = ros::Duration(0); // Save the time it took to do all the turns
  
  
  calculateSpeedsAndTime();
  
  //For each waypoint we publish the Twist message
  for(unsigned int i=0;i<num-1;i++) {
  
    // We need to make sure we are at the correct direction to reach the next waypoint
    if (angular_speeds_waypoints.at(i) > 0.01)
    {
        twist.linear.x = 0;
        twist.angular.z = angular_speeds_waypoints.at(i);
        while(ros::ok() && ros::Time::now() < (end_times.at(i) + ros::Duration(timeNeededToTurn))) {
            pub_twist_.publish(twist); 
            r.sleep();
        }
        delay += ros::Duration(timeNeededToTurn); //we save as a delay the time it took to turn
    }
    end_times.at(i) += delay; // we make sure that the time it took us for all the turns doesn't make the robot go straight for less time than it should 
    
    // Now we can go straight to reach the waypoint
    twist.linear.x = speeds.at(i);
    twist.angular.z = 0;
    //Send the twist msg at some rate r
    while(ros::ok() && ros::Time::now() < end_times.at(i)) {
      pub_twist_.publish(twist); 
      r.sleep();
    }

    // rotate the robot to the specify angle if a knot point has been reached. 
    if ( i+1 == trajectory_.index_knot_points.at(i_knot_points))
    {
      twist.linear.x = 0;
      twist.angular.z = angular_speeds_knotpoints.at(i_knot_points-1);
      //Send the twist msg at some rate r
      while(ros::ok() && ros::Time::now() < (end_times.at(i) + ros::Duration(timeNeededToTurn))) {
        pub_twist_.publish(twist); 
        r.sleep();
        delay += ros::Duration(timeNeededToTurn);
      }
      //change the index to the next knot point
      i_knot_points++;
    }

  }
}


