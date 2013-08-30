
#include "corobot.h"

const std::string Corobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string Corobot::TOPIC_STR_ODOMETRY="odometry";
const std::string Corobot::TOPIC_STR_UPDATE="update";
const std::string Corobot::TOPIC_STR_TWIST="twist";

const float timeNeededToTurn = 2.5; 

Corobot::Corobot() : k_dof_(3){
  for(unsigned int i=0;i<k_dof_;i++) {
    configuration_.K.push_back(0);
  }
}


Corobot::~Corobot() {}



void Corobot::setConfiguration(float x, float y, float theta) {
  configuration_.K.clear();

  configuration_.K.push_back(x);
  configuration_.K.push_back(y);
  configuration_.K.push_back(theta);
}


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
    setConfiguration(msg->pose.pose.position.x, msg->pose.pose.position.y, avg_theta);
     
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
    float angle = asin((y_dif)/(sqrt(x_dif*x_dif + y_dif*y_dif))); // Orientation of this trajectory in the X/Y axes
    return angle;
}

/** 
 * Calculate all the necessary values to move the robot: the linear and angular velocities as well as ending times
 */
void Corobot::calculateSpeedsAndTime ()
{
  int i_knot_points = 0; // Index for going through knotpoints. We don't need the index of the current knot point but the next one
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
    
    //Caculate the angle the make sure we are at the correct orientation before trying to reach for the next knotpoint
    if (i == trajectory_.index_knot_points.at(i_knot_points))
    {
        //calculate the orientation of each trajectory
        current_orientation = getTrajectoryOrientation(trajectory_.trajectory.points.at(i), trajectory_.trajectory.points.at(i+1));
        //caculate the angular speed needed before to satisfy the correct direction for the next knotpoint
        angular_speeds_knotpoints.push_back(getAngularSpeed(past_orientation, current_orientation));
        past_orientation = current_orientation;
        
        i_knot_points++;
    }
    
    //calculate the angular speed for the final point
    if ( i == num-2)
    {
      angular_speeds_knotpoints.push_back(getAngularSpeed(current_orientation, trajectory_.trajectory.points.at(i+1).positions.at(2)));
    }
  } 
}


void Corobot::controlCycle(geometry_msgs::Twist twist, ros::Time end_time, ros::Rate r) {

    //Send the twist msg at some rate r
    //while(ros::ok() && ros::Time::now() < end_times.at(i)) {
    while(ros::ok() && ros::Time::now() < end_time) {
      pub_twist_.publish(twist); 
      r.sleep();
    }
}

void Corobot::moveOnTrajectory() 
{
  int num = trajectory_.trajectory.points.size(); //Get the number of waypoints
  int i_knot_points = 0; // Index for going through knotpoints. We don't need the index of the current knot point but the next one 
  ros::Rate r(50);
  geometry_msgs::Twist twist;
  ros::Duration delay = ros::Duration(0); // Save the time it took to do all the turns
  ros::Time start;
  
  calculateSpeedsAndTime();
  
  
  //For each waypoint we publish the Twist message
  for(unsigned int i=0;i<num-1;i++) {
  
    ROS_ERROR("knotpoint: %d/%d angular speed: %f, linear speed: %f\n", i,i_knot_points, angular_speeds_knotpoints.at(i_knot_points), speeds.at(i));
    // We need to make sure we are at the correct direction to reach the next waypoint
    if (i == trajectory_.index_knot_points.at(i_knot_points))
    {   
        if ((angular_speeds_knotpoints.at(i_knot_points) > 0.01 || angular_speeds_knotpoints.at(i_knot_points) < -0.01) )
        {
            twist.linear.x = 0;
            twist.angular.z = angular_speeds_knotpoints.at(i_knot_points);
            start = ros::Time::now();

            while(ros::ok() && ros::Time::now() < (start + ros::Duration(timeNeededToTurn))) {
                pub_twist_.publish(twist); 
                r.sleep();
            }

            delay += ros::Duration(timeNeededToTurn); //we save as a delay the time it took to turn
        }
        i_knot_points++;
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
    
    // Satisfy the orientation onces the final knotpoint has been reached
    if ( i == num-2)
    {
        twist.linear.x = 0;
        twist.angular.z = angular_speeds_knotpoints.at(i_knot_points);
        start = ros::Time::now();

        while(ros::ok() && ros::Time::now() < (start + ros::Duration(timeNeededToTurn))) {
            pub_twist_.publish(twist); 
            r.sleep();
        }

        delay += ros::Duration(timeNeededToTurn); //we save as a delay the time it took to turn
    }

  }
}


