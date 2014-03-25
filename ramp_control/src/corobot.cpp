
#include "corobot.h"

const std::string Corobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string Corobot::TOPIC_STR_ODOMETRY="odometry";
const std::string Corobot::TOPIC_STR_UPDATE="update";
const std::string Corobot::TOPIC_STR_TWIST="twist";
const float BASE_WIDTH=0.2413;

const float timeNeededToTurn = 2.5; 



Corobot::Corobot() : k_dof_(3), restart(false), mutex_(false), moving_(false), num(0), num_traveled(0), i_knot_points(0) { 
  for(unsigned int i=0;i<k_dof_;i++) {
    configuration_.K.push_back(0);
    configuration_.ranges.push_back(u.standardRanges.at(i));
  }
}


Corobot::~Corobot() {}


void Corobot::releaseMutex() {
  mutex_ = true;
}

/** Returns true if the mutex was initially locked */
void Corobot::lockMutex() {
  while(!mutex_) {}
  mutex_ = false;
}


void Corobot::setConfiguration(float x, float y, float theta) {
  configuration_.K.clear();
  
  configuration_.K.push_back(x);
  configuration_.K.push_back(y);

  // *** This was (theta - angle_at_start) before - why?
  configuration_.K.push_back(theta);
}






/** This is a callback for receiving odometry from the robot and sets the configuration of the robot */
void Corobot::updateState(const nav_msgs::Odometry& msg) {
  
  // Set configuration
  setConfiguration(msg.pose.pose.position.x, msg.pose.pose.position.y, tf::getYaw(msg.pose.pose.orientation));
} // End updateState




/** This method is on a timer to publish the robot's latest configuration */
void Corobot::updatePublishTimer(const ros::TimerEvent&) {
    ramp_msgs::Update msg;
    msg.configuration = configuration_;
    
    if (pub_update_) {
        pub_update_.publish(msg);
    }
} // End updatePublishTimer





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

  // Send Command
  drive(msg);
}


void Corobot::driveStraight(const unsigned int speed) const {

  corobot_msgs::MotorCommand msg;

  // Set the speed
  msg.leftSpeed = speed;
  msg.rightSpeed = speed;
  
  // The time should be indefinite, so just make it very high
  msg.secondsDuration = 1000;

  // acceleration does not matter with corobot
  msg.acceleration = ACCELERATION_CONSTANT;

  // Send command
  drive(msg);
}


void Corobot::turn(const unsigned int speed, const bool cwise) const {


  corobot_msgs::MotorCommand msg;

  // Set the speed
  if(cwise) {
    msg.leftSpeed = speed;
  }
  else 
    msg.leftSpeed = -speed;

  msg.rightSpeed = -msg.leftSpeed;
  
  // The time should be indefinite, so just make it very high
  msg.secondsDuration = 1000;

  // acceleration does not matter with corobot
  msg.acceleration = ACCELERATION_CONSTANT;
  
  // Send Command
  drive(msg);
}


void Corobot::turn(const float speed, const float angle) const {
 geometry_msgs::Twist v;

 v.linear.x = 0;
 v.angular.z = speed;
 
 // Need to set up stopping the turn once the desired angle has been turned 
 pub_twist_.publish(v);
}

/** This method updates the Corobot's trajectory
 *   It calls calculateSpeedsAndTimes to update the robot's vectors needed to move */
void Corobot::updateTrajectory(const ramp_msgs::Trajectory msg) {
  std::cout<<"\nIn updateTrajectory!\n";
  
  restart = true;
  num_traveled = 0;
  i_knot_points = 0;
  trajectory_ = msg;
  num = trajectory_.trajectory.points.size();
  calculateSpeedsAndTime();

  //std::cout<<"\nVectors now:";
  //printVectors();
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
  // Get the velocity vector, index 0 is x component, index 1 is y component
  float v[2];
  v[0] = waypoint2.positions.at(0) - waypoint1.positions.at(0);
  v[1] = waypoint2.positions.at(1) - waypoint1.positions.at(1);

  float mag_v = sqrt( pow(v[0],2) + pow(v[1],2) );
  float time = waypoint2.time_from_start.toSec() - waypoint1.time_from_start.toSec();
  float speed = mag_v / time;
  //std::cout<<"\nspeed: "<<speed;
  
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
    float x_dif = waypoint2.positions.at(0) - waypoint1.positions.at(0); //  difference in x between the waypoint 2 and 1
    float y_dif = waypoint2.positions.at(1) - waypoint1.positions.at(1); //  difference in y between the waypoint 2 and 1
    float angle = asin((y_dif)/(sqrt(x_dif*x_dif + y_dif*y_dif))); //  Orientation of this trajectory in the X/Y axes
    if (x_dif < 0)
	    return M_PI - angle;
    return angle;
}

/** 
 * Calculate all the necessary values to move the robot: the linear and angular velocities as well as ending times
 * This method sets the vectors speeds, angular_speeds, and end_times
 */
void Corobot::calculateSpeedsAndTime ()
{
  angular_speeds.clear();
  orientations.clear();
  speeds.clear();
  end_times.clear();
  
  // Get number of knot points
  int num = trajectory_.trajectory.points.size(); 

  // Get the starting time
  ros::Time start_time = ros::Time::now() + ros::Duration(0.0);

  float speed_loc;
  // We go through all the waypoints
  for(int i=0;i<num-1;i++) {

    // Get the next points on the trajectory
    trajectory_msgs::JointTrajectoryPoint current = trajectory_.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint next    = trajectory_.trajectory.points.at(i+1);

    // Push on the linear speed for the waypoint
    // Find the norm of the velocity vector
    speed_loc = sqrt(pow(current.velocities.at(0),2) + pow(current.velocities.at(1), 2));
    speeds.push_back(speed_loc);
      

    // Push on the angular speed for the waypoint
    // Angular speed is at index 2 of the point 
    angular_speeds.push_back(current.velocities.at(2));
    
    // Calculate the ending time for each waypoints
    end_times.push_back(start_time + next.time_from_start );

    // Push on orientation at knot point
    orientations.push_back(current.positions.at(2));

  } 
  
  //printVectors();
}


void Corobot::sendTwist() const {
  pub_twist_.publish(twist_); 
}


void Corobot::sendTwist(const geometry_msgs::Twist t) const {
  pub_twist_.publish(t); 
}

void Corobot::printVectors() const {
    
  std::cout<<"\nspeeds: [";
  for(unsigned int i=0;i<speeds.size()-1;i++) {
    std::cout<<speeds.at(i)<<", ";
  }
  std::cout<<speeds.at(speeds.size()-1)<<"]";

  std::cout<<"\nend_times: [";
  for(unsigned int i=0;i<end_times.size()-1;i++) {
    std::cout<<end_times.at(i)<<", ";
  }
  std::cout<<end_times.at(end_times.size()-1)<<"]";

  std::cout<<"\nangular_speeds: [";
  for(unsigned int i=0;i<angular_speeds.size()-1;i++) {
    std::cout<<angular_speeds.at(i)<<", ";
  }
  std::cout<<angular_speeds.at(angular_speeds.size()-1)<<"]";


}


void Corobot::moveOnTrajectory(bool simulation) 
{
  restart = false;
  
  ros::Rate r(50);
  
  ros::Duration delay = ros::Duration(0); //  Save the time it took to do all the turns
  ros::Time start;
    

  while( (num_traveled+1) < num) {
    restart = false;
    
    // Set velocities
    twist_.linear.x  = speeds.at(num_traveled);
    twist_.angular.z = angular_speeds.at(num_traveled);
    //std::cout<<"\ntwist_linear: "<<twist_.linear.x;
    //std::cout<<"\ntwist_angular: "<<twist_.angular.z;
    //std::cout<<"\nend_times.size():"<<end_times.size()<<"\n";
    //std::cout<<"\norientations.size():"<<orientations.size()<<"\n";

    ros::Time g_time = end_times.at(num_traveled);
    while(ros::ok() && ros::Time::now() < g_time) {
    
      // Adjust the angular speed to correct errors in turning
      // Commented out because it was producing erratic driving
      // Should be fixed at some point
      if(twist_.linear.x > 0.0f) {
        //std::cout<<"\ninitial_theta: "<<initial_theta;
        float actual_theta = u.displaceAngle(initial_theta, configuration_.K.at(2));
        
        float dist = u.findDistanceBetweenAngles(actual_theta, orientations.at(num_traveled));
        //std::cout<<"\nactual theta: "<<actual_theta;
        //std::cout<<"\norientations.at("<<num_traveled<<"): "<<orientations.at(num_traveled);
        //std::cout<<"\ndist: "<<dist;
        twist_.angular.z = -1*dist;
      }
    
      // Send the twist_message to move the robot
      sendTwist();

      // If we have the simulation up, publish to cmd_vel
      if(simulation) pub_cmd_vel_.publish(twist_);

      // Spin to check for updates
      ros::spinOnce();
      
      //Sleep
      r.sleep();

      // Check if a new trajectory has been received
      if(restart) {
        break;
      }
    }
    
    if(restart) {
      delay = ros::Duration(0);
      restart=false;
      continue;
    }
  
    num_traveled++;

    // Spin once to check for updates in the trajectory
    ros::spinOnce();
  } // end while

    // Stops the wheels
    twist_.linear.x = 0;
    twist_.angular.z = 0;
    sendTwist();

} // End moveOnTrajectory


