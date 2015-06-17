
#include "mobile_robot.h"

const std::string MobileRobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string MobileRobot::TOPIC_STR_ODOMETRY="odometry";
const std::string MobileRobot::TOPIC_STR_UPDATE="update";
const std::string MobileRobot::TOPIC_STR_TWIST="twist";
const float BASE_WIDTH=0.2413;

const float timeNeededToTurn = 2.5; 



MobileRobot::MobileRobot() : restart_(false), num_(0), num_traveled_(0), k_dof_(3), h_traj_req_(0) { 
  for(unsigned int i=0;i<k_dof_;i++) {
    motion_state_.positions.push_back(0);
    motion_state_.velocities.push_back(0);
    motion_state_.accelerations.push_back(0);
    motion_state_.jerks.push_back(0);
  }

  prev_motion_state_ = motion_state_;


  zero_twist_.linear.x = 0.;
  zero_twist_.angular.z = 0.;
}


MobileRobot::~MobileRobot() {
  if(h_traj_req_ != 0) {
    delete h_traj_req_;
    h_traj_req_ = 0;
  }
}


void MobileRobot::init(ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler((const ros::NodeHandle&)h);
}




/** Publishes the MotorCommand msg. The MobileRobot will drive based on the msg. */
void MobileRobot::drive(const corobot_msgs::MotorCommand msg) const {
  pub_phidget_motor_.publish(msg);
}

void MobileRobot::stop() const {
  corobot_msgs::MotorCommand msg;
  msg.leftSpeed  = 0;
  msg.rightSpeed = 0;
  msg.secondsDuration = 0;
  msg.acceleration = ACCELERATION_CONSTANT;

  // Send Command
  drive(msg);
}


void MobileRobot::driveStraight(const unsigned int speed) const {

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


void MobileRobot::turn(const unsigned int speed, const bool cwise) const {


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


void MobileRobot::turn(const float speed, const float angle) const {
 geometry_msgs::Twist v;

 v.linear.x = 0;
 v.angular.z = speed;
 
 // Need to set up stopping the turn once the desired angle has been turned 
 pub_twist_.publish(v);
}


/*
 * Computes acceleration manually
 */
const std::vector<double> MobileRobot::computeAcceleration() const 
{
  std::vector<double> result;

  for(unsigned int i=0;i<k_dof_;i++) 
  {
    double a = (motion_state_.velocities.at(i) - prev_motion_state_.velocities.at(i)) 
              / (ros::Time::now() - prev_t_).toSec();
    result.push_back(a);
  }

  return result;
}

/* 
 * This is a callback for receiving odometry from the robot and sets the configuration of the robot 
 * It does not mutate any motion data. The time value is added based on num_travaled_.
 */
void MobileRobot::odomCb(const nav_msgs::Odometry& msg) {
  //ROS_INFO("Received odometry update!");
 
  prev_motion_state_ = motion_state_;

  // Clear position and velocity vectors
  motion_state_.positions.clear();
  motion_state_.velocities.clear();
  motion_state_.accelerations.clear();
  
  // Set latest positions
  motion_state_.positions.push_back(msg.pose.pose.position.x);
  motion_state_.positions.push_back(msg.pose.pose.position.y);
  motion_state_.positions.push_back(tf::getYaw(msg.pose.pose.orientation));

  // Set latest velocities
  motion_state_.velocities.push_back(msg.twist.twist.linear.x);
  motion_state_.velocities.push_back(msg.twist.twist.linear.y);
  motion_state_.velocities.push_back(msg.twist.twist.angular.z);

  // Odometry does not have acceleration info, but
  // it would be pushed on here
  std::vector<double> a = computeAcceleration();
  for(unsigned int i=0;i<a.size();i++) 
  {
    motion_state_.accelerations.push_back(a.at(i));
  }

  motion_state_.time = num_traveled_ * CYCLE_TIME_IN_SECONDS;
  
  //ROS_INFO("Motion state: %s", utility_.toString(motion_state_).c_str());
    
  prev_t_ = ros::Time::now();
} // End updateState




/** This method is on a timer to publish the robot's latest configuration */
void MobileRobot::updateCallback(const ros::TimerEvent& e) {
  //ROS_INFO("Publishing latest MotionState");
  //std::cout<<"\nIn updatePublishTimer\n";
  
  if (pub_update_) 
  {
      pub_update_.publish(motion_state_);
      //ROS_INFO("Motion state: %s", utility_.toString(motion_state_).c_str());
  }
} // End updatePublishTimer



/** This method updates the MobileRobot's trajectory
 *   It calls calculateSpeedsAndTimes to update the robot's vectors needed to move */
void MobileRobot::updateTrajectory(const ramp_msgs::RampTrajectory msg) 
{
  //ROS_INFO("Received RampTrajectory");
  //std::cout<<"\nTrajectory: "<<utility_.toString(msg);
  
  // Update data members
  restart_        = true;
  num_traveled_   = 1;
  trajectory_     = msg;
  num_            = trajectory_.trajectory.points.size();
  t_immiColl_     = ros::Duration(0);
  
  // Update vectors for speeds and times
  if(trajectory_.trajectory.points.size() > 0) 
  {
    calculateSpeedsAndTime();
  }
  else 
  {
    ROS_INFO("Stopping at state: %s", utility_.toString(motion_state_).c_str());
    twist_.linear.x = 0;
    twist_.angular.z = 0;
    sendTwist();
    sendTwist();
    sendTwist();
  }

  //if(trajectory_.trajectory.points.size() > 2)
    //ROS_INFO("Moving on: %s", utility_.toString(trajectory_).c_str());
} // End updateTrajectory




void MobileRobot::accountForAcceleration() {
  std::vector<double> v;
  for(int i=0;i<num_-1;i++) {
    // Get the next points on the trajectory
    trajectory_msgs::JointTrajectoryPoint current = trajectory_.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint next    = trajectory_.trajectory.points.at(i+1);
  }
}



/** 
 * Calculate all the necessary values to move the robot: the linear and angular velocities as well as ending times
 * This method sets the vectors speeds, orientations, and end_times
 **/
void MobileRobot::calculateSpeedsAndTime () {
  speeds_linear_.clear();
  speeds_angular_.clear();
  end_times.clear();
  orientations_.clear();
  

  // Get the starting time
  ros::Time start_time = ros::Time::now();



  // Go through all the points of the received trajectory
  for(int i=0;i<num_-1;i++) {
    trajectory_msgs::JointTrajectoryPoint current = trajectory_.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint next    = trajectory_.trajectory.points.at(i+1);
    //std::cout<<"\nPoint "<<i;

    double vx = (next.positions.at(0) - current.positions.at(0)) / 0.1;
    double vy = (next.positions.at(1) - current.positions.at(1)) / 0.1;
    //ROS_INFO("t: %f v: %f vx: %f vy: %f", current.time_from_start.toSec(), sqrt(vx*vx+vy*vy), vx, vy);

    speeds_linear_.push_back( sqrt( pow(vx,2)
                                  + pow(vy,2) ));

    double w = utility_.findDistanceBetweenAngles(current.positions.at(2), next.positions.at(2)) / 0.1;
    speeds_angular_.push_back( w ); 

    end_times.push_back(start_time + next.time_from_start);

    orientations_.push_back( utility_.findAngleFromAToB(current.positions, next.positions) ); 
  }


  printVectors();
} // End calculateSpeedsAndTime






void MobileRobot::sendTwist() const 
{
  //ROS_INFO("In MobileRobot::sendTwist()");
  pub_twist_.publish(twist_); 

  // If we have the simulation up, publish to cmd_vel
  if(sim_) 
  {
    pub_cmd_vel_.publish(twist_);
  }
  
  //ROS_INFO("Exiting MobileRobot::sendTwist()");
}


void MobileRobot::sendTwist(const geometry_msgs::Twist t) const 
{
  pub_twist_.publish(t); 

  // If we have the simulation up, publish to cmd_vel
  if(sim_) 
  {
    pub_cmd_vel_.publish(t);
  }
}


/** This method prints out the information vectors */
void MobileRobot::printVectors() const 
{
    
  std::cout<<"\nspeeds_linear size: "<<speeds_linear_.size();
  std::cout<<"\nspeeds_linear: [";
  for(unsigned int i=0;i<speeds_linear_.size()-1;i++) 
  {
    std::cout<<speeds_linear_.at(i)<<", ";
  }
  std::cout<<speeds_linear_.at(speeds_linear_.size()-1)<<"]";
  
  std::cout<<"\nspeeds_angular size: "<<speeds_angular_.size();
  std::cout<<"\nspeeds_angular_: [";
  for(unsigned int i=0;i<speeds_angular_.size()-1;i++) 
  {
    std::cout<<speeds_angular_.at(i)<<", ";
  }
  std::cout<<speeds_angular_.at(speeds_angular_.size()-1)<<"]";

  std::cout<<"\nend_times size: "<<end_times.size();
  std::cout<<"\nend_times: [";
  for(unsigned int i=0;i<end_times.size()-1;i++) 
  {
    std::cout<<end_times.at(i)<<", ";
  }
  std::cout<<end_times.at(end_times.size()-1)<<"]";

  std::cout<<"\norientations_ size: "<<orientations_.size();
  std::cout<<"\norientations_: [";
  for(unsigned int i=0;i<orientations_.size()-1;i++) 
  {
    std::cout<<orientations_.at(i)<<", ";
  }
  std::cout<<orientations_.at(orientations_.size()-1)<<"]";
} // End printVectors



/** Returns true if there is imminent collision */
const bool MobileRobot::checkImminentCollision()  
{
  bool result = false;
  ros::param::get("imminent_collision", result);
  if(result)
  {
    ROS_ERROR("Imminent Collision exists! Stopping robot, initial_theta_: %f", initial_theta_);
  }
  //ROS_INFO("Imminent Collision: %s", result ? "True" : "False");
  return result;
} // End checkImminentCollision






/** This method moves the robot along trajectory_ */
void MobileRobot::moveOnTrajectory() 
{
  restart_ = false;
  ros::Rate r(20);

  // Execute the trajectory
  while( (num_traveled_+1) < num_) 
  {
    //ROS_INFO("num_traveled_: %i/%i", num_traveled_, num_);
    //ROS_INFO("At state: %s", utility_.toString(motion_state_).c_str());
    restart_ = false;
 

    // Force a stop until there is no imminent collision
    if(check_imminent_coll_)
    {
      ros::Time t_startIC = ros::Time::now();
      while(checkImminentCollision()) 
      {
        sendTwist(zero_twist_);
        ros::spinOnce();
      }
      t_immiColl_ += ros::Time::now() - t_startIC;
    }
    //std::cout<<"\nt_immiColl: "<<t_immiColl_;

    
    // If a new trajectory was received, restart the outer while 
    if(restart_) 
    {
      continue;
    }

    // Move to the next point
    ros::Time g_time = end_times.at(num_traveled_) + t_immiColl_;
    while(ros::ok() && ros::Time::now() < g_time && !checkImminentCollision()) 
    {
      twist_.linear.x   = speeds_linear_.at(num_traveled_);
      twist_.angular.z  = speeds_angular_.at(num_traveled_);
 
      // When driving straight, adjust the angular speed 
      // to maintain orientation
      // TODO: Works with Bezier curve?
      if(fabs(twist_.linear.x) > 0.0f && fabs(twist_.angular.z) < 0.0001f) 
      {
        //ROS_INFO("initial_theta_: %f motion_state_.positions.at(2): %f", initial_theta_, motion_state_.positions.at(2));
        float actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
        float dist = utility_.findDistanceBetweenAngles(actual_theta, orientations_.at(num_traveled_));
        //ROS_INFO("actual_theta: %f orientations[%i]: %f dist: %f", actual_theta, num_traveled_, 
            //orientations_.at(num_traveled_), dist);
        twist_.angular.z = dist;
      }

      ROS_INFO("twist.linear.x: %f twist.angular.z: %f", twist_.linear.x, twist_.angular.z);

      // Send the twist_message to move the robot
      sendTwist();
      
      // Sleep
      r.sleep();
      
      // Spin to check for updates
      ros::spinOnce();
    } // end while (move to the next point)
    
    // If a new trajectory was received, restart the outer while 
    if(restart_) 
    {
      continue;
    }

    // Increment num_traveled
    num_traveled_++;

    // Spin once to check for updates in the trajectory
    ros::spinOnce();
  } // end while

  // Check that we moved on a trajectory
  if(num_traveled_ > 1)
  {
    // Stops the wheels
    twist_.linear.x = 0;
    twist_.angular.z = 0;
    sendTwist();
    sendTwist();

    // Set num and num_traveled so we don't come back into this if-block each time
    num_ = 0;
    num_traveled_ = 0;
  } // end if finished a trajectory
} // End moveOnTrajectory


