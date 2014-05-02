
#include "corobot.h"

const std::string Corobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string Corobot::TOPIC_STR_ODOMETRY="odometry";
const std::string Corobot::TOPIC_STR_UPDATE="update";
const std::string Corobot::TOPIC_STR_TWIST="twist";
const float BASE_WIDTH=0.2413;

const float timeNeededToTurn = 2.5; 



Corobot::Corobot() : restart_(false), num(0), num_traveled(0), k_dof_(3), h_traj_req_(0) { 
  for(unsigned int i=0;i<k_dof_;i++) {
    motion_state_.positions.push_back(0);
    motion_state_.velocities.push_back(0);
    motion_state_.accelerations.push_back(0);
    motion_state_.jerks.push_back(0);
  }

  prev_motion_state_ = motion_state_;
}


Corobot::~Corobot() {
  if(h_traj_req_ != 0) {
    delete h_traj_req_;
    h_traj_req_ = 0;
  }
}


void Corobot::init(ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler((const ros::NodeHandle&)h);
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


const std::vector<float> Corobot::computeAcceleration() const {
  std::vector<float> result;

  for(unsigned int i=0;i<k_dof_;i++) {
    float a = (motion_state_.velocities.at(i) - prev_motion_state_.velocities.at(i)) 
              / (ros::Time::now() - prev_t_).toSec();
    result.push_back(a);
  }

  return result;
}

/** This is a callback for receiving odometry from the robot and sets the configuration of the robot */
void Corobot::updateState(const nav_msgs::Odometry& msg) {
  
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
  // computeAcceleration
    
  prev_t_ = ros::Time::now();
} // End updateState




/** This method is on a timer to publish the robot's latest configuration */
void Corobot::updatePublishTimer(const ros::TimerEvent&) {
    
  if (pub_update_) {
      pub_update_.publish(motion_state_);
  }
} // End updatePublishTimer



/** This method updates the Corobot's trajectory
 *   It calls calculateSpeedsAndTimes to update the robot's vectors needed to move */
void Corobot::updateTrajectory(const ramp_msgs::Trajectory msg) {
  //std::cout<<"\nIn updateTrajectory!\n";
  
  //std::cout<<"\nNew Trajectory: "<<utility_.toString(msg);
  
  // Update data members
  restart_      = true;
  num_traveled  = 0;
  trajectory_   = msg;
  num           = trajectory_.trajectory.points.size();

  // Update vectors for speeds and times
  calculateSpeedsAndTime();
} // End updateTrajectory



/** 
 * Calculate all the necessary values to move the robot: 
 * the linear and angular velocities as well as ending times
 * This method sets the vectors speeds, orientations, and end_times
 **/
void Corobot::calculateSpeedsAndTime () {
  orientations_.clear();
  speeds.clear();
  end_times.clear();
  
  // Get number of knot points
  int num = trajectory_.trajectory.points.size(); 

  // Get the starting time
  ros::Time start_time = ros::Time::now();

  // We go through all the waypoints
  for(int i=0;i<num-1;i++) {

    // Get the next points on the trajectory
    trajectory_msgs::JointTrajectoryPoint current = trajectory_.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint next    = trajectory_.trajectory.points.at(i+1);


    // Push on the linear speed for the waypoint
    // Find the norm of the velocity vector
    speeds.push_back( sqrt(pow(current.velocities.at(0),2)
                           + pow(current.velocities.at(1),2) ));


    // Push on the orientation needed to move 
    // from point i to point i+1
    //std::cout<<"\nCurrent: ("<<current.positions.at(0)<<", "<<current.positions.at(1)<<")";
    //std::cout<<"\nNext: ("<<next.positions.at(0)<<", "<<next.positions.at(1)<<")";
    orientations_.push_back(utility_.findAngleFromAToB(current, next));
    //std::cout<<"\norientation: "<<orientations_.at(i)<<"\n";
    
    // Calculate the ending time for each waypoints
    end_times.push_back(start_time + next.time_from_start );
  } 
  
  //printVectors();
}


void Corobot::sendTwist() const {
  pub_twist_.publish(twist_); 
}


void Corobot::sendTwist(const geometry_msgs::Twist t) const {
  pub_twist_.publish(t); 
}


/** This method prints out the information vectors */
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

  std::cout<<"\norientations_: [";
  for(unsigned int i=0;i<orientations_.size()-1;i++) {
    std::cout<<orientations_.at(i)<<", ";
  }
  std::cout<<orientations_.at(orientations_.size()-1)<<"]";
} // End printVectors



/** Returns true if there is imminent collision */
const bool Corobot::checkImminentCollision() const {
  bool result;
  ros::param::get("imminent_collision", result);
  return result;
}


/** 
 * Returns true if the robot has orientation to move 
 * from point i to point i+1
 **/
const bool Corobot::checkOrientation(const int i, const bool simulation) const {
  //std::cout<<"\nIn checkOrientation";
  
  float actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
  //std::cout<<"\nactual_theta: "<<actual_theta;
  //std::cout<<"\norientations_.at("<<i<<"): "<<orientations_.at(i)<<"\n";
  float diff = fabs(utility_.findDistanceBetweenAngles(actual_theta, orientations_.at(i)));
  //std::cout<<"\ndiff: "<<diff;

  if( (!simulation && diff > PI/12) || (simulation && diff > PI/6) ) {
    return false;
  }

  return true;
}


/** This method returns a rotation trajectory for the robot */
const ramp_msgs::Trajectory Corobot::getRotationTrajectory() const {
  //std::cout<<"\nIn getRotationTrajectory\n";
  ramp_msgs::Trajectory result;

  ramp_msgs::TrajectoryRequest tr; 
  ramp_msgs::KnotPoint temp1;
  ramp_msgs::KnotPoint temp2;
  temp1.motionState = motion_state_;
  temp2.motionState = motion_state_;
  temp2.motionState.positions.at(2) = orientations_.at(num_traveled);

  tr.request.path.points.push_back(temp1);
  tr.request.path.points.push_back(temp2);
      
  if(h_traj_req_->request(tr)) {
    result = tr.response.trajectory;  
  }

  //std::cout<<"\nRotation trajectory: "<<utility_.toString(result);
  return result;
}


void Corobot::moveOnTrajectoryRot(const ramp_msgs::Trajectory traj, bool simulation) {
  std::cout<<"\n**************In moveOnTrajectoryRot****************\n";

  ros::Rate r(50);

  ros::Time start = ros::Time::now();
  for(int i=0;i<traj.trajectory.points.size();i++) {
    twist_.linear.x  = 0;
    twist_.angular.z = traj.trajectory.points.at(i).velocities.at(2);

    ros::Time g_time = start + traj.trajectory.points.at(i).time_from_start;

    while(ros::Time::now() < g_time) {
      sendTwist();
      if(simulation) pub_cmd_vel_.publish(twist_);

      r.sleep();
      ros::spinOnce();
    }
  }
}


/** This method moves the robot along trajectory_ */
void Corobot::moveOnTrajectory(bool simulation) {
  restart_ = false;
  ros::Rate r(50);
  ros::Rate d(0.5);
    


  // Execute the trajectory
  while( (num_traveled+1) < num) { 
    //std::cout<<"\nnum_traveled: "<<num_traveled;
    restart_ = false;
    
    // Force a stop until there is no imminent collision
    while(checkImminentCollision()) {
      ros::spinOnce();
    }

    // Check that we are at the correct orientation
    // If not, rotate
    // Because the last point is the goal
    if(!checkOrientation(num_traveled, simulation)) {
      ramp_msgs::Trajectory temp = trajectory_;
      ramp_msgs::Trajectory rot_t = getRotationTrajectory();
      updateTrajectory(rot_t);
      moveOnTrajectoryRot(rot_t, simulation);
    
      ros::spinOnce();
    }    
    
    
    // Set velocities
    twist_.linear.x  = speeds.at(num_traveled);

    // Move to the next point
    ros::Time g_time = end_times.at(num_traveled);
    while(ros::ok() && ros::Time::now() < g_time) {
    
      // Adjust the angular speed to correct errors in turning
      // Commented out because it was producing erratic driving
      // Should be fixed at some point
      if(fabs(twist_.linear.x) > 0.0f && fabs(twist_.angular.z) < 0.15) {
        //std::cout<<"\ninitial_theta_: "<<initial_theta_;
        float actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
        float dist = utility_.findDistanceBetweenAngles(actual_theta, orientations_.at(num_traveled));
        //std::cout<<"\nactual theta: "<<actual_theta;
        //std::cout<<"\norientations_.at("<<num_traveled<<"): "<<orientations_.at(num_traveled);
        //std::cout<<"\ndist: "<<dist;
        //twist_.angular.z = -1*dist;
      }
    
      //std::cout<<"\ntwist_linear: "<<twist_.linear.x;
      //std::cout<<"\ntwist_angular: "<<twist_.angular.z<<"\n";

      // Send the twist_message to move the robot
      sendTwist();

      // If we have the simulation up, publish to cmd_vel
      if(simulation) pub_cmd_vel_.publish(twist_);
      
      // Sleep
      r.sleep();
      
      // Spin to check for updates
      ros::spinOnce();

      // Check if a new trajectory has been received before we reach next point
      if(restart_) {
        break;
      }
    } // end while (move to the next point
    
    // If a new trajectory was received, restart the outer while 
    if(restart_) {
      continue;
    }

    // Increment num_traveled
    num_traveled++;

    // Spin once to check for updates in the trajectory
    ros::spinOnce();
  } // end while

  // Stops the wheels
  twist_.linear.x = 0;
  twist_.angular.z = 0;
  sendTwist();
} // End moveOnTrajectory


