
#include "mobile_robot.h"

const std::string MobileRobot::TOPIC_STR_PHIDGET_MOTOR="PhidgetMotor";
const std::string MobileRobot::TOPIC_STR_ODOMETRY="odometry";
const std::string MobileRobot::TOPIC_STR_UPDATE="update";
const std::string MobileRobot::TOPIC_STR_TWIST="twist";
const float BASE_WIDTH=0.2413;

const float timeNeededToTurn = 2.5; 



MobileRobot::MobileRobot() : restart_(false), num_(0), num_prev_(0), num_traveled_(0), k_dof_(3), h_traj_req_(0) { 
  for(unsigned int i=0;i<k_dof_;i++) {
    motion_state_.positions.push_back(0);
    motion_state_.velocities.push_back(0);
    motion_state_.accelerations.push_back(0);
    motion_state_.jerks.push_back(0);
  }

  prev_motion_state_ = motion_state_;
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


const std::vector<float> MobileRobot::computeAcceleration() const {
  std::vector<float> result;

  for(unsigned int i=0;i<k_dof_;i++) {
    float a = (motion_state_.velocities.at(i) - prev_motion_state_.velocities.at(i)) 
              / (ros::Time::now() - prev_t_).toSec();
    result.push_back(a);
  }

  return result;
}

/** This is a callback for receiving odometry from the robot and sets the configuration of the robot */
void MobileRobot::updateState(const nav_msgs::Odometry& msg) {
  //std::cout<<"\nReceived odometry update\n";
  
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
  std::vector<float> a = computeAcceleration();
  for(unsigned int i=0;i<a.size();i++) {
    motion_state_.accelerations.push_back(a.at(i));
  }
    
  prev_t_ = ros::Time::now();
} // End updateState




/** This method is on a timer to publish the robot's latest configuration */
void MobileRobot::updatePublishTimer(const ros::TimerEvent&) {
  //std::cout<<"\nIn updatePublishTimer\n";
    
  if (pub_update_) {
      pub_update_.publish(motion_state_);
  }
} // End updatePublishTimer



/** This method updates the MobileRobot's trajectory
 *   It calls calculateSpeedsAndTimes to update the robot's vectors needed to move */
void MobileRobot::updateTrajectory(const ramp_msgs::Trajectory msg) {
  //std::cout<<"\nIn updateTrajectory!\n";
  //std::cout<<"\nNew Trajectory first point: "<<utility_.toString(msg.trajectory.points.at(0));
  
  // Update data members
  restart_        = true;
  num_traveled_   = 0;
  trajectory_     = msg;
  num_prev_       = num_;
  num_            = trajectory_.trajectory.points.size();
  
  // Update vectors for speeds and times
  calculateSpeedsAndTime();
} // End updateTrajectory



/** 
 * Calculate all the necessary values to move the robot: 
 * the linear and angular velocities as well as ending times
 * This method sets the vectors speeds, orientations, and end_times
 **/
void MobileRobot::calculateSpeedsAndTime () {
  orientations_.clear();
  speeds.clear();
  end_times.clear();
  
  // Get number of knot points
  int num = trajectory_.trajectory.points.size(); 

  // Get the starting time
  ros::Time start_time = ros::Time::now();
  std::cout<<"\nstart_time: "<<start_time.toSec();

  // We go through all the points
  for(int i=0;i<num_-1;i++) {

    // Get the next points on the trajectory
    trajectory_msgs::JointTrajectoryPoint current = trajectory_.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint next    = trajectory_.trajectory.points.at(i+1);


    // Push on the linear speed for the waypoint
    // Find the norm of the velocity vector
    speeds_linear_.push_back( sqrt(pow(current.velocities.at(0),2)
                           + pow(current.velocities.at(1),2) ));

    speeds_angular_.push_back( current.velocities.at(2) );

    // Find which knot point is next
    /*int kp, kp_prev=0;
    for(kp=0;kp<trajectory_.index_knot_points.size();kp++) {
      //std::cout<<"\ntrajectory_.index_knot_points.at(kp): "<<trajectory_.index_knot_points.at(kp);
      if(i < trajectory_.index_knot_points.at(kp)) {
        kp_prev = trajectory_.index_knot_points.at(kp-1);
        kp = trajectory_.index_knot_points.at(kp);
        break;
      }
    }
    //std::cout<<"\ni: "<<i<<" kp: "<<kp<<"kp_prev: "<<kp_prev<<"\n";

    // Push on the orientation needed to move 
    // from point i to point i+1
    //std::cout<<"\nCurrent: "<<utility_.toString(current);
    //std::cout<<"\nNext: "<<utility_.toString(trajectory_.trajectory.points.at(kp));
    double theta = utility_.findAngleFromAToB(trajectory_.trajectory.points.at(kp_prev), trajectory_.trajectory.points.at(kp));
    if(theta == 0 && utility_.planarDistance(trajectory_.trajectory.points.at(kp_prev).positions, trajectory_.trajectory.points.at(kp).positions) < 0.01) {
      orientations_.push_back(trajectory_.trajectory.points.at(kp).positions.at(2));
    }
    else 
      orientations_.push_back(theta);*/

    //std::cout<<"\norientation: "<<orientations_.at(i)<<"\n";
    
    // Calculate the ending time for each waypoints
    ros::Duration d(next.time_from_start.toSec());
    ros::Time t = start_time + d;
    end_times.push_back(t);
    //std::cout<<"\nnext.time_from_start: "<<next.time_from_start;
    std::cout<<"\nd: "<<d.toSec();
    std::cout<<"\nstart + d: "<< t;
  } 
  
  printVectors();
} // End calculateSpeedsAndTime






void MobileRobot::sendTwist() const {
  pub_twist_.publish(twist_); 
}


void MobileRobot::sendTwist(const geometry_msgs::Twist t) const {
  pub_twist_.publish(t); 
}


/** This method prints out the information vectors */
void MobileRobot::printVectors() const {
    
  std::cout<<"\nspeeds: [";
  for(unsigned int i=0;i<speeds_linear_.size()-1;i++) {
    std::cout<<speeds_linear_.at(i)<<", ";
  }
  std::cout<<speeds_linear_.at(speeds_linear_.size()-1)<<"]";

  std::cout<<"\nend_times: [";
  for(unsigned int i=0;i<end_times.size()-1;i++) {
    std::cout<<end_times.at(i)<<", ";
  }
  std::cout<<end_times.at(end_times.size()-1)<<"]";

  std::cout<<"\norientations_: [";
  for(unsigned int i=0;i<speeds_angular_.size()-1;i++) {
    std::cout<<speeds_angular_.at(i)<<", ";
  }
  std::cout<<speeds_angular_.at(speeds_angular_.size()-1)<<"]";
} // End printVectors



/** Returns true if there is imminent collision */
const bool MobileRobot::checkImminentCollision() const {
  bool result;
  ros::param::get("imminent_collision", result);
  return result;
} // End checkImminentCollision





/** 
 * Returns true if the robot has orientation to move 
 * from point i to point i+1
 **/
const bool MobileRobot::checkOrientation(const int i, const bool simulation) const {
  //std::cout<<"\nIn checkOrientation";
  
  // Find which knot point is next
  int kp;
  for(kp=0;kp<trajectory_.index_knot_points.size();kp++) {
    //std::cout<<"\ntrajectory_.index_knot_points.at(kp): "<<trajectory_.index_knot_points.at(kp);
    if(i < trajectory_.index_knot_points.at(kp)) {
      kp = trajectory_.index_knot_points.at(kp);
      break;
    }
  }
  /*std::cout<<"\ni: "<<i<<" kp: "<<kp;
  std::cout<<"\n"<<utility_.toString(trajectory_.trajectory.points.at(i));
  std::cout<<"\n"<<utility_.toString(trajectory_.trajectory.points.at(kp));*/

  float t_or = utility_.findAngleFromAToB(trajectory_.trajectory.points.at(i),
                                          trajectory_.trajectory.points.at(kp));

  float actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
  float diff = fabs(utility_.findDistanceBetweenAngles(actual_theta, t_or));

  /*std::cout<<"\nactual_theta: "<<actual_theta;
  std::cout<<"\norientations_.at("<<i<<"): "<<orientations_.at(i)<<"\n";
  std::cout<<"\ndiff: "<<diff;*/
  
  if(t_or == 0 && utility_.planarDistance(trajectory_.trajectory.points.at(i).positions, trajectory_.trajectory.points.at(kp).positions) < 0.01) {
    return true;
  }

  else if( (!simulation && diff > PI/12) || (simulation && diff > PI/24) ) {
    return false;
  }

  return true;
} // End checkOrientation





/** This method returns a rotation trajectory for the robot */
const ramp_msgs::Trajectory MobileRobot::getRotationTrajectory() const {
  //std::cout<<"\nIn getRotationTrajectory\n";
  ramp_msgs::Trajectory result;

  // Build a trajectory request
  ramp_msgs::TrajectoryRequest tr; 
  ramp_msgs::KnotPoint temp1;
  ramp_msgs::KnotPoint temp2;
  
  
  temp1.motionState = motion_state_;
  temp1.motionState.positions.at(2) = 0;
  
  temp2.motionState = motion_state_;

  double actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
  temp2.motionState.positions.at(2) = utility_.findDistanceBetweenAngles(actual_theta, orientations_.at(num_traveled_));
  //std::cout<<"\nactual_theta: "<<actual_theta<<" orientations_.at(num_traveled_): "<<orientations_.at(num_traveled_);

  tr.request.path.points.push_back(temp1);
  tr.request.path.points.push_back(temp2);


  //std::cout<<"\nRotation path: "<<utility_.toString(tr.request.path);
      
  tr.request.rotational = true;

  // Send request
  if(h_traj_req_->request(tr)) {
    result = tr.response.trajectory;  
  }

  //std::cout<<"\nRotation trajectory: "<<utility_.toString(result);
  return result;
} // End getRotationTrajectory






void MobileRobot::moveOnTrajectoryRot(const ramp_msgs::Trajectory traj, bool simulation) {
  //std::cout<<"\n**************In moveOnTrajectoryRot****************\n";
  //std::cout<<"\ntraj.size(): "<<traj.trajectory.points.size();

  ros::Rate r(50);
  ros::Time start = ros::Time::now();

  double goal_theta = traj.trajectory.points.at( traj.trajectory.points.size()-1 ).positions.at(2);

  // Go through each point
  for(int i=0;i<traj.trajectory.points.size();i++) {

    // Set Twist values
    twist_.linear.x  = 0;
    twist_.angular.z = traj.trajectory.points.at(i).velocities.at(2);

    // Set goal time
    ros::Time g_time = start + traj.trajectory.points.at(i).time_from_start;

    // Send twist commands until goal time
    while(ros::Time::now() < g_time) {
      sendTwist();
      if(simulation) pub_cmd_vel_.publish(twist_);

      r.sleep();
      ros::spinOnce();

      // Check if the latest trajectory is different
      if(restart_ && speeds.size() != (num_prev_-1)) {
        i = traj.trajectory.points.size();
        break;
      } // end if
  
      // TODO: make actual_theta a class member, set in odometry callback
      double actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
      if( fabs(utility_.findDistanceBetweenAngles( actual_theta, goal_theta )) <= PI/12) {
        break;
      }
    } // end while
  } // end for

  //std::cout<<"\nLeaving moveOnTrajectoryRot\n";
} // End moveOnTrajectoryRot






/** This method moves the robot along trajectory_ */
void MobileRobot::moveOnTrajectory(bool simulation) {
  restart_ = false;
  ros::Rate r(50);


  // Execute the trajectory
  while( (num_traveled_+1) < num_) { 
    std::cout<<"\nnum_traveled: "<<num_traveled_<<"\n";
    restart_ = false;
    

    // Force a stop until there is no imminent collision
    while(checkImminentCollision()) {
      ros::spinOnce();
    }

    // Check that we are at the correct orientation
    // If not, rotate
    // Because the last point is the goal
    /*if(!checkOrientation(num_traveled_, simulation)) {

      // Get rotation trajectory and move on it
      ramp_msgs::Trajectory rot_t = getRotationTrajectory();

      //std::cout<<"\nRotation Trajectory: "<<utility_.toString(rot_t);

      moveOnTrajectoryRot(rot_t, simulation);
      
      ros::spinOnce();
    } // end if   
    
    // Check if a new trajectory has been received before we go to next point
    if(restart_) {
      break;
    }*/
    
    
    
    // Move to the next point
    ros::Time g_time = end_times.at(num_traveled_);
    while(ros::ok() && ros::Time::now() < g_time) {
    
      twist_.linear.x   = speeds_linear_.at(num_traveled_);
      twist_.angular.z  = speeds_angular_.at(num_traveled_);
    
      //if(!simulation) {

        // When driving straight, adjust the angular speed 
        // to maintain orientation
        // TODO: Works with Bezier curve?
        /*if(fabs(twist_.linear.x) > 0.0f) {
          
          //std::cout<<"\ninitial_theta_: "<<initial_theta_;
          float actual_theta = utility_.displaceAngle(initial_theta_, motion_state_.positions.at(2));
          float dist = utility_.findDistanceBetweenAngles(actual_theta, orientations_.at(num_traveled_));
          //std::cout<<"\nactual theta: "<<actual_theta;
          //std::cout<<"\norientations_.at("<<num_traveled_<<"): "<<orientations_.at(num_traveled_);
          //std::cout<<"\ndist: "<<dist;
          twist_.angular.z = dist/2;
        }*/
      //}
    
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
    num_traveled_++;

    // Spin once to check for updates in the trajectory
    ros::spinOnce();
  } // end while

  // Stops the wheels
  twist_.linear.x = 0;
  twist_.angular.z = 0;
  sendTwist();
} // End moveOnTrajectory


