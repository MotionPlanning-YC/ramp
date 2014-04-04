#include "collision_detection.h"


CollisionDetection::CollisionDetection() : h_traj_req_(0), predictionTime_(ros::Duration(5)) {}

CollisionDetection::~CollisionDetection() {
  if(h_traj_req_ != 0) {
    delete h_traj_req_;
    h_traj_req_ = 0;
  }
}

void CollisionDetection::init(ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler((const ros::NodeHandle&)h);
  setOb_T_w_b(id);
}




/** Returns true if trajectory_ is in collision with any of the objects */
const CollisionDetection::QueryResult CollisionDetection::perform() const {
  CollisionDetection::QueryResult result;

  // Duration for predicting the object trajectories
  ros::Duration d(5);
  
  // Predict the obstacle's trajectory
  ramp_msgs::Trajectory ob_trajectory = getPredictedTrajectory(obstacle_); 
  
  // Query for collision
  CollisionDetection::QueryResult q = query(ob_trajectory);
  
  // If collision, set result to q
  if(q.collision_) {
    result = q;
  }

  return result;  
} //End perform



/** Transformation matrix of obstacle robot from base frame to world frame*/
void CollisionDetection::setOb_T_w_b(int id) {

  if(id == 1) {
    tf::Vector3 pos(3.5f, 2.f, 0);
    ob_T_w_b_.setOrigin(pos);
    ob_T_w_b_.setRotation(tf::createQuaternionFromYaw(PI));
  }

  else {
    tf::Vector3 pos(0.f, 2.f, 0.f);
    ob_T_w_b_.setRotation(tf::createQuaternionFromYaw(0));
    ob_T_w_b_.setOrigin(pos);
  }
} // End setOb_T_w_b



/** 
 * This method returns true if there is collision between trajectory_ and the obstacle's trajectory, false otherwise 
 * The robots are treated as circles for simple collision detection
 */
const CollisionDetection::QueryResult CollisionDetection::query(const ramp_msgs::Trajectory ob_trajectory) const {
  //std::cout<<"\nQuery on "<<utility.toString(trajectory_)<<" \n*******and*******\n"<<utility.toString(ob_trajectory);
  CollisionDetection::QueryResult result;

  if(ob_trajectory.trajectory.points.size() == 0) {
    if(id == 0)
      std::cout<<"\nObstacle 1 has no trajectory!\n";
    else  
      std::cout<<"\nObstacle 0 has no trajectory!\n";
  }
  
  // For every 3 points, check circle detection
  float radius = 0.45f;
  for(unsigned int i=0;i<trajectory_.trajectory.points.size();i+=3) {
    
    // Get the ith point on the trajectory
    trajectory_msgs::JointTrajectoryPoint p_i = trajectory_.trajectory.points.at(i);

    // ***Test position i for collision against some points on obstacle's trajectory***
    // Obstacle trajectory should already be in world coordinates!
    for(int j = (i>0 ? i-1 : 0) ;j<i+1 && j<ob_trajectory.trajectory.points.size();j++) {

      // Get the jth point of the obstacle's trajectory
      trajectory_msgs::JointTrajectoryPoint p_ob  = ob_trajectory.trajectory.points.at(j);

      // Get the distance between the centers
      float dist = sqrt( pow(p_i.positions.at(0) - p_ob.positions.at(0),2) + pow(p_i.positions.at(1) - p_ob.positions.at(1),2) );
        
      // If the distance between the two centers is less than the sum of the two radii, 
      // there is collision
      if( dist <= radius*2 ) {
        result.collision_ = true;
        result.time_until_collision_ = p_i.time_from_start.toSec();
        j = i+1;
        i = trajectory_.trajectory.points.size();
      } // end if
    } // end for
  } // end for

  return result;
} //End query







/** This method determines what type of motion an obstacle has */
const MotionType CollisionDetection::findMotionType(const ramp_msgs::Obstacle ob) const {
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
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.1) {
    result = MotionType::Rotation;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.1) {
    result = MotionType::TranslationAndRotation;
  } //end else if

  // Else, there is no motion
  else {
    result = MotionType::None;
  }

  return result;
} // End findMotionType




/** This method returns the predicted trajectory for an obstacle for the future duration d 
 * TODO: Remove Duration parameter and make the predicted trajectory be computed until robot reaches bounds of environment */
const ramp_msgs::Trajectory CollisionDetection::getPredictedTrajectory(const ramp_msgs::Obstacle ob) const {
  ramp_msgs::Trajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(ob);
  

  // Now build a Trajectory Request 
  ramp_msgs::TrajectoryRequest tr;
    tr.request.path = getObstaclePath(ob, motion_type);
    tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
    tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
    tr.request.v_start.push_back(ob.odom_t.twist.twist.angular.z);
    tr.request.resolutionRate = 5;

  // Get trajectory
  if(h_traj_req_->request(tr)) {
    result = tr.response.trajectory;
  }

  if(tr.response.trajectory.trajectory.points.size() == 0) {
    std::cout<<"\nObstacle motion type: ";
    if(motion_type == MotionType::None)
      std::cout<<"None";
    else if(motion_type == MotionType::Rotation)
      std::cout<<"Rotation";

    std::cout<<"\nObstacle path: "<<utility.toString(tr.request.path);
  }

  return result;
} // End getPredictedTrajectory






/** 
 *  This method returns a prediction for the obstacle's path. 
 *  The path is based on 1) the type of motion the obstacle currently has
 *  2) the duration that we should predict the motion for 
 */
const ramp_msgs::Path CollisionDetection::getObstaclePath(const ramp_msgs::Obstacle ob, const MotionType mt) const {
  ramp_msgs::Path result;

  std::vector<ramp_msgs::KnotPoint> path;

  // Create and initialize the first point in the path
  ramp_msgs::KnotPoint start;
  start.configuration.ranges = utility.ranges_;
  start.configuration.K.push_back(ob.odom_t.pose.pose.position.x);
  start.configuration.K.push_back(ob.odom_t.pose.pose.position.y);
  start.configuration.K.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

  // Push the first point onto the path
  path.push_back(start);

  /** Find the ending configuration for the predicted trajectory based on motion type */
  // If translation
  if(mt == MotionType::Translation) {

    // Create the Goal Knotpoint
    ramp_msgs::KnotPoint goal;
    goal.configuration.ranges = utility.ranges_;

    // Get the goal position in the base frame
    tf::Vector3 ob_goal_b(start.configuration.K.at(0) + (ob.odom_t.twist.twist.linear.x * predictionTime_.toSec()), 
                          start.configuration.K.at(1) + (ob.odom_t.twist.twist.linear.y * predictionTime_.toSec()),
                          0);

    // Convert the goal position to world coordinates
    tf::Vector3 goal_w = ob_T_w_b_ * ob_goal_b;
    
    // Push on the world coordinates
    goal.configuration.K.push_back(goal_w.getX());
    goal.configuration.K.push_back(goal_w.getY());
    goal.configuration.K.push_back(start.configuration.K.at(2));
    
    // Push goal onto the path
    path.push_back(goal);
  } // end if translation

  // If translation and rotation
  else if(mt == MotionType::TranslationAndRotation) {

    // Find the linear and angular velocity vectors
    tf::Vector3 v_linear;
    tf::Vector3 v_angular;
    tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);
    tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

    // Find magnitudes of velocity vectors and radius r
    float v = sqrt( tf::tfDot(v_linear, v_linear)   );
    float w = sqrt( tf::tfDot(v_angular, v_angular) );
    float r = v / w;
    //std::cout<<"\nv: "<<v<<" w: "<<w<<" r: "<<r;

    // Find the angle from base origin to robot position for polar coordinates
    tf::Vector3 a(0, 0, 0);
    tf::Vector3 b(ob.odom_t.pose.pose.position.x, ob.odom_t.pose.pose.position.y, 0);
    float polar_theta_r = utility.findAngleFromAToB(a, b);

    // Find the radius from base origin to robot position for polar coordinates
    float polar_r_r = sqrt(pow(start.configuration.K.at(0),2) + pow(start.configuration.K.at(1), 2));
    
    //std::cout<<"\npolar_theta_r: "<<polar_theta_r;
    //std::cout<<"\npolar_r_r: "<<polar_r_r;

    // Generate intermediate points for circlular motion
    for(float i=0.25f;i<predictionTime_.toSec();i+=0.25f) {

      // Create new knot point for the path
      ramp_msgs::KnotPoint temp;
      temp.configuration.ranges = utility.ranges_;

      // Get the polar coordinates theta value in base frame 
      float theta_prime_r = utility.displaceAngle(polar_theta_r, w*i);

      // Convert from polar to cartesian in base frame
      float x_prime_r = polar_r_r * cos(theta_prime_r);
      float y_prime_r = polar_r_r * sin(theta_prime_r);
      float theta_r = utility.displaceAngle(start.configuration.K.at(2), w*i);
      //std::cout<<"\nx_prime_r: "<<x_prime_r<<" y_prime_r: "<<y_prime_r<<" theta_r: "<<theta_r;

      // Now convert position in base frame to world coordinates
      tf::Vector3 p_r(x_prime_r, y_prime_r, 0);
      tf::Vector3 p_w = ob_T_w_b_ * p_r;

      // Push the values onto temp
      temp.configuration.K.push_back(p_w.getX());
      temp.configuration.K.push_back(p_w.getY());
      temp.configuration.K.push_back(utility.displaceAngle(theta_r, tf::getYaw(ob_T_w_b_.getRotation())));
      
      // Push temp onto path
      path.push_back(temp);
    } // end for
  } // end else if
  



  // If rotation
  // Since our robot models are circles, rotation is the same as no movement
  else if(mt == MotionType::Rotation || mt == MotionType::None) {
    
    // Create the Goal Knotpoint
    ramp_msgs::KnotPoint goal;
    goal.configuration.ranges = utility.ranges_;
    tf::Vector3 ob_goal(start.configuration.K.at(0), start.configuration.K.at(1), 0);
    tf::Vector3 goal_w = ob_T_w_b_ * ob_goal;

    
    // Push on the world coordinates
    goal.configuration.K.push_back(goal_w.getX());
    goal.configuration.K.push_back(goal_w.getY());
    goal.configuration.K.push_back(start.configuration.K.at(2));

    path.push_back(goal);
  } // end if self-rotation, none


  // Convert the starting point in path to world coordinates
  tf::Vector3 start_w(start.configuration.K.at(0), start.configuration.K.at(1), 0);
  start_w = ob_T_w_b_ * start_w;
  path.at(0).configuration.K.at(0) = start_w.getX();
  path.at(0).configuration.K.at(1) = start_w.getY();
  path.at(0).configuration.K.at(2) = utility.displaceAngle(start.configuration.K.at(2), tf::getYaw(ob_T_w_b_.getRotation()));


  result = utility.getPath(path);
  return result; 
}



