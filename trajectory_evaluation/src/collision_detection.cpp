#include "collision_detection.h"

CollisionDetection::CollisionDetection() : h_traj_req_(0) {}

CollisionDetection::~CollisionDetection() {
  if(h_traj_req_ != 0) {
    delete h_traj_req_;
    h_traj_req_ = 0;
  }
}

void CollisionDetection::init(ros::NodeHandle& h) {
  h_traj_req_ = new TrajectoryRequestHandler((const ros::NodeHandle&)h);
  //pub_pop = h.advertise<ramp_msgs::Population>("population", 1000);
  setT_od_w(id);
}




/** Returns true if trajectory_ is in collision with any of the objects */
const CollisionDetection::QueryResult CollisionDetection::perform() const {
  CollisionDetection::QueryResult result;

  // Duration for predicting the object trajectories
  ros::Duration d(5);
  
  // Predict the obstacle's trajectory
  ramp_msgs::Trajectory ob_trajectory = getPredictedTrajectory(obstacle_, d); 
  
  // Query for collision
  CollisionDetection::QueryResult q = query(ob_trajectory);
  
  // If collision, set result to q
  if(q.collision_) {
    result = q;
  }

  return result;  
} //End perform


/** 
 * Assume a 45 degree angle is formed between the robot's center and the reference point (left wheel)
 * */
const std::vector<float> CollisionDetection::getCenter(std::vector<float> p, float orientation) const {
  std::vector<float> result;
 
  // Get world coordinates of reference point 
  float x = p.at(0);
  float y = p.at(1);
  
  // Radius
  float r = 0.2155261;

  // Get world coodinates of center point
  x += r*cos( u.displaceAngle((-3*PI/4), orientation));
  y += r*sin( u.displaceAngle((-3*PI/4), orientation));
  
  result.push_back(x);
  result.push_back(y);

  return result;
} //End getCenter



/** Transformation matrix of obstacle robot */
void CollisionDetection::setT_od_w(int id) {

  if(id == 1) {
    tf::Vector3 pos(3.5f, 2.f, 0);
    T_od_w_.setOrigin(pos);
    T_od_w_.setRotation(tf::createQuaternionFromYaw(PI));
  }

  else {
    tf::Vector3 pos(0.f, 2.f, 0.f);
    T_od_w_.setRotation(tf::createQuaternionFromYaw(0));
    T_od_w_.setOrigin(pos);
  }
}


/** 
 * This method returns true if there is collision between trajectory_ and the ob_trajectory, false otherwise 
 * The robot and obstacles can be treated as circles for simple collision detection
 */
const CollisionDetection::QueryResult CollisionDetection::query(const ramp_msgs::Trajectory ob_trajectory) const {

  /*if(id == 1) {
    // Create population one member - ob_trajectory
    ramp_msgs::Population p; 
    ramp_msgs::Trajectory temp = transformT(ob_trajectory);
    p.population.push_back(temp);
    p.best_id = 0;
    p.robot_id = 2;
    pub_pop.publish(p);
  }*/


  //std::cout<<"\nQuery on "<<u.toString(trajectory_)<<" \n*******and*******\n"<<u.toString(ob_trajectory);
  CollisionDetection::QueryResult result;
  //std::cout<<"\nresult.collision_: "<<result.collision_;
  
  //std::cout<<"\nobstacle trajectory: "<<u.toString(ob_trajectory);
  // For every 3 points, check circle detection
  float radius = 0.55f;
  for(unsigned int i=0;i<trajectory_.trajectory.points.size() && i<ob_trajectory.trajectory.points.size();i+=3) {
    
    // Get the point on the trajectory, p
    trajectory_msgs::JointTrajectoryPoint p = trajectory_.trajectory.points.at(i);
    
    // Get the position vector for p
    std::vector<float> p_loc;
    p_loc.push_back(p.positions.at(0));
    p_loc.push_back(p.positions.at(1));

    // Get the center of the robot
    std::vector<float> p_center = getCenter(p_loc, p.positions.at(2));
    
    //std::cout<<"\nob_trajectory.size(): "<<ob_trajectory.trajectory.points.size();
    //std::cout<<"\n("<<ob_trajectory.trajectory.points.at(0).positions.at(0)<<", "<<ob_trajectory.trajectory.points.at(0).positions.at(1)<<")";

    // ***Test collision against the obstacle's trajectory***
    for(unsigned int j=i-1;j<i+1 && j<ob_trajectory.trajectory.points.size();j++) {

      // Get the points as the centers of the circles
      trajectory_msgs::JointTrajectoryPoint p_ob  = ob_trajectory.trajectory.points.at(j);
      //trajectory_msgs::JointTrajectoryPoint p_ob  = ob_trajectory.trajectory.points.at(i);

      // Get the position vector for the obstacle p_ob
      // Transform obstacle odometry coordinates to world CS
      // Find the centers of the objects
      //std::vector<float> p_ob_center  = getCenter(p_ob_loc, p.positions.at(2));
      std::vector<float> p_ob_center;
      
      // Find the center in odometry CS - center is -6, -6 from reference point
      float temp_x = p_ob.positions.at(0) - 0.1524f;
      float temp_y = p_ob.positions.at(1) - 0.1524f;

      // Transform the center
      tf::Vector3 temp_pos(temp_x, temp_y, 0);
      tf::Vector3 pos = T_od_w_ * temp_pos;
 
      // Push on the obstacle's center
      p_ob_center.push_back(pos.getX());
      p_ob_center.push_back(pos.getY());
      
      
      // Get the distance between the centers
      float dist = sqrt( pow(p_center.at(0) - p_ob_center.at(0),2) + pow(p_center.at(1) - p_ob_center.at(1),2) );
        
      /*if(id == 1) 
        std::cout<<"\nRobot 1 as p_center: ";
      else
        std::cout<<"\nRobot 2 as p_center: ";
      std::cout<<"Distance between ("<<p_center.at(0)<<", "<<p_center.at(1)<<") and ("<<p_ob_center.at(0)<<", "<<p_ob_center.at(1)<<"): "<<dist;*/
      
      
      // If the distance between the two centers is less than the sum of the two radii, 
      // there is collision
      if( dist <= radius*2 ) {
        result.collision_ = true;
        result.time_until_collision_ = p.time_from_start.toSec();
        j = i+1;
        i = trajectory_.trajectory.points.size();
      }
    } //end for
  } //end for

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
  if(mag_linear_t >= 0.15 && mag_angular_t < 0.25) {
    result = MotionType::Translation;
  }

  // Self-Rotation
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.25) {
    result = MotionType::SelfRotation;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.15 && mag_angular_t >= 0.25) {

    // Find v(t-1)
    tf::Vector3 v_linear_prev;
    tf::vector3MsgToTF(ob.odom_t_prev.twist.twist.linear, v_linear_prev);
    
    // Find angle between v(t) and v(t-1)
    double theta = tf::tfAngle(v_linear, v_linear_prev);

    // Check if v(t) and v(t-1) have similar directions
    if(theta > 0.01) {
      result = MotionType::TranslationAndSelfRotation;
    }
    else {
      result = MotionType::GlobalRotation;
    }
  } // end if t+sr or gr

  // Else, there is no motion
  else {
    result = MotionType::None;
  }

  return result;
} // End findMotionType






/** This method returns the predicted trajectory for an obstacle for the future duration d 
 * TODO: Remove Duration parameter and make the predicted trajectory be computed until robot reaches bounds of environment */
const ramp_msgs::Trajectory CollisionDetection::getPredictedTrajectory(const ramp_msgs::Obstacle ob, const ros::Duration d) const {
  ramp_msgs::Trajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(ob);
  
  
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
   
    // WE need reflexxes to have the current velocity of the robot and we use v_start for that
    // v_start is a velocity vector of size 3 
    tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
    tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.y);
    tr.request.v_start.push_back(ob.odom_t.twist.twist.angular.z);

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

    for(unsigned int i=0;i<p.points.size()-1;i++) {
      tr.request.v_start.push_back(PI/4);
      tr.request.v_end.push_back(PI/4);
    }

    tr.request.resolutionRate = 5;

    // Get trajectory
    if(h_traj_req_->request(tr)) {
      result = tr.response.trajectory;
    }
  } // end if self-rotation, none


  return result;
}
