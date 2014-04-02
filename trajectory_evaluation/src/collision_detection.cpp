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

const std::vector<float> CollisionDetection::getCenter(std::vector<float> p, float orientation) const {
std::vector<float> result;

  // Get world coordinates of reference point
  float x = p.at(0);
  float y = p.at(1);

  // Radius
  float r = 0.2155261;
      
  float temp_x = x - 0.1524f;
  float temp_y = y - 0.1524f;

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
    T_w_r_.setOrigin(pos);
    T_w_r_.setRotation(tf::createQuaternionFromYaw(PI));
  }

  else {
    tf::Vector3 pos(0.f, 2.f, 0.f);
    T_w_r_.setRotation(tf::createQuaternionFromYaw(0));
    T_w_r_.setOrigin(pos);
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
      tf::Vector3 pos = T_w_r_ * temp_pos;
 
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
  // normally 0.0066 when idle
  if(mag_linear_t >= 0.15 && mag_angular_t < 0.1) {
    result = MotionType::Translation;
  }

  // Self-Rotation
  // normally 0.053 when idle
  else if(mag_linear_t < 0.15 && mag_angular_t >= 0.25) {
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
const ramp_msgs::Trajectory CollisionDetection::getPredictedTrajectory(const ramp_msgs::Obstacle ob, const ros::Duration d) const {
  ramp_msgs::Trajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion_type = findMotionType(ob);

  
  // The starting point 
  //trajectory_msgs::JointTrajectoryPoint start;
  ramp_msgs::KnotPoint start;
  start.configuration.ranges = u.ranges_;



  // Positions
  start.configuration.K.push_back(ob.odom_t.pose.pose.position.x);
  start.configuration.K.push_back(ob.odom_t.pose.pose.position.y);
  start.configuration.K.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));


  ramp_msgs::TrajectoryRequest tr;
  std::vector<ramp_msgs::KnotPoint> cs;
  
  // start will be the starting configuration in the path
  cs.push_back(start);

  /** Find the ending configuration for the predicted trajectory based on motion type */
  // If translation
  if(motion_type == MotionType::Translation) {

    // Get the Goal configuration
    ramp_msgs::KnotPoint end;
    end.configuration.ranges = u.ranges_;
    
    end.configuration.K.push_back(start.configuration.K.at(0) + (ob.odom_t.twist.twist.linear.x * d.toSec()));
    end.configuration.K.push_back(start.configuration.K.at(1) + (ob.odom_t.twist.twist.linear.y * d.toSec()));
    end.configuration.K.push_back(start.configuration.K.at(2));
    

    // Now we have starting and ending configurations
    // Build a Path
    cs.push_back(end);
  } // end if translation




  // If translation and rotation
  else if(motion_type == MotionType::TranslationAndRotation) {
    // Find the linear and angular velocities
    tf::Vector3 v_linear;
    tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);

    tf::Vector3 v_angular;
    tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

    // Find magnitude of velocity vectors
    float v = sqrt( tf::tfDot(v_linear, v_linear)   );
    float w = sqrt( tf::tfDot(v_angular, v_angular) );
    float r = v / w;
    std::cout<<"\nv: "<<v<<" w: "<<w<<" r: "<<r;

    std::vector<float> a;
    a.push_back(0);
    a.push_back(0);

    std::vector<float> b;
    b.push_back(ob.odom_t.pose.pose.position.x);
    b.push_back(ob.odom_t.pose.pose.position.y);
    
    // This is the theta from robot origin to robot position
    float polar_theta_r = u.findAngleFromAToB(a, b);
    // This is the radius from robot origin to robot position
    float polar_r_r = sqrt(pow(start.configuration.K.at(0),2) + pow(start.configuration.K.at(1), 2));
    std::cout<<"\npolar_theta_r: "<<polar_theta_r;
    std::cout<<"\npolar_r_r: "<<polar_r_r;


    //float polar_theta_w = u.findAngleFromAToB(a, b);
    //float r_w = sqrt(pow(start.configuration.K.at(0),2) + pow(start.configuration.K.at(1), 2));

    /*std::cout<<"\npolar_theta_w: "<<polar_theta_w;
    std::cout<<"\nr_w: "<<r_w;
    std::cout<<"\n"<<r_w<<"*cos("<<polar_theta_w<<"): "<<r_w*cos(polar_theta_w);
    std::cout<<"\n"<<r_w<<"*sin("<<polar_theta_w<<"): "<<r_w*sin(polar_theta_w);*/

    // Generate intermediate points
    for(float i=0.25f;i<d.toSec();i+=0.25f) {
      ramp_msgs::KnotPoint temp;
      temp.configuration.ranges = u.ranges_;

      // Get the new polar coodinates theta value in robot frame 
      float theta_prime_r = u.displaceAngle(polar_theta_r, w*i);
      //float theta_prime_w = u.displaceAngle(polar_theta_w, w*i);

      // Convert from polar to cartesian in robot frame
      float x_prime_r = polar_r_r * cos(theta_prime_r);
      float y_prime_r = polar_r_r * sin(theta_prime_r);
      float theta_r = u.displaceAngle(start.configuration.K.at(2), w*i);
      std::cout<<"\nx_prime_r: "<<x_prime_r<<" y_prime_r: "<<y_prime_r<<" theta_r: "<<theta_r;

      // Now convert to world coordinates
      tf::Vector3 p_r(x_prime_r, y_prime_r, 0);
      tf::Vector3 p_w = T_w_r_ * p_r;

      // Push the values onto temp
      temp.configuration.K.push_back(p_w.getX());
      temp.configuration.K.push_back(p_w.getY());
      temp.configuration.K.push_back(u.displaceAngle(theta_r, tf::getYaw(T_w_r_.getRotation())));
      
      //temp.configuration.K.push_back(x);
      //temp.configuration.K.push_back(y);
      //temp.configuration.K.push_back(theta);
      
      cs.push_back(temp);
    }
  }
  



  // If rotation
  // Since our robot models are circles, rotation is the same as no movement
  else if(motion_type == MotionType::Rotation || motion_type == MotionType::None) {
    cs.push_back(start);
  } // end if self-rotation, none


  std::cout<<"\nPath: ";
  for(int i=0;i<cs.size();i++) {
    std::cout<<"\n("<<cs.at(i).configuration.K.at(0)<<", "<<cs.at(i).configuration.K.at(1)<<", "<<cs.at(i).configuration.K.at(2);
  }


  tf::Vector3 start_w(start.configuration.K.at(0), start.configuration.K.at(1), 0);
  start_w = T_w_r_ * start_w;
  cs.at(0).configuration.K.at(0) = start_w.getX();
  cs.at(0).configuration.K.at(1) = start_w.getY();
  cs.at(0).configuration.K.at(2) = u.displaceAngle(start.configuration.K.at(2), tf::getYaw(T_w_r_.getRotation()));


  // Now build a Trajectory Request 
  ramp_msgs::Path p = u.getPath(cs);
  tr.request.path = p;
  tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
  tr.request.v_start.push_back(ob.odom_t.twist.twist.linear.x);
  tr.request.v_start.push_back(ob.odom_t.twist.twist.angular.z);
  tr.request.resolutionRate = 5;

  // Get trajectory
  if(h_traj_req_->request(tr)) {
    result = tr.response.trajectory;
  }

  return result;
}
