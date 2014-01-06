#include "collision_detection.h"

/** Returns true if trajectory_ is in collision with any of the objects */
const bool CollisionDetection::perform() const {
  
  ros::Duration d(3);
  // For each obstacle
  // Compute its trajectory to query collision detection
  for(unsigned int i=0;i<obstacleList_.obstacles.size();i++) {
    ramp_msgs::Trajectory ob_trajectory = getPredictedTrajectory(obstacleList_.obstacles.at(i), d); 
    //std::cout<<"\ntrajectory_: "<<u.toString(trajectory_);
    //std::cout<<"\nob_trajectory: "<<u.toString(ob_trajectory);
    if(query(ob_trajectory)) {
      return true;
    }
  }

  return false;  
}


/** This method returns true if k lies on the segment, ij */
const bool CollisionDetection::onSegment(const tf::Point p_i, const tf::Point p_j, const tf::Point p_k) const {
  
  float min_x_ij = (p_i.m_floats[0] < p_j.m_floats[0]) ? p_i.m_floats[0] : p_j.m_floats[0]; 
  float max_x_ij = (p_i.m_floats[0] > p_j.m_floats[0]) ? p_i.m_floats[0] : p_j.m_floats[0]; 

  float min_y_ij = (p_i.m_floats[1] < p_j.m_floats[1]) ? p_i.m_floats[1] : p_j.m_floats[1]; 
  float max_y_ij = (p_i.m_floats[1] > p_j.m_floats[1]) ? p_i.m_floats[1] : p_j.m_floats[1]; 
 
  float x_k = p_k.m_floats[0];
  float y_k = p_k.m_floats[1];

  if( (min_x_ij <= x_k && x_k <= max_x_ij) && (min_y_ij <= y_k && y_k <= max_y_ij) ) {
    return true;
  }
  
  return false;
}



/** 
 * This method returns true if there is collision between trajectory_ and the ob_trajectory, false otherwise 
 * The robot and obstacles can be treated as circles for simple collision detection
 */
const bool CollisionDetection::query(const ramp_msgs::Trajectory ob_trajectory) const {
  
  // For every 2 points, check circle detection
  float radius = 0.33;
  for(unsigned int i=0;i<trajectory_.trajectory.points.size() && i<ob_trajectory.trajectory.points.size();i+=4) {

    // Get the points as the centers of the circles
    trajectory_msgs::JointTrajectoryPoint r_center  = trajectory_.trajectory.points.at(i);
    trajectory_msgs::JointTrajectoryPoint ob_center = ob_trajectory.trajectory.points.at(i);

    // Get the distance between the centers
    float dist = sqrt( pow(r_center.positions.at(0) - ob_center.positions.at(0),2) + pow(r_center.positions.at(1) - ob_center.positions.at(1),2) );

    // If the distance between the two centers is greater than the sum of the radiuses
    if( dist <= radius*2 ) {
      return true;
    }
  } //end for

  return false;
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
  if(mag_linear_t >= 0.1 && mag_angular_t < 0.1) {
    result = MotionType::Translation;
  }

  // Self-Rotation
  else if(mag_linear_t < 0.1 && mag_angular_t >= 0.1) {
    result = MotionType::SelfRotation;
  }

  // Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.1 && mag_angular_t >= 0.1) {

    // Find v(t-1)
    tf::Vector3 v_linear_prev;
    tf::vector3MsgToTF(ob.odom_t_prev.twist.twist.linear, v_linear_prev);
    
    // Find angle between v(t) and v(t-1)
    double theta = tf::tfAngle(v_linear, v_linear_prev);

    // Check if v(t) and v(t-1) have similar directions
    if(theta > 0.1) {
      result = MotionType::TranslationAndSelfRotation;
    }
    else {
      result = MotionType::GlobalRotation;
    }
  }

  // Else, there is no motion
  else {
    result = MotionType::None;
  }


  return result;
} // End findMotionType


/** This method returns the predicted trajectory for an obstacle for the future duration d */
const ramp_msgs::Trajectory CollisionDetection::getPredictedTrajectory(const ramp_msgs::Obstacle ob, const ros::Duration d) const {
  ramp_msgs::Trajectory result;

  // First, identify which type of trajectory it is
  // translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion = findMotionType(ob);
  
  // The starting point 
  trajectory_msgs::JointTrajectoryPoint start;

  if(motion == MotionType::Translation) {

    // Positions
    start.positions.push_back(ob.odom_t.pose.pose.position.x);
    start.positions.push_back(ob.odom_t.pose.pose.position.y);
    start.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

    // Velocities
    start.velocities.push_back(ob.odom_t.twist.twist.linear.x);
    start.velocities.push_back(ob.odom_t.twist.twist.linear.y);
    start.velocities.push_back(0);

    // Accelerations
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);


    // Goal point
    trajectory_msgs::JointTrajectoryPoint end;
    for(unsigned int i=0;i<3;i++) {
      end.positions.push_back(start.positions.at(i) + (start.velocities.at(i) * d.toSec()));
      end.velocities.push_back(start.velocities.at(i));
      end.accelerations.push_back(start.accelerations.at(i));
    }
    end.time_from_start = d;

    // Push knot points onto result
    result.index_knot_points.push_back(0);
    result.index_knot_points.push_back(1);

    result.trajectory.points.push_back(start);
    result.trajectory.points.push_back(end);

  } // end if translation


  else if(motion == MotionType::SelfRotation) {

    // Positions
    start.positions.push_back(ob.odom_t.pose.pose.position.x);
    start.positions.push_back(ob.odom_t.pose.pose.position.y);
    start.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

    // Velocities
    start.velocities.push_back(0);
    start.velocities.push_back(0);
    start.velocities.push_back(ob.odom_t.twist.twist.angular.z);

    // Accelerations
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);
    
    // Goal point
    trajectory_msgs::JointTrajectoryPoint end;
    for(unsigned int i=0;i<3;i++) {
      end.positions.push_back(start.positions.at(i) + (start.velocities.at(i) * d.toSec()));
      end.velocities.push_back(start.velocities.at(i));
      end.accelerations.push_back(start.accelerations.at(i));
    }
    end.time_from_start = d;

    result.index_knot_points.push_back(0);
    result.index_knot_points.push_back(1);

    result.trajectory.points.push_back(start);
    result.trajectory.points.push_back(end);
  }


  return result;
}
