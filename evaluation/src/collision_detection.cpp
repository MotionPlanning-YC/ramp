#include "collision_detection.h"

/** Returns true if trajectory_ is in collision with any of the objects */
const bool CollisionDetection::perform() const {
  
  ros::Duration d(3);
  //For each obstacle
  //Compute its trajectory to query collision detection
  for(unsigned int i=0;i<obstacleList_.obstacles.size();i++) {
    ramp_msgs::Trajectory ob_trajectory = getTrajectoryRequest(obstacleList_.obstacles.at(i), d); 
    std::cout<<"\ntrajectory_: "<<u.toString(trajectory_);
    std::cout<<"\nob_trajectory: "<<u.toString(ob_trajectory);
    if(query(ob_trajectory)) {
      return true;
    }
  }

  return false;  
}

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

const bool CollisionDetection::query(const ramp_msgs::Trajectory ob_trajectory) const {
  //The trajectories are linear segments so a simple geometric method will suffice

  //Find the endpoints of the segments
  tf::Point p1, p2, p3, p4;
  
  //Robot trajectory segments
  p1.m_floats[0] = trajectory_.trajectory.points.at(0).positions.at(0);
  p1.m_floats[1] = trajectory_.trajectory.points.at(0).positions.at(1);
  p1.m_floats[2] = 0;
  p2.m_floats[0] = trajectory_.trajectory.points.at(1).positions.at(0);
  p2.m_floats[1] = trajectory_.trajectory.points.at(1).positions.at(1);
  p2.m_floats[2] = 0;

  for(unsigned int i=0;i<3;i++) {
    std::cout<<"\np1.m_floats["<<i<<"]: "<<p1.m_floats[i];
  }
  for(unsigned int i=0;i<3;i++) {
    std::cout<<"\np2.m_floats["<<i<<"]: "<<p2.m_floats[i];
  }

  //Obstacle trajectory segments
  p3.m_floats[0] = ob_trajectory.trajectory.points.at(0).positions.at(0);
  p3.m_floats[1] = ob_trajectory.trajectory.points.at(0).positions.at(1);
  p3.m_floats[2] = 0;
  p4.m_floats[0] = ob_trajectory.trajectory.points.at(1).positions.at(0);
  p4.m_floats[1] = ob_trajectory.trajectory.points.at(1).positions.at(1);
  p4.m_floats[2] = 0;

  for(unsigned int i=0;i<3;i++) {
    std::cout<<"\np3.m_floats["<<i<<"]: "<<p3.m_floats[i];
  }
  for(unsigned int i=0;i<3;i++) {
    std::cout<<"\np4.m_floats["<<i<<"]: "<<p4.m_floats[i];
  }

  //Find the angle between various line segments
  /*float d1 = tf::tfAngle(p1 - p3, p4 - p3) * (180.0f/M_PI);  
  float d2 = tf::tfAngle(p2 - p3, p4 - p3) * (180.0f/M_PI);  
  float d3 = tf::tfAngle(p3 - p1, p2 - p1) * (180.0f/M_PI);  
  float d4 = tf::tfAngle(p4 - p1, p2 - p1) * (180.0f/M_PI);  
  std::cout<<"\nd1: "<<d1<<" d2: "<<d2<<" d3: "<<d3<<" d4: "<<d4;*/

  tf::Vector3 v1_3 = p1 - p3;
  tf::Vector3 v4_3 = p4 - p3;
  tf::Vector3 v2_3 = p2 - p3;
  tf::Vector3 v3_1 = p3 - p1;
  tf::Vector3 v2_1 = p2 - p1;
  tf::Vector3 v4_1 = p4 - p1;
  std::cout<<"\nv1_3: <"<<v1_3.m_floats[0]<<", "<<v1_3.m_floats[1]<<">";
  std::cout<<"\nv4_3: <"<<v4_3.m_floats[0]<<", "<<v4_3.m_floats[1]<<">";

  tf::Vector3 c1 = tf::tfCross(v1_3, v4_3);  
  tf::Vector3 c2 = tf::tfCross(v2_3, v4_3);  
  tf::Vector3 c3 = tf::tfCross(v3_1, v2_1);  
  tf::Vector3 c4 = tf::tfCross(v4_1, v2_1);  
  std::cout<<"\nc1: <"<<c1.m_floats[0]<<", "<<c1.m_floats[1]<<", "<<c1.m_floats[2]<<">";

  float d1 = (c1.m_floats[0] - c1.m_floats[1] + c1.m_floats[2]);
  float d2 = (c2.m_floats[0] - c2.m_floats[1] + c2.m_floats[2]);
  float d3 = (c3.m_floats[0] - c3.m_floats[1] + c3.m_floats[2]);
  float d4 = (c4.m_floats[0] - c4.m_floats[1] + c4.m_floats[2]);
  std::cout<<"\nd1: "<<d1<<" d2: "<<d2<<" d3: "<<d3<<" d4: "<<d4;


  if( ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
      ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0)) )
  {
    return true;
  }

  else if(d1 == 0 && onSegment(p3, p4, p1)) {
    return true;
  }
  
  else if(d1 == 0 && onSegment(p3, p4, p2)) {
    return true;
  }

  else if(d1 == 0 && onSegment(p1, p2, p3)) {
    return true;
  }

  else if(d1 == 0 && onSegment(p1, p2, p4)) {
    return true;
  }

  return false;
}


/** This method determines what type of motion an obstacle has */
const MotionType CollisionDetection::findMotionType(const ramp_msgs::Obstacle ob) const {
  MotionType result;

  //Find the linear and angular velocities
  tf::Vector3 v_linear;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.linear, v_linear);

  tf::Vector3 v_angular;
  tf::vector3MsgToTF(ob.odom_t.twist.twist.angular, v_angular);

  //Find magnitude of velocity vectors
  float mag_linear_t  = sqrt( tf::tfDot(v_linear, v_linear)   );
  float mag_angular_t = sqrt( tf::tfDot(v_angular, v_angular) );


  //Translation only
  if(mag_linear_t >= 0.1 && mag_angular_t < 0.1) {
    result = MotionType::Translation;
  }

  //Self-Rotation
  else if(mag_linear_t < 0.1 && mag_angular_t >= 0.1) {
    result = MotionType::SelfRotation;
  }

  //Either translation+self-rotation or global rotation
  else if(mag_linear_t >= 0.1 && mag_angular_t >= 0.1) {

    //Find v(t-1)
    tf::Vector3 v_linear_prev;
    tf::vector3MsgToTF(ob.odom_t_prev.twist.twist.linear, v_linear_prev);
    
    //Find angle between v(t) and v(t-1)
    double theta = tf::tfAngle(v_linear, v_linear_prev);

    //Check if v(t) and v(t-1) have similar directions
    if(theta > 0.1) {
      result = MotionType::TranslationAndSelfRotation;
    }
    else {
      result = MotionType::GlobalRotation;
    }
  }

  //Else, there is no motion
  else {
    result = MotionType::None;
  }


  return result;
} //End findMotionType


const ramp_msgs::Trajectory CollisionDetection::getTrajectoryRequest(const ramp_msgs::Obstacle ob, const ros::Duration d) const {
  ramp_msgs::Trajectory result;

  //First, identify which type of trajectory it is
  //translations only, self-rotation, translation and self-rotation, or global rotation
  MotionType motion = findMotionType(ob);

  if(motion == MotionType::Translation) {
    std::cout<<"\nIn translation\n";

    trajectory_msgs::JointTrajectoryPoint start;
    
    //Positions
    start.positions.push_back(ob.odom_t.pose.pose.position.x);
    start.positions.push_back(ob.odom_t.pose.pose.position.y);
    start.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

    //Velocities
    start.velocities.push_back(ob.odom_t.twist.twist.linear.x);
    start.velocities.push_back(ob.odom_t.twist.twist.linear.y);
    start.velocities.push_back(0);

    //Accelerations
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);


    //Goal point
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

  } //end if translation


  else if(motion == MotionType::SelfRotation) {
     
    trajectory_msgs::JointTrajectoryPoint start;

    //Positions
    start.positions.push_back(ob.odom_t.pose.pose.position.x);
    start.positions.push_back(ob.odom_t.pose.pose.position.y);
    start.positions.push_back(tf::getYaw(ob.odom_t.pose.pose.orientation));

    //Velocities
    start.velocities.push_back(0);
    start.velocities.push_back(0);
    start.velocities.push_back(ob.odom_t.twist.twist.angular.z);

    //Accelerations
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);
    start.accelerations.push_back(0);
    
    //Goal point
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


  //Now send the trajectory


  //return traj_req;
  return result;
}
