#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory(const float resRate, unsigned int id) {
  msg_.id = id;
  msg_.feasible = true;
  msg_.fitness = -1;  
  msg_.t_firstCollision = 9999.f;
  msg_.resolutionRate = resRate;
}

RampTrajectory::RampTrajectory(const ramp_msgs::RampTrajectory msg) : msg_(msg) {}


const bool RampTrajectory::equals(const RampTrajectory& other) const {
  if(msg_.id == other.msg_.id) {
    return true;
  }

  return path_.equals(other.path_);
}


const Path RampTrajectory::getPath() const {
  Path result;

  for(unsigned int i=0;i<msg_.i_knotPoints.size();i++) {

    MotionState ms(msg_.trajectory.points.at( msg_.i_knotPoints.at(i)));
  
    result.all_.push_back(ms);
  }

  result.start_ = result.all_.at(0);
  result.goal_  = result.all_.at( result.all_.size()-1 );
  
  return result;
}


/** Time is in seconds */
const trajectory_msgs::JointTrajectoryPoint RampTrajectory::getPointAtTime(const float t) const {
  //ROS_INFO("In RampTrajectory::getPointAtTime");
  
  double resolutionRate = msg_.trajectory.points.at(1).time_from_start.toSec() -
                          msg_.trajectory.points.at(0).time_from_start.toSec();
  int i = ceil((t/resolutionRate));
  //ROS_INFO("t: %f resolutionRate: %f i: %i", t, resolutionRate, i);

  if( i > msg_.trajectory.points.size() ) {
    return msg_.trajectory.points.at( msg_.trajectory.points.size()-1 );
  }

  return msg_.trajectory.points.at(i);
}





/** Returns the direction of the trajectory, i.e. the
* orientation the base needs to move on the trajectory */
const double RampTrajectory::getDirection() const {
  //std::cout<<"\nIn getDirection\n";
  std::vector<double> a = path_.start_.motionState_.msg_.positions;

  std::vector<double> b = path_.all_.at(1).motionState_.msg_.positions;

    //msg_.trajectory.points.at(msg_.i_knotPoints.at(2)) :
    //msg_.trajectory.points.at(msg_.i_knotPoints.at(1)) ;
  //std::cout<<"\nLeaving getDirection\n";
  return utility_.findAngleFromAToB(a, b);
}




// Inclusive
const RampTrajectory RampTrajectory::getSubTrajectory(const float t) const {
  //ROS_INFO("t: %f", t);
  ramp_msgs::RampTrajectory rt;

  uint8_t i_kp = 0;
  for(float i=0.f;i<=t+0.000001;i+=0.1f) { // TODO: i+=cycle_time
    rt.trajectory.points.push_back(msg_.trajectory.points.at(i*10.));  // todo: i*1/cycle_time
    if(msg_.i_knotPoints.at(i_kp) == (i*10)) {
      rt.i_knotPoints.push_back(i*10);
      i_kp++;
    }
  }

  RampTrajectory result(rt);

  return result;
}


const RampTrajectory RampTrajectory::clone() const { 
  return *this;
}


const std::string RampTrajectory::fitnessFeasibleToString() const {
  std::ostringstream result;
 
  result<<"\nTrajectory ID: "<<msg_.id;
  result<<"\n Number of knot points: "<<msg_.i_knotPoints.size(); 
  result<<"\n Path: "<<path_.toString();
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<msg_.id<<"\nPath: "<<utility_.toString(msg_);
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;
  
  return result.str();
}




