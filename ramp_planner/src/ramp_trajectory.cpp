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
  
  
  float resolutionRate = 0.1;
  int i = ceil((t/resolutionRate));
  /*ROS_INFO("t: %f resolutionRate: %f i: %i size: %i", 
      t, 
      resolutionRate, 
      i, 
      (int)msg_.trajectory.points.size());*/

  if( i >= msg_.trajectory.points.size() ) {
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
  ramp_msgs::RampTrajectory rt;

  double t_stop = t;

  if(msg_.trajectory.points.size() < 1)
  {
    ROS_ERROR("msg_.trajectory.points.size == 0");
    return clone();
  }
  else if(msg_.trajectory.points.size() == 1)
  {
    rt.trajectory.points.push_back(msg_.trajectory.points.at(0));
    rt.i_knotPoints.push_back(0);
  }

  else
  {
    if( t > msg_.trajectory.points.at( msg_.trajectory.points.size()-1 ).time_from_start.toSec()) 
    {
      t_stop = msg_.trajectory.points.at( msg_.trajectory.points.size()-1 ).time_from_start.toSec();
    }

    //ROS_INFO("t: %f t_stop: %f", t, t_stop);

    uint8_t i_kp = 0;
    for(float i=0.f;i<=t_stop+0.000001;i+=0.1f) { // TODO: i+=cycle_time
      uint16_t index = floor(i*10.) < msg_.trajectory.points.size() ? floor(i*10) : 
        msg_.trajectory.points.size()-1;

      //ROS_INFO("index: %i size: %i", index, (int)msg_.trajectory.points.size());
      rt.trajectory.points.push_back(msg_.trajectory.points.at(index));  // todo: i*1/cycle_time
      if(msg_.i_knotPoints.at(i_kp) == index) {
        rt.i_knotPoints.push_back(index);
        i_kp++;
      }
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
  result<<"\n Path: "<<bezierPath_.toString();
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<msg_.id<<"\nTrajec: "<<utility_.toString(msg_);
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;
  
  return result.str();
}




