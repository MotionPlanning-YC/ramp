#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory(const float resRate, unsigned int id) : timeUntilCollision_(9999.f)  {
  msg_.id = id;
  msg_.feasible = true;
  msg_.fitness = -1;  
  msg_.resolutionRate = resRate;
}

RampTrajectory::RampTrajectory(const ramp_msgs::RampTrajectory msg) : msg_(msg), timeUntilCollision_(9999.0f) {}


const bool RampTrajectory::equal(const RampTrajectory& other) const {
  return msg_.id == other.msg_.id;
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
  double resolutionRate = msg_.trajectory.points.at(1).time_from_start.toSec() -
                          msg_.trajectory.points.at(0).time_from_start.toSec();
  if( (t/resolutionRate) > msg_.trajectory.points.size() ) {
    return msg_.trajectory.points.at( msg_.trajectory.points.size()-1 );
  }

  return msg_.trajectory.points.at( (t / resolutionRate) );
}





/** Returns the direction of the trajectory, i.e. the
* orientation the base needs to move on the trajectory */
const double RampTrajectory::getDirection() const {
  trajectory_msgs::JointTrajectoryPoint a = msg_.trajectory.points.at(0);
  trajectory_msgs::JointTrajectoryPoint b = msg_.trajectory.points.at(msg_.i_knotPoints.at(1));

  return utility_.findAngleFromAToB(a, b);
}



const RampTrajectory RampTrajectory::getStraightSegment(uint8_t i) const {

  int i_startSegment = msg_.i_knotPoints.at(i);

  for(uint16_t i=i_startSegment;i<msg_.i_knotPoints.at(i+1);i++) {
    if(msg_.trajectory.points.at(i).positions.at(2) > 0.0001) {
      RampTrajectory result = *this;
      
      // Get subset of vector
      std::vector<trajectory_msgs::JointTrajectoryPoint>::const_iterator first = 
        msg_.trajectory.points.begin()+i;
      std::vector<trajectory_msgs::JointTrajectoryPoint> m(first, msg_.trajectory.points.end());

      result.msg_.trajectory.points = m;

      return result;
    }
  } 

  return *this;
}


const RampTrajectory RampTrajectory::clone() const { 
  return *this;
}


const std::string RampTrajectory::fitnessFeasibleToString() const {
  std::ostringstream result;
 
  result<<"\nTrajectory ID: "<<msg_.id;
  result<<"\n Number of knot points: "<<msg_.i_knotPoints.size(); 
  result<<"\n Path: "<<path_.toString();
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<msg_.feasible<<" Collision Time: "<<timeUntilCollision_;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<msg_.id<<"\n"<<utility_.toString(msg_);
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<msg_.feasible<<" Collision Time: "<<timeUntilCollision_;
  
  return result.str();
}


