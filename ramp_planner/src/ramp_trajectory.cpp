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
  std::cout<<"\nIn getDirection\n";
  std::vector<double> a = path_.start_.motionState_.msg_.positions;

  std::vector<double> b = path_.all_.at(1).motionState_.msg_.positions;

    //msg_.trajectory.points.at(msg_.i_knotPoints.at(2)) :
    //msg_.trajectory.points.at(msg_.i_knotPoints.at(1)) ;
  std::cout<<"\nLeaving getDirection\n";
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
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<msg_.id<<"\n"<<utility_.toString(msg_);
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;
  
  return result.str();
}


/** Returns the first index with non-zero angular velocity, -1 if no angular v */
const int RampTrajectory::getIndexFirstTurn() const {

  for(uint16_t i=0;i<msg_.trajectory.points.size()-1;i++) {
    if( fabs(msg_.trajectory.points.at(i).velocities.at(2)) > 0.01 &&
        fabs(msg_.trajectory.points.at(i+1).velocities.at(2))) {
      return i;
    }
  }
  return -1;
}


const int RampTrajectory::getIndexStartOfCurve() const {
  for(uint16_t i=6;i>0;i--) {
    if(fabs( msg_.trajectory.points.at(i).positions.at(2) ) < 0.0001) {
      return i;
    }
  }

  return 6;
}

/** Returns the first index with non-zero angular velocity, -1 if no angular v */
const int RampTrajectory::getIndexFirstTurn(const uint16_t start) const {

  for(uint16_t i=start;i<msg_.trajectory.points.size();i++) {
    if( fabs(msg_.trajectory.points.at(i).velocities.at(2)) > 0.01) {
      return i;
    }
  }

  return -1;
}


/** Returns the first index with non-zero angular velocity, -1 if no angular v */
const double RampTrajectory::getTimeFirstTurn() const {

  int i = getIndexFirstTurn();
  
  if(i > -1) {
    return msg_.trajectory.points.at(i).time_from_start.toSec();
  }
  
  return -1;
}



/** Returns the first index with non-zero angular velocity, -1 if no angular v */
const double RampTrajectory::getTimeFirstTurn(const uint16_t start) const {

  int i = getIndexFirstTurn(start);
  
  if(i > -1) {
    return msg_.trajectory.points.at(i).time_from_start.toSec();
  }
  
  return -1;
}

