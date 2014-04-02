#include "utility.h"


Utility::Utility() {
  float loc_min = 0;
  float loc_max = 3.5;

  ramp_msgs::Range r1;
  r1.min = loc_min;
  r1.max = loc_max;
  ramp_msgs::Range r2;
  r2.min = loc_min;
  r2.max = loc_max;
  ramp_msgs::Range r3;
  r3.min = -PI;
  r3.max = PI;

  ranges_.push_back(r1); 
  ranges_.push_back(r2); 
  ranges_.push_back(r3); 
}


Utility::~Utility() {}

/** This method returns the Euclidean distance between two position vectors */
const float Utility::euclideanDistance(const std::vector<float> a, const std::vector<float> b) const {

  float d_x = b.at(0) - a.at(0);
  float d_y = b.at(1) - a.at(1);
  return sqrt( pow(d_x,2) + pow(d_y,2) );
} //End euclideanDistance


const float Utility::findAngleFromAToB(const tf::Vector3 a, const tf::Vector3 b) const {
  std::vector<float> a_vec;
  a_vec.push_back(a.getX());
  a_vec.push_back(a.getY());
  
  std::vector<float> b_vec;
  b_vec.push_back(b.getX());
  b_vec.push_back(b.getY());

  return findAngleFromAToB(a_vec, b_vec);
}


/** This method returns the angle that will form a straight line from position a to position b. a and b are [x, y] vectors. */
const float Utility::findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const {
  float result;

  // Find the distances in x,y directions and Euclidean distance
  float d_x = b.at(0) - a.at(0);
  float d_y = b.at(1) - a.at(1);
  float euc_dist = sqrt( pow(d_x,2) + pow(d_y,2) );
  
  // If the positions are the same, set the result to starting angle
  if(euc_dist == 0) {
    result = 0;
  }

  // If b is in the 1st or 2nd quadrants
  else if(d_y > 0) {
    result = acos(d_x / euc_dist);
  }

  // If b is in the 3rd quadrant, d_y<0 & d_x<0
  else if(d_x < 0) {
    result = -PI - asin(d_y / euc_dist);
  }

  // If b is in the 4th quadrant, d_y<=0 & d_x>=0
  else {
    result = asin(d_y / euc_dist); 
  }

  return result;
} //End findAngleFromAToB


/** This method returns distance between orientations a1 and a2. The distance is in the range [-PI, PI]. */
const float Utility::findDistanceBetweenAngles(const float a1, const float a2) const {
  float result;

  float difference = a1 - a2;
  
  // If difference > pi, the result should be in [-PI,0] range
  if(difference > PI) {
    difference = fmodf(difference, PI);
    result = (difference == 0) ? 0 : difference - PI;
  }

  // If difference < -pi, the result should be in [0,PI] range
  else if(difference < -PI) {
    result = difference + (2*PI);
  }

  // Else, the difference is fine
  else {
    result = difference;
  }

  return result;
} //End findDistanceBetweenAngles 


const float Utility::displaceAngle(const float a1, float a2) const {

  a2 = fmodf(a2, 2*PI);

  if(a2 > PI) {
    a2 = fmodf(a2,PI) - PI;
  }

  return findDistanceBetweenAngles(a1, -a2);
} //End displaceAngle



const ramp_msgs::Configuration Utility::getConfigurationFromPoint(const trajectory_msgs::JointTrajectoryPoint p) const {
  ramp_msgs::Configuration result; 

  result.ranges = ranges_;

  for(unsigned int i=0;i<3;i++) {
    result.K.push_back(p.positions.at(i));
  }

  return result;
}


const ramp_msgs::Path Utility::getPath(const std::vector<ramp_msgs::Configuration> configs) const {
  ramp_msgs::Path result;

  for(unsigned int i=0;i<configs.size();i++) {
    ramp_msgs::KnotPoint kp;
    kp.configuration = configs.at(i);
    kp.stop_time = 0;
    result.points.push_back(kp);
  }

  return result;
}

const ramp_msgs::Path Utility::getPath(const std::vector<ramp_msgs::KnotPoint> kps) const {
  ramp_msgs::Path result;

  for(unsigned int i=0;i<kps.size();i++) {
    result.points.push_back(kps.at(i));
  }

  return result;
}


const std::string Utility::toString(const ramp_msgs::Trajectory traj) const {
  std::ostringstream result;

  result<<"\n Knot Points:";

  for(unsigned int i=0;i<traj.index_knot_points.size();i++) {
    
    result<<"\n   "<<i<<":";
    unsigned int index = traj.index_knot_points.at(i);

    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(index);
    


    result<<"\n       Positions: ("<<p.positions.at(0);
    for(unsigned int k=1;k<p.positions.size();k++) {
      result<<", "<<p.positions.at(k);
    }
    result<<")";

  }


  result<<"\n Points:";
  for(unsigned int i=0;i<traj.trajectory.points.size();i++) {
    result<<"\n\n   Point "<<i<<":";
    
    trajectory_msgs::JointTrajectoryPoint p = traj.trajectory.points.at(i);
  
    //Positions
    result<<"\n       Positions: ("<<p.positions.at(0);
    for(unsigned int k=1;k<p.positions.size();k++) {
      result<<", "<<p.positions.at(k);
    }
    result<<")";
  
    //Velocities
    result<<"\n       Velocities: ("<<p.velocities.at(0);
    for(unsigned int k=1;k<p.velocities.size();k++) {
      result<<", "<<p.velocities.at(k);
    }
    result<<")";
    
    //Accelerations
    result<<"\n       Accelerations: ("<<p.accelerations.at(0);
    for(unsigned int k=1;k<p.accelerations.size();k++) {
      result<<", "<<p.accelerations.at(k);
    }
    result<<")";
    
    result<<"\n Time From Start: "<<p.time_from_start;

  }

  result<<"\n Feasible: "<<traj.feasible;
  result<<"\n Fitness:  "<<traj.fitness;

  return result.str();
}



const std::string Utility::toString(const ramp_msgs::KnotPoint kp) const {
  std::ostringstream result;

  result<<"\nConfiguration: "<<toString(kp.configuration);
  result<<", Stop time: "<<kp.stop_time;

  return result.str();
}

const std::string Utility::toString(const ramp_msgs::Configuration c) const {
  std::ostringstream result;
  result<<"(";
  for(unsigned int i=0;i<c.K.size()-1;i++) {
    result<<c.K.at(i)<<", ";
  }
  result<<c.K.at(c.K.size()-1)<<")";
  return result.str();
}


const std::string Utility::toString(const ramp_msgs::Path path) const {
  std::ostringstream result;

  result<<"\nPath: ";
  for(unsigned int i=0;i<path.points.size();i++) {
    result<<"\n "<<i<<": "<<toString(path.points.at(i));
  }

  return result.str();
}
