#include "utility.h"
#include "range.h"

Utility::Utility() {
  Range range0(0, 3.5);
  Range range1(0, 3.5);
  Range range2(-PI, PI);
  standardRanges.push_back(range0);
  standardRanges.push_back(range1);
  standardRanges.push_back(range2);
}


/** This method returns the Euclidean distance between two position vectors */
const float Utility::positionDistance(const std::vector<float> a, const std::vector<float> b) const {

  float d_x = b.at(0) - a.at(0);
  float d_y = b.at(1) - a.at(1);
  return sqrt( pow(d_x,2) + pow(d_y,2) );
} //End euclideanDistance

/** This method returns the angle that will form a straight line from position a to position b. a and b are [x, y] vectors. */
const float Utility::findAngleFromAToB(const std::vector<float> a, const std::vector<float> b) const {
  float result;

  // Find the distances in x,y directions and Euclidean distance
  float d_x = b.at(0) - a.at(0);
  float d_y = b.at(1) - a.at(1);
  float euc_dist = sqrt( pow(d_x,2) + pow(d_y,2) );
  
  // If the positions are the same,
  // Set the result to the starting orientation if one is provided
  // Or to 0 if no starting orientation is provided
  // TODO: Make the comparison proportionate to size of space
  if(euc_dist <= 0.01) {
    if(a.size() > 2) {
      result = a.at(2);
    }
    else {
      result = 0;
    }
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
    result = difference - PI;
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



/** a and b must be the same size */
const float Utility::getEuclideanDist(const std::vector<float> a, const std::vector<float> b) const {
  float result=0;

  for(unsigned int i=0;i<a.size();i++) {
    result += pow(a.at(i) - b.at(i), 2);
  }
  
  result = sqrt(result);
  return result;
}


const std::string Utility::toString(const ramp_msgs::KnotPoint kp) const {
  std::ostringstream result;

  result<<"\nConfiguration: "<<toString(kp.motionState);
  result<<", Stop time: "<<kp.stopTime;

  return result.str();
}


const std::string Utility::toString(const ramp_msgs::MotionState mp) const {
  std::ostringstream result;

  result<<"\np: [ ";
  for(unsigned int i=0;i<mp.positions.size();i++) {
    result<<mp.positions.at(i)<<" ";
  }
  result<<"]";

  result<<"\nv: [ ";
  for(unsigned int i=0;i<mp.velocities.size();i++) {
    result<<mp.velocities.at(i)<<" ";
  }
  result<<"]";

  result<<"\na: [ ";
  for(unsigned int i=0;i<mp.accelerations.size();i++) {
    result<<mp.accelerations.at(i)<<" ";
  }
  result<<"]";

  result<<"\nj: [ ";
  for(unsigned int i=0;i<mp.jerks.size();i++) {
    result<<mp.jerks.at(i)<<" ";
  }
  result<<"]";

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

  return result.str();
}
