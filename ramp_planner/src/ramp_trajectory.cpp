#include "ramp_trajectory.h"

RampTrajectory::RampTrajectory(const float resRate, unsigned int id) {
  msg_.id = id;
  msg_.feasible = true;
  msg_.fitness = -1;  
  msg_.t_firstCollision = ros::Duration(9999.f);
  msg_.t_start          = ros::Duration(2.0f);
  msg_.resolutionRate = resRate;
}

RampTrajectory::RampTrajectory(const ramp_msgs::RampTrajectory msg) : msg_(msg) {}


const bool RampTrajectory::equals(const RampTrajectory& other) const {
  if(msg_.id == other.msg_.id) 
  {
    return true;
  }

  return path_.equals(other.path_);
}


const double RampTrajectory::getT() const
{
  return msg_.trajectory.points.at(msg_.trajectory.points.size()-1).time_from_start.toSec();
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
  ROS_INFO("t: %f resolutionRate: %f i: %i size: %i", 
      t, 
      resolutionRate, 
      i, 
      (int)msg_.trajectory.points.size());

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
    for(float i=0.f;i<=t_stop+0.000001;i+=0.1f) 
    { 
      uint16_t index = floor(i*10.) < msg_.trajectory.points.size() ? floor(i*10) : 
        msg_.trajectory.points.size()-1;

      //ROS_INFO("index: %i size: %i", index, (int)msg_.trajectory.points.size());
      rt.trajectory.points.push_back(msg_.trajectory.points.at(index)); 
      if(msg_.i_knotPoints.at(i_kp) == index) 
      {
        rt.i_knotPoints.push_back(index);
        i_kp++;
      } // end if
    } // end for
    
    // If the last point was not a knot point, make it one for the sub-trajectory
    if(rt.i_knotPoints.at( rt.i_knotPoints.size()-1 ) != rt.trajectory.points.size()-1)
    {
      rt.i_knotPoints.push_back(rt.trajectory.points.size()-1);
    }
  }

  RampTrajectory result(rt);

  return result;
}


const RampTrajectory RampTrajectory::getSubTrajectoryPost(const double t) const
{
  ROS_INFO("In RampTrajectory::getSubTrajectoryPost");
  RampTrajectory rt;

  double t_start = t;

  if(msg_.trajectory.points.size() < 1)
  {
    ROS_ERROR("msg_.trajectory.points.size == 0");
    return clone();
  }
  else if(msg_.trajectory.points.size() == 1 ||
      t > msg_.trajectory.points.at( msg_.trajectory.points.size()-1 ).time_from_start.toSec())
  {
    rt.msg_.trajectory.points.push_back(msg_.trajectory.points.at(0));
    rt.msg_.i_knotPoints.push_back(0);
  }
  
  
  else
  {
    rt.msg_.i_knotPoints.push_back(0);
    uint8_t i_kp = 1;
    
    // Push on all the points
    //for(float i=t_start;i<=t_stop;i+=0.1f) 
    for(int i=floor(t_start*10.);i<msg_.trajectory.points.size();i++) 
    {

      // Get point
      trajectory_msgs::JointTrajectoryPoint p = msg_.trajectory.points.at(i);
      
      // Adjust time
      p.time_from_start = ros::Duration(p.time_from_start.toSec() - t_start);

      // Add to trajectory
      rt.msg_.trajectory.points.push_back(p); 

      // If it's a knot point, push on it's index
      if(msg_.i_knotPoints.at(i_kp) == i) 
      {
        rt.msg_.i_knotPoints.push_back(rt.msg_.trajectory.points.size()-1);
        i_kp++;
      }
    } // end for


    // If the last point was not a knot point, make it one for the sub-trajectory
    if(rt.msg_.i_knotPoints.at( rt.msg_.i_knotPoints.size()-1 ) != rt.msg_.trajectory.points.size()-1)
    {
      rt.msg_.i_knotPoints.push_back(rt.msg_.trajectory.points.size()-1);
    }

    // Make a path
    /*Path temp(rt.msg_.trajectory.points.at(0), rt.msg_.trajectory.points.at(rt.msg_.trajectory.points.size()-1));
    for(uint8_t i=0;i<path_.size();i++)
    {
      if(path_.at(i).motionState_.msg_.time > t_start)
      {
        temp.addBeforeGoal(path_.at(i));
      }
    }

    if(msg_.i_curveEnd >= (t*10.))
    {
      KnotPoint k(msg_.curves.at(0).segmentPoints.at(0));
      temp.all_.insert(temp.all_.begin()+1, k);
    }

    rt.path_ = temp;*/
  } // end else

  ROS_INFO("Returning sub-trajectory: %s", rt.toString().c_str());
  ROS_INFO("Exiting RampTrajectory::getSubTrajectoryPost");
  return rt;
}




/*
 * Concatenate traj onto this trajectory. kp is the knot to start on traj
 */
const RampTrajectory RampTrajectory::concatenate(const RampTrajectory traj, const uint8_t kp) const 
{
  ROS_INFO("In RampTrajectory::concatenate");
  RampTrajectory result = clone();
  uint8_t c_kp = kp+1;

  if(msg_.trajectory.points.size() == 0)
  {
    ROS_WARN("msg_trajectory.points.size() == 0, Returning parameter traj");
    return traj;
  }

  if(traj.msg_.trajectory.points.size() == 0)
  {
    ROS_WARN("traj.msg_trajectory.points.size() == 0, Returning *this");
    return *this;
  }
  
  if(traj.msg_.i_knotPoints.size() == 1)
  {
    ROS_WARN("traj.msg_.i_knotPoints.size() <= kp: %i, returning *this", (int)kp);
    return *this;
  }

  if(traj.msg_.i_knotPoints.size() <= kp)
  {
    ROS_WARN("traj.msg_.i_knotPoints.size() <= kp: %i, returning *this", (int)kp);
    return *this;
  }


  /*
   * Test that the last point in this trajectory matches the first point in traj
   */
  trajectory_msgs::JointTrajectoryPoint last  = msg_.trajectory.points.at(
                                                msg_.trajectory.points.size()-1);
  trajectory_msgs::JointTrajectoryPoint first = traj.msg_.trajectory.points.at(traj.msg_.i_knotPoints.at( kp ));
  if( fabs(utility_.positionDistance(last.positions, first.positions)) > 0.1)
  {
    ROS_WARN("First and last points don't match!");
    ROS_WARN("last: %s\nfirst: %s\ndiff: %f", utility_.toString(last).c_str(), utility_.toString(first).c_str(), fabs(utility_.positionDistance(last.positions, first.positions)));
    return traj;
  }

  ROS_INFO("traj.msg_.trajectory.points.size(): %i", (int)traj.msg_.trajectory.points.size());
  ROS_INFO("kp: %i", kp);
  ROS_INFO("traj.msg_.i_knotPoints.size(): %i", (int)traj.msg_.i_knotPoints.size());
  trajectory_msgs::JointTrajectoryPoint endOfFirstSegment = traj.msg_.trajectory.points.at(traj.msg_.i_knotPoints.at( kp+1 ));
  // If the last segment of this and first segment of traj have the same orientation
  // Remove the last knot point of this trajectory
  if( msg_.curves.size() == 0 && utility_.findDistanceBetweenAngles(last.positions.at(2), first.positions.at(2)) < 0.01)
  {
    ROS_INFO("Last segment of this and first segment of traj have the same orientation");
    ROS_INFO("last.positions.at(2): %f first.positions.at(2): %f", 
        last.positions.at(2), first.positions.at(2));
    ROS_INFO("Removing last knotpoint of this trajectory");
    result.msg_.i_knotPoints.pop_back();
  }


  ros::Duration t_cycleTime(0.1);
  ros::Duration t_latest = msg_.trajectory.points.at(msg_.trajectory.points.size()-1).time_from_start;
  for(uint16_t  i=traj.msg_.i_knotPoints.size() == 1 ? 0 : traj.msg_.i_knotPoints.at(c_kp-1)+1 ;
                i<traj.msg_.trajectory.points.size() ;
                i++)
  {
    trajectory_msgs::JointTrajectoryPoint temp = traj.msg_.trajectory.points.at(i);

    // Set time
    temp.time_from_start = ros::Duration(t_latest+t_cycleTime);
    t_latest += t_cycleTime;

    result.msg_.trajectory.points.push_back(temp);

    if( i == traj.msg_.i_knotPoints.at(c_kp) )
    {
      ROS_INFO("i: %i traj.msg_.i_knotPoints.at(%i): %i", i, c_kp, traj.msg_.i_knotPoints.at(c_kp));
      ROS_INFO("temp: %s", utility_.toString(temp).c_str());
      result.msg_.i_knotPoints.push_back( result.msg_.trajectory.points.size()-1 ); 
      c_kp++;
    }
  } //end for


  // Push on the target trajectory's Bezier curve
  for(uint8_t i_curve=0;i_curve<traj.msg_.curves.size();i_curve++) {
    result.msg_.curves.push_back(traj.msg_.curves.at(i_curve));
  }

  /*ROS_INFO("Path before concat: %s", result.path_.toString().c_str());

  // Set the correct Path 
  for(uint8_t i=0;i<traj.path_.size();i++)
  {
    ROS_INFO("traj path[%i]: %s", i, traj.path_.at(i).toString().c_str());
    result.path_.addBeforeGoal(traj.path_.at(i));
  }
  ROS_INFO("New path: %s", result.path_.toString().c_str());*/

  // Set i_curveEnd for the 1st curve
  /*if(result.msg_.curves.size() > 0) {
    ROS_INFO("In switching curve if");
    result.msg_.i_curveEnd = switching.msg_.i_curveEnd;
  }
  else if(result.msg_.curves.size() == 0) {
    ROS_INFO("In switching curve result.curves.size() == 0");
    result.msg_.i_curveEnd = 0;
  }
  else {
    ROS_INFO("In switching curve else");
    ROS_INFO("to.msg_.i_curveEnd: %i", (int)traj.msg_.i_curveEnd);
    ROS_INFO("to_msg.curves.size(): %i", (int)traj.msg_.curves.size());
    result.msg_.i_curveEnd = traj.msg_.curves.at(0).numOfPoints + switching.msg_.trajectory.points.size(); 
  }*/


  ROS_INFO("result: %s", result.toString().c_str());
  ROS_INFO("Exiting RampTrajectory::concatenate");
  return result;
}



const RampTrajectory RampTrajectory::clone() const { 
  return *this;
}


const std::string RampTrajectory::fitnessFeasibleToString() const {
  std::ostringstream result;
 
  result<<"\nTrajectory ID: "<<msg_.id;
  //result<<"\n Path: "<<bezierPath_.toString();
  result<<"\n Path: "<<getPath().toString();
  result<<"\n t_start: "<<msg_.t_start.toSec()<<" Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;

  return result.str();
}

const std::string RampTrajectory::toString() const {
  std::ostringstream result;
  
  result<<"\nTrajectory ID: "<<msg_.id<<"\nTrajec: "<<utility_.toString(msg_);
  result<<"\n Fitness: "<<msg_.fitness<<" Feasible: "<<(bool)msg_.feasible<<" Collision Time: "<<msg_.t_firstCollision;
  result<<"\n   Transition traj size: "<<transitionTraj_.trajectory.points.size();
  
  return result.str();
}




