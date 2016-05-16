#include "collision_detection.h"


CollisionDetection::CollisionDetection() {}

CollisionDetection::~CollisionDetection() 
{}

void CollisionDetection::init(ros::NodeHandle& h) {}




/** Returns true if trajectory_ is in collision with any of the objects */
void CollisionDetection::perform(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, QueryResult& result) const 
{
  //ros::Time t_start = ros::Time::now();
  //ROS_INFO("In CollisionDetection::perform()");

  int s = obstacle_trjs.size(); 
  
  //ROS_INFO("obstacle_trjs_.size(): %i", (int)obstacle_trjs_.size());
  // Predict the obstacle's trajectory
  for(int i=0;i<s;i++)
  {
    //ramp_msgs::RampTrajectory ob_trajectory = obstacle_trjs.at(i);

    // Query for collision
    //CollisionDetection::QueryResult q = query(ob_trajectory);
    //CollisionDetection::QueryResult q = queryAnalytical(ob_trajectory);
    //CollisionDetection::QueryResult q = queryAnalytical(obstacle_trjs_.at(i));
    queryAnalytical(trajectory, obstacle_trjs.at(i), result);
  }

  //ros::Time t_done = ros::Time::now();
  //ROS_INFO("t_perform(CD): %f", (t_done - t_start).toSec());
  //ROS_INFO("Exiting CollisionDetection::perform()");
} //End perform



void CollisionDetection::queryAnalytical(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& result) const
{
  ros::Time t_start = ros::Time::now();

  /* Line - Line */

  // Line 1
  std::vector<double> l1_p1 = trajectory.trajectory.points.at(0).positions;
  std::vector<double> l1_p2 = trajectory.trajectory.points.at(1).positions;

  double l1_slope = (l1_p2.at(1) - l1_p1.at(1)) / (l1_p2.at(0) - l1_p1.at(0));
  double l1_b = l1_p2.at(1) - (l1_slope*l1_p2.at(0));

  //ROS_INFO("line 1 - slope: %f b: %f", l1_slope, l1_b);

  // Line 2
  std::vector<double> l2_p1 = ob_trajectory.trajectory.points.at(0).positions;
  std::vector<double> l2_p2 = ob_trajectory.trajectory.points.at(1).positions;

  double l2_slope = (l2_p2.at(1) - l2_p1.at(1)) / (l2_p2.at(0) - l2_p1.at(0));
  double l2_b = l2_p2.at(1) - (l2_slope*l2_p2.at(0));
  
  //ROS_INFO("line 2 - slope: %f b: %f", l2_slope, l2_b);
  
  // Parallel lines
  // Check that they are not the same line!
  if( fabs(l2_slope - l1_slope) < 0.01 )
  {
    //ROS_INFO("Lines are parallel");
  }
  else
  {
    //ROS_INFO("Lines are not parallel");
    double x_intersect = (-(l1_b - l2_b)) / (l1_slope - l2_slope);
    double l1_x_max = trajectory.trajectory.points.at(trajectory.trajectory.points.size()-1).positions.at(0);
    double l2_x_max = ob_trajectory.trajectory.points.at(ob_trajectory.trajectory.points.size()-1).positions.at(0);

    //ROS_INFO("x_intersect: %f l1_x_max: %f l2_x_max: %f", x_intersect, l1_x_max, l2_x_max);

    if( l1_x_max < x_intersect && l2_x_max < x_intersect )
    {
      //ROS_INFO("In if lines intersect");
      result.collision_ = true;
    }
  }


  //ROS_INFO("Query Elapsed time: %f", (ros::Time::now()-t_start).toSec());
}

/** 
 * This method returns true if there is collision between trajectory_ and the obstacle's trajectory, false otherwise 
 * The robots are treated as circles for simple collision detection
 */
const CollisionDetection::QueryResult CollisionDetection::query(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory) const 
{
  ros::Time time_start = ros::Time::now();
  //ROS_INFO("In CollisionDetection::query"); 
  //ROS_INFO("trajectory.points.size(): %i", (int)trajectory.trajectory.points.size());
  //ROS_INFO("ob_trajectory.points.size(): %i", (int)ob_trajectory.trajectory.points.size());
  /*if(ob_trajectory.trajectory.points.size() > 2)
  {
    ROS_INFO("ob_trajectory: %s", utility_.toString(ob_trajectory).c_str());
  }*/

  double t_start = trajectory.t_start.toSec();
  int j_offset = t_start * 10.f;
  //ROS_INFO("t_start: %f j_offset: %i", t_start, j_offset);

  CollisionDetection::QueryResult result;
  uint8_t t_checkColl = 3;

  /*if(ob_trajectory.trajectory.points.size() <= 2) {
    if(id == 0)
      std::cout<<"\nRobot 1 has no trajectory!\n";
    else  
      std::cout<<"\nRobot 0 has no trajectory!\n";
  }*/
 
  // Find the point that ends the trajectory's non-holonomic section
  uint16_t i_stop = 0;

  // If there are no curves
  // If there is a curve and only two knot points (curve ends traj)
  if(t_start < 0.01)
  {
    //ROS_INFO("In 1st if");
    i_stop = trajectory.i_knotPoints.at(trajectory.i_knotPoints.size()-1);
  }
  else if(   trajectory.curves.size() == 0 ||
      ( trajectory.curves.size() == 1 && trajectory.i_knotPoints.size() == 2) )
  {
    //ROS_INFO("In 2nd if");
    i_stop = trajectory.i_knotPoints.at(1);
  }
  
  // If there's only one curve 
  //  (If no transition traj, then two segments)
  //  (If transition traj, then only one segment)
  else if(trajectory.curves.size() == 1)
  {
    //ROS_INFO("In 3rd if");
    i_stop = trajectory.i_knotPoints.at(2);
  }

  // If there's two curves
  else
  {
    //ROS_INFO("In 4th if");
    i_stop = trajectory.i_knotPoints.at(3);
  }
 
  int j_start;
 
  //ROS_INFO("i_stop: %i", i_stop);
  
  // For every point, check circle detection on a subset of the obstacle's trajectory
  float radius = 0.19f;
  for(uint16_t i=0;i<i_stop;i++) 
  {
    
    // Get the ith point on the trajectory
    trajectory_msgs::JointTrajectoryPoint p_i = trajectory.trajectory.points.at(i);

    //ROS_INFO("p_i: %s", utility_.toString(p_i).c_str());

    
    // Compute which point on the obstacle trajectory to start doing collision checking
    if(ob_trajectory.trajectory.points.size() == 1)
    {
      j_start = 0;
      t_checkColl = 0;
    }
    else if(i <= t_checkColl)
    {
      j_start = 0+j_offset;
    }
    else
    {
      j_start = (i-t_checkColl)+j_offset;
    }

    //ROS_INFO("j_start: %i", j_start);

    // *** Test position i for collision against some points on obstacle's trajectory ***
    // Obstacle trajectory should already be in world coordinates!
    for(int j = j_start;
        j<=(i+t_checkColl+j_offset) && j<ob_trajectory.trajectory.points.size();
        j++)
    {
    /*if(ob_trajectory.trajectory.points.size() > 2)
    {
      ROS_INFO("i: %i j: %i", i, j);
    }*/

      // Get the jth point of the obstacle's trajectory
      trajectory_msgs::JointTrajectoryPoint p_ob  = ob_trajectory.trajectory.points.at(j);

      // Get the distance between the centers
      float dist = sqrt( pow(p_i.positions.at(0) - p_ob.positions.at(0),2) + pow(p_i.positions.at(1) - p_ob.positions.at(1),2) );

    /*if(ob_trajectory.trajectory.points.size() > 2)
    {
      ROS_INFO("Comparing trajectory point (%f,%f) and obstacle point (%f,%f): dist = %f", 
          p_i.positions.at(0), p_i.positions.at(1), 
          p_ob.positions.at(0), p_ob.positions.at(1), 
          dist);
    }*/
        

      // If the distance between the two centers is less than the sum of the two radii, 
      // there is collision
      if( dist <= radius*2 ) 
      {
        /*ROS_INFO("Points in collision: (%f,%f), and (%f,%f), dist: %f i: %i j: %i",
            p_i.positions.at(0),
            p_i.positions.at(1),
            p_ob.positions.at(0),
            p_ob.positions.at(1),
            dist,
            (int)i,
            (int)j);*/
        result.collision_ = true;
        result.t_firstCollision_ = p_i.time_from_start.toSec();
        i = i_stop;
        break;
      } // end if
    } // end for
  } // end for

  //ROS_INFO("result: %s", result.collision_ ? "True" : "False");
  //ROS_INFO("Exiting CollisionDetection::query");
  //ROS_INFO("Query Elapsed time: %f", (ros::Time::now()-time_start).toSec());
  return result;
} //End query








