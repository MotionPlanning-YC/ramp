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
    //LineLine(trajectory, obstacle_trjs.at(i), result);
    LineArc(trajectory, obstacle_trjs.at(i), result);
    //BezierLine(trajectory, obstacle_trjs.at(i), result);
  }

  //ros::Time t_done = ros::Time::now();
  //ROS_INFO("t_perform(CD): %f", (t_done - t_start).toSec());
  //ROS_INFO("Exiting CollisionDetection::perform()");
} //End perform



void CollisionDetection::LineLine(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& result) const
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
    
    // Same line
    if( fabs(l1_slope - l2_slope) < 0.01 && fabs(l1_b - l2_b) < 0.01 )
    {
      result.collision_ = true;
    }
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
} // End LineLine






void CollisionDetection::BezierLine(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const
{
  ROS_INFO("In CollisionDetection::BezierLine");
  ros::Time t_start = ros::Time::now();


  double S,Q,R,T;

  // Get values for Bezier control points
  double X0 = trajectory.curves.at(0).controlPoints.at(0).positions.at(0);
  double Y0 = trajectory.curves.at(0).controlPoints.at(0).positions.at(1);
  double X1 = trajectory.curves.at(0).controlPoints.at(1).positions.at(0);
  double Y1 = trajectory.curves.at(0).controlPoints.at(1).positions.at(1);
  double X2 = trajectory.curves.at(0).controlPoints.at(2).positions.at(0);
  double Y2 = trajectory.curves.at(0).controlPoints.at(2).positions.at(1);

  ROS_INFO("Control Points (X0, Y0): (%f, %f) (X1, Y1): (%f, %f) (X2, Y2): (%f, %f)",
      X0, Y0, X1, Y1, X2, Y2);

  // Get values for line segment
  double x1 = ob_trajectory.trajectory.points.at(0).positions.at(0);
  double y1 = ob_trajectory.trajectory.points.at(0).positions.at(1);
  double x2 = ob_trajectory.trajectory.points.at( ob_trajectory.trajectory.points.size()-1 ).positions.at(0);
  double y2 = ob_trajectory.trajectory.points.at( ob_trajectory.trajectory.points.size()-1 ).positions.at(1);

  ROS_INFO("(x1, y1): (%f, %f) (x2, y2): (%f, %f)", x1, y1, x2, y2);


  Q = y2*( X0 - 2*X1 + X2 ) - y1*( X0 - 2*X1 + X2 );
  S = x1*( Y0 - 2*Y1 + Y2 ) - x2*( Y0 - 2*Y1 + Y2 );
  R = y2*( 2*X1 - 2*X0) - y1*( 2*X1 - 2*X0 );
  T = x1*( 2*Y1 - 2*Y0 ) - x2*( 2*Y1 - 2*Y0 );

  ROS_INFO("Q: %f R: %f S: %f T: %f", Q, R, S, T);

  
  double A = S+Q;
  double B = R+T;
  double C = y2*X0 - y1*X0 + x1*Y0 + x2*Y0 + x1*(y1-y2) + y1*(x2-x1);

  ROS_INFO("A: %f B: %f C: %f", A, B, C);

  double u_1, u_2;

  ROS_INFO("Discriminant: %f", pow(B,2) - 4*A*C);
  ROS_INFO("sqrt(Discriminant): %f", sqrt(pow(B,2) - 4*A*C));

  u_1 = (-B + sqrt( pow(B,2) - 4*A*C )) / 2*A;
  u_2 = (-B - sqrt( pow(B,2) - 4*A*C )) / 2*A;
  
  ROS_INFO("u_1: %f u_2: %f", u_1, u_2);

  ROS_INFO("BezierLine time: %f", (ros::Time::now()-t_start).toSec());
}






void CollisionDetection::LineArc(const ramp_msgs::RampTrajectory& trajectory, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const
{
  ros::Time t_start = ros::Time::now();

  // Get Line info
  std::vector<double> l_p1 = trajectory.trajectory.points.at(0).positions;
  std::vector<double> l_p2 = trajectory.trajectory.points.at(1).positions;

  double slope = (l_p2.at(1) - l_p1.at(1)) / (l_p2.at(0) - l_p1.at(0));
  double b = l_p2.at(1) - (slope*l_p2.at(0));


  // Get circle info
  double w = ob_trajectory.trajectory.points.at(0).velocities.at(2);
  double v = sqrt(pow(ob_trajectory.trajectory.points.at(0).velocities.at(0),2) + pow(ob_trajectory.trajectory.points.at(0).velocities.at(1),2) );
  double r = v / w;

  ROS_INFO("w: %f v: %f r: %f", w, v, r);

  double h, k;

  trajectory_msgs::JointTrajectoryPoint p1 = ob_trajectory.trajectory.points.at(0);
  trajectory_msgs::JointTrajectoryPoint p2 = ob_trajectory.trajectory.points.at( ob_trajectory.trajectory.points.size() 
      / 2.f);
  trajectory_msgs::JointTrajectoryPoint p3 = ob_trajectory.trajectory.points.at( 
      ob_trajectory.trajectory.points.size()-1);


  double q = utility_.positionDistance(p1.positions, p2.positions);

  double x_mid = (p1.positions.at(0) + p2.positions.at(0)) / 2.f;
  double y_mid = (p1.positions.at(1) + p2.positions.at(1)) / 2.f;

  double x_dir = p1.positions.at(0) - p2.positions.at(0);
  double y_dir = p1.positions.at(1) - p2.positions.at(1);

  double x_dir_per = y_dir / q;
  double y_dir_per = -x_dir / q;

  std::vector<double> p_vec0;
  p_vec0.push_back(x_mid);
  p_vec0.push_back(y_mid);
  std::vector<double> r_vec0;
  r_vec0.push_back(x_dir_per);
  r_vec0.push_back(y_dir_per);

  // Get second mid line
  x_mid = (p2.positions.at(0) + p3.positions.at(0)) / 2.f;
  y_mid = (p2.positions.at(1) + p3.positions.at(1)) / 2.f;

  x_dir = p2.positions.at(0) - p3.positions.at(0);
  y_dir = p2.positions.at(1) - p3.positions.at(1);

  x_dir_per = y_dir / q;
  y_dir_per = -x_dir / q;
  
  std::vector<double> p_vec1;
  p_vec1.push_back(x_mid);
  p_vec1.push_back(y_mid);
  std::vector<double> r_vec1;
  r_vec1.push_back(x_dir_per);
  r_vec1.push_back(y_dir_per);

  // Find intersection of two lines that are in vector form
  double p0_x = p_vec0.at(0); double p0_y = p_vec0.at(1);
  double p1_x = p_vec1.at(0); double p1_y = p_vec1.at(1);
  double r0_x = r_vec0.at(0); double r0_y = r_vec0.at(1);
  double r1_x = r_vec1.at(0); double r1_y = r_vec1.at(1);

  double s = (r0_y*(p1_x-p0_x) + r0_x*(p0_y-p1_y)) / (r1_y*r0_x - r0_y*r1_x);

  double t = ( p1_x - p0_x +s*r1_x) / r0_x;

  double a_x = p0_x + r0_x*t;
  double a_y = p0_y + r0_y*t;
  double b_x = p1_x + r1_x*s;
  double b_y = p1_y + r1_y*s;
  
  ROS_INFO("s: %f t: %f", s, t);
  ROS_INFO("a_x: %f b_x: %f a_y: %f b_y: %f", a_x, b_x, a_y, b_y);
  

  // Comment out the "Get second mid line" section for this to work...
  h = a_x;
  k = a_y;


  ROS_INFO("q: %f mid: (%f, %f) dir: (%f, %f) dir_per: (%f, %f)",
      q, x_mid, y_mid, x_dir, y_dir, x_dir_per, y_dir_per);
  ROS_INFO("h: %f k: %f r: %f", h, k, r);

  double d = b;
  double m = slope;
  double A = pow(slope,2) + 1;
  double B = 2*(m*d - m*k - h);
  double C = pow(h,2) + pow(d,2) + pow(k,2) - pow(r,2) - 2*d*k;

  ROS_INFO("d: %f m: %f A: %f B: %f C: %f", d, m, A, B, C);

  double discriminant = pow(B,2) - 4*A*C;
  double x_intersect_1 = (-B + sqrt(discriminant)) / (2*A);
  double x_intersect_2 = (-B - sqrt(discriminant)) / (2*A);

  double y = slope*x_intersect_1 + b;
  double y_2 = pow( (x_intersect_1 - h),2 ) + pow( y-k, 2 );
  
  ROS_INFO("x_intersect_1: %f y: %f y_2: %f r^2: %f", x_intersect_1, y, y_2, pow(r,2));

  ROS_INFO("discriminant: %f sqrt(discriminant): %f", discriminant, sqrt(discriminant));
  ROS_INFO("-B + sqrt(discriminant): %f", -B+sqrt(discriminant));
  ROS_INFO("-B - sqrt(discriminant): %f", -B-sqrt(discriminant));
  
  ROS_INFO("2*A: %f", 2*A);
  ROS_INFO("x_intersect_1: %f x_intersect_2: %f", x_intersect_1, x_intersect_2);


  double x_min = trajectory.trajectory.points.at(0).positions.at(0);
  double x_max = trajectory.trajectory.points.at(trajectory.trajectory.points.size()-1).positions.at(0);
  if( x_min > x_max )
  {
    double temp = x_min;
    x_min = x_max;
    x_max = temp;
  }
  if( (x_intersect_1 >= x_min && x_intersect_1 <= x_max) ||
      (x_intersect_2 >= x_min && x_intersect_2 <= x_max )  )
  {
    qr.collision_ = true;
  }
  else
  {
    qr.collision_ = false;
  }
  ROS_INFO("LineArc time: %f", (ros::Time::now()-t_start).toSec());
} // End LineArc



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








