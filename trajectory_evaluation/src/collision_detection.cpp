#include "collision_detection.h"


CollisionDetection::CollisionDetection() {}

CollisionDetection::~CollisionDetection() 
{}

void CollisionDetection::init(ros::NodeHandle& h) {}




/** Returns true if trajectory_ is in collision with any of the objects */
void CollisionDetection::perform(const ramp_msgs::RampTrajectory& trajectory, const std::vector<ramp_msgs::RampTrajectory>& obstacle_trjs, QueryResult& result)  
{
  ros::Duration d_points, d_getMisc, d_linearc;
  ros::Time t_start = ros::Time::now();
  //ROS_INFO("In CollisionDetection::perform()");

  size_t s = obstacle_trjs.size(); 
  size_t s_segment = trajectory.i_knotPoints.size();
  uint8_t segment;
  int8_t ob_i;
  
  //ROS_INFO("obstacle_trjs_.size(): %i", (int)obstacle_trjs_.size());
  // Predict the obstacle's trajectory
  for(ob_i=0;ob_i<s;ob_i++)
  {
    //t_for = ros::Time::now();
    //const ramp_msgs::RampTrajectory& ob_trajectory = obstacle_trjs[ob_i];
    
    ros::Time t_points = ros::Time::now();
    const trajectory_msgs::JointTrajectoryPoint& ob_a = obstacle_trjs[ob_i].trajectory.points[0];
    const trajectory_msgs::JointTrajectoryPoint& ob_b = obstacle_trjs[ob_i].trajectory.points[ 
        obstacle_trjs[ob_i].trajectory.points.size()-1 ];
    bool ob_trj_line = fabs(utility_.findDistanceBetweenAngles(ob_a.positions[2], ob_b.positions[2])) < 0.01;
    d_points = ros::Time::now() - t_points;

    /*ROS_INFO("ob_a: %s", utility_.toString(ob_a).c_str());
    ROS_INFO("ob_b: %s", utility_.toString(ob_b).c_str());
    ROS_INFO("ob_trj_line: %s", ob_trj_line ? "True" : "False");*/
    //d_for = ros::Time::now()-t_for;
    //t_inner_for = ros::Time::now();
    for(segment=1;segment<s_segment && !result.collision_;segment++)
    {
      //ROS_INFO("Segment %i", segment);

      // Check a point in the middle of the segment for angular velocity
      // If there's no angular velocity, then it's a straight-line, otherwise it's a curve
      ros::Time t_getMisc = ros::Time::now();
      uint16_t index = (trajectory.i_knotPoints[segment-1] + trajectory.i_knotPoints[segment]) / 2.f;


      const trajectory_msgs::JointTrajectoryPoint* c = &trajectory.trajectory.points[index];
      double v = sqrt( c->velocities[0]*c->velocities[0] + c->velocities[1]*c->velocities[1] );
      double w = c->velocities[2];
      d_getMisc = ros::Time::now()-t_getMisc;

      // Straight-line
      //if( fabs(utility_.findDistanceBetweenAngles(a.positions.at(2), b.positions.at(2))) < 0.01 )
      ros::Time t_if = ros::Time::now();
      if( v > 0.01 && w*w < 0.0001 )
      {
        //ROS_INFO("t_if: %f", (ros::Time::now()-t_if).toSec());
        //t_if = ros::Time::now();
        // Line-Line
        if(ob_trj_line)
        {
          //ROS_INFO("inner t_if: %f", (ros::Time::now()-t_if).toSec());
          //ROS_INFO("Line Line");
          //t_if = ros::Time::now();
          LineLine(trajectory, segment, obstacle_trjs[ob_i], result);
          //ROS_INFO("LineLine t_if: %f", (ros::Time::now()-t_if).toSec());
        }
        // Line-Arc
        else
        {
          //ROS_INFO("Line Arc");
          ros::Time t_linearc = ros::Time::now();
          LineArc(trajectory, segment, obstacle_trjs[ob_i], result);
          //empty();
          d_linearc = ros::Time::now() - t_linearc;
        }
      }
      // Bezier curve
      else
      {
        // Bezier-Line
        if(ob_trj_line)
        {
          //ROS_INFO("Bezier Line");
          BezierLine(trajectory.curves[0].controlPoints, obstacle_trjs[ob_i], result);
        }
        // Bezier-Arc
        else
        {
          //ROS_INFO("Bezier Arc");
          BezierArc(trajectory.curves[0].controlPoints, obstacle_trjs[ob_i], result); 
        }
      }
      //ROS_INFO("Segment %i Collision: %s", segment, result.collision_ ? "True" : "False");
    } // end for segment
    //d_inner_for = ros::Time::now()-t_inner_for;
  } // end for obstacle */
    

  ros::Time t_done = ros::Time::now();
  //ROS_INFO("t_for: %f", d_for.toSec());
  //ROS_INFO("t_inner_for: %f", d_inner_for.toSec());
  /*ROS_INFO("t_points: %f", d_points.toSec());
  ROS_INFO("t_getMisc: %f", d_getMisc.toSec());
  ROS_INFO("t_linearc: %f", d_linearc.toSec());
  ROS_INFO("t_perform(CD): %f", (t_done - t_start).toSec());*/
  //ROS_INFO("Exiting CollisionDetection::perform()");
} //End perform*/



void CollisionDetection::BezierArc(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const
{
  ros::Time t_start = ros::Time::now();

  //ROS_INFO("w: %f v: %f r: %f", w, v, r);


  // Start sub-dividing the CP
  int i=0; 
  bool coll_array[7] = {false, false, false, false, false, false, false};
  std::vector< std::vector<ramp_msgs::MotionState> > control_poly_tree;
  ros::Time t_tree = ros::Time::now();
  buildTree(control_points, 2, control_poly_tree);
  ros::Duration d_tree = ros::Time::now()-t_tree;


  ros::Time t_for = ros::Time::now();
  //ROS_INFO("control_poly_tree size: %i", (int)control_poly_tree.size());
  for(i=0;i<control_poly_tree.size();i++)
  {
    //std::vector<ramp_msgs::MotionState> control_poly = control_poly_tree.at(i);
    /*ROS_INFO("control_poly:");
    for(int j=0;j<control_poly.size();j++)
    {
      ROS_INFO("Vertex %i: %s", i, utility_.toString(control_poly.at(j)).c_str());
    }*/

    ControlPolyArc(control_poly_tree[i], ob_trajectory, coll_array[i]);
    //ROS_INFO("collision at %i: %s", i, coll_array[i] ? "True" : "False");

    // If no collision with the initial control polygon, return false
    if(!coll_array[i] && i == 0)
    {
      //ROS_INFO("In 1st if");
      qr.collision_ = false;
      break;
    }
    // If no collision with depth-1 polygons
    else if(i == 2 && !coll_array[i] && !coll_array[i-1])
    {
      //ROS_INFO("In 2nd if");
      qr.collision_ = false;
      break;
    }
    // If collision with any depth-n level polygons, return true
    else if(coll_array[i] && i > 2)
    {
      //ROS_INFO("In 3rd if");
      qr.collision_ = true;
      break;
    }
    // If no collision with any depth-n level polygons, return false
    else if(i == control_poly_tree.size()-1 && !coll_array[i])
    {
      //ROS_INFO("In 4th if");
      qr.collision_ = false;
    }
  } // end for
  ros::Duration d_for = ros::Time::now() - t_for;
  ros::Duration d_total = ros::Time::now() - t_start;
  
  ROS_INFO("t_tree: %f", d_tree.toSec());
  ROS_INFO("t_for: %f", d_for.toSec());
  ROS_INFO("t_BezierArc: %f", d_total.toSec());
} // End BezierArc


void CollisionDetection::buildTree(const std::vector<ramp_msgs::MotionState>& control_poly, const int& depth, std::vector< std::vector<ramp_msgs::MotionState> >& result ) const
{

  result.push_back(control_poly);

  for(int i=0;i<pow(2,depth);i++)
  {
    std::vector< std::vector<ramp_msgs::MotionState> > subdivide;
    deCasteljau(result.at(i), subdivide);

    result.push_back(subdivide[0]);
    result.push_back(subdivide[1]);
  }
}



void CollisionDetection::deCasteljau(const std::vector<ramp_msgs::MotionState>& control_poly, std::vector< std::vector<ramp_msgs::MotionState> >& result) const
{
  std::vector<ramp_msgs::MotionState> result_left;
  std::vector<ramp_msgs::MotionState> result_right;

  double t = 0.5f;

  double x_b_1_0 = (1-t)*control_poly[0].positions[0] + t*control_poly[1].positions[0];
  double y_b_1_0 = (1-t)*control_poly[0].positions[1] + t*control_poly[1].positions[1];

  double x_b_1_1 = (1-t)*control_poly[1].positions[0] + t*control_poly.at(2).positions[0];
  double y_b_1_1 = (1-t)*control_poly[1].positions[1] + t*control_poly.at(2).positions[1];

  double x_b_2_0 = (1-t)*x_b_1_0 + t*x_b_1_1;
  double y_b_2_0 = (1-t)*y_b_1_0 + t*y_b_1_1;

  ramp_msgs::MotionState b_1_0;
  b_1_0.positions.push_back(x_b_1_0);
  b_1_0.positions.push_back(y_b_1_0);

  ramp_msgs::MotionState b_1_1;
  b_1_1.positions.push_back(x_b_1_1);
  b_1_1.positions.push_back(y_b_1_1);

  ramp_msgs::MotionState b_2_0;
  b_2_0.positions.push_back(x_b_2_0);
  b_2_0.positions.push_back(y_b_2_0);

  result_left.push_back( control_poly[0] );
  result_left.push_back( b_1_0 );
  result_left.push_back( b_2_0 );

  result_right.push_back( b_2_0 );
  result_right.push_back( b_1_1 );
  result_right.push_back( control_poly[2] );

  result.push_back(result_left);
  result.push_back(result_right);
}




void CollisionDetection::ControlPolyArc(const std::vector<ramp_msgs::MotionState>& con_poly_vert, const ramp_msgs::RampTrajectory& ob_trajectory, bool& result) const
{
  
  // Line-Arc for each segment on control polygon
  for(int i=0;i<con_poly_vert.size();i++)
  {
    int start =  i;
    int end   = (i == con_poly_vert.size()-1) ? 0 : i+1;
    //ROS_INFO("Segment %i", i);
    std::vector<double> l_p1;
    l_p1.push_back(con_poly_vert.at(start).positions.at(0));
    l_p1.push_back(con_poly_vert.at(start).positions.at(1));
    
    std::vector<double> l_p2;
    l_p2.push_back(con_poly_vert.at(end).positions.at(0));
    l_p2.push_back(con_poly_vert.at(end).positions.at(1));
    
    //ROS_INFO("l_p1: (%f, %f)", l_p1.at(0), l_p1.at(1));
    //ROS_INFO("l_p2: (%f, %f)", l_p2.at(0), l_p2.at(1));

    LineArc(l_p1, l_p2, ob_trajectory, result);
  } // end for
} // End ControlPolyArc 


void CollisionDetection::LineLine(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& result) const
{
  ros::Time t_start = ros::Time::now();

  /* Line - Line */

  // Line 1
  //std::vector<double> l1_p1 = trajectory.trajectory.points.at( trajectory.i_knotPoints.at(segment-1)).positions;
  double l1_p1_x = trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[0];

  double l1_p1_y = trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[1];
  
  double l1_p2_x = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[0];

  double l1_p2_y = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[1];
  
  //std::vector<double> l1_p2 = trajectory.trajectory.points.at( trajectory.i_knotPoints.at(segment)).positions;

  double l1_slope = (l1_p2_y - l1_p1_y) / (l1_p2_x - l1_p1_x);
  double l1_b = l1_p2_y - (l1_slope*l1_p2_x);

  //ROS_INFO("line 1 - slope: %f b: %f", l1_slope, l1_b);

  // Line 2
  //std::vector<double> l2_p1 = ob_trajectory.trajectory.points[0].positions;
  //std::vector<double> l2_p2 = ob_trajectory.trajectory.points[1].positions;
  double l2_p1_x = ob_trajectory.trajectory.points[ 0 ].positions[0];

  double l2_p1_y = ob_trajectory.trajectory.points[ 0 ].positions[1];
  
  double l2_p2_x = ob_trajectory.trajectory.points[ 1 ].positions[0];

  double l2_p2_y = ob_trajectory.trajectory.points[ 1 ].positions[1];

  double l2_slope = (l2_p2_y - l2_p1_y) / (l2_p2_x - l2_p1_x);
  double l2_b = l2_p2_y - (l2_slope*l2_p2_x);
  
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
    double l1_x_max = trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[0];
    double l2_x_max = ob_trajectory.trajectory.points[ob_trajectory.trajectory.points.size()-1].positions[0];

    //ROS_INFO("x_intersect: %f l1_x_max: %f l2_x_max: %f", x_intersect, l1_x_max, l2_x_max);

    if( l1_x_max < x_intersect && l2_x_max < x_intersect )
    {
      //ROS_INFO("In if lines intersect");
      result.collision_ = true;
    }
  }

  //ROS_INFO("Query Elapsed time: %f", (ros::Time::now()-t_start).toSec());
} // End LineLine






void CollisionDetection::BezierLine(const std::vector<ramp_msgs::MotionState>& control_points, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const
{
  //ROS_INFO("In CollisionDetection::BezierLine");
  ros::Time t_start = ros::Time::now();


  double S,Q,R,T;

  // Get values for Bezier control points
  double X0 = control_points[0].positions[0];
  double Y0 = control_points[0].positions[1];
  double X1 = control_points[1].positions[0];
  double Y1 = control_points[1].positions[1];
  double X2 = control_points[2].positions[0];
  double Y2 = control_points[2].positions[1];

  //ROS_INFO("Control Points (X0, Y0): (%f, %f) (X1, Y1): (%f, %f) (X2, Y2): (%f, %f)", X0, Y0, X1, Y1, X2, Y2);

  // Get values for line segment
  double x1 = ob_trajectory.trajectory.points[0].positions[0];
  double y1 = ob_trajectory.trajectory.points[0].positions[1];
  double x2 = ob_trajectory.trajectory.points[ ob_trajectory.trajectory.points.size()-1 ].positions[0];
  double y2 = ob_trajectory.trajectory.points[ ob_trajectory.trajectory.points.size()-1 ].positions[1];

  //ROS_INFO("(x1, y1): (%f, %f) (x2, y2): (%f, %f)", x1, y1, x2, y2);


  Q = y2*( X0 - 2*X1 + X2 ) - (y1*( X0 - 2*X1 + X2 ));
  S = x1*( Y0 - 2*Y1 + Y2 ) - (x2*( Y0 - 2*Y1 + Y2 ));
  R = y2*( 2*X1 - 2*X0 ) - (y1*( 2*X1 - 2*X0 ));
  T = x1*( 2*Y1 - 2*Y0 ) - (x2*( 2*Y1 - 2*Y0 ));

  //ROS_INFO("Q: %f R: %f S: %f T: %f", Q, R, S, T);

  
  double A = S+Q;
  double B = R+T;
  double C = y2*X0 - y1*X0 + x1*Y0 + x2*Y0 + x1*(y1-y2) + y1*(x2-x1);

  //ROS_INFO("A: %f B: %f C: %f", A, B, C);

  double u_1, u_2;
  double discriminant = pow(B,2) - (4*A*C);

  //ROS_INFO("Discriminant: %f", discriminant);
  //ROS_INFO("sqrt(Discriminant): %f", sqrt(discriminant));

  u_1 = (-B + sqrt( discriminant )) / (2*A);
  u_2 = (-B - sqrt( discriminant )) / (2*A);
  /*ROS_INFO("-B: %f discriminant: %f", -B, discriminant);
  ROS_INFO("-B - sqrt(discriminant): %f", -B - sqrt(discriminant));
  ROS_INFO("2*A: %f", 2*A);
  ROS_INFO("u_1: %f u_2: %f", u_1, u_2);*/

  double x_min = ob_trajectory.trajectory.points[0].positions[0];
  double x_max = ob_trajectory.trajectory.points[ob_trajectory.trajectory.points.size()-1].positions[0];
  if(x_min > x_max)
  {
    double temp = x_min;
    x_min = x_max;
    x_max = temp;
  }
  //ROS_INFO("x_min: %f x_max: %f", x_min, x_max);
  
  // If intersection values are within bezier domain
  if( (u_1 >= 0.f && u_1 <= 1.f) )
  {
    double bezier_x = pow( (1-u_1), 2 )*X0 + 2*u_1*(1-u_1)*X1 + pow(u_1,2)*X2;
    //ROS_INFO("bezier_x: %f", bezier_x);
    
    // Check if the intersection value is also on the line segment
    if( bezier_x >= x_min && bezier_x <= x_max )
    {
      qr.collision_ = true;
    }
  }

  else if( (u_2 >= 0.f && u_2 <= 1.f) )
  {
    double bezier_x = pow( (1-u_2), 2 )*X0 + 2*u_2*(1-u_2)*X1 + pow(u_2,2)*X2;
    if( bezier_x >= x_min && bezier_x <= x_max )
    {
      qr.collision_ = true;
    }
  }

  else
  {
    qr.collision_ = false;
  }

  /*double test_a = A*pow(u_1,2) + B*u_1 + C;
  double test_b = A*pow(u_2,2) + B*u_2 + C;

  double bezier_x = pow( (1-u_2), 2 )*X0 + 2*u_2*(1-u_2)*X1 + pow(u_2,2)*X2;
  double bezier_y = pow( (1-u_2), 2 )*Y0 + 2*u_2*(1-u_2)*Y1 + pow(u_2,2)*Y2;

  ROS_INFO("test_a: %f, test_b: %f", test_a, test_b);
  ROS_INFO("bezier (x,y): (%f, %f):", bezier_x, bezier_y);*/
      
  //ROS_INFO("BezierLine time: %f", (ros::Time::now()-t_start).toSec());
}



void CollisionDetection::LineArc(const std::vector<double> l_p1, const std::vector<double> l_p2, const ramp_msgs::RampTrajectory& ob_trajectory, bool& result) const
{
  
  // Circle info
  double r, h, k;
  getCircleInfo(ob_trajectory, r, h, k);
  ROS_INFO("Circle info: r: %f h: %f k: %f", r, h, k);
  
  // Line info
  double slope = (l_p2.at(1) - l_p1.at(1)) / (l_p2.at(0) - l_p1.at(0));
  double b = l_p2.at(1) - (slope*l_p2.at(0));
  double m = slope;
  
  // Solve to find values of intersection for x
  double A = pow(slope,2) + 1;
  double B = 2*(m*b - m*k - h);
  double C = pow(h,2) + pow(b,2) + pow(k,2) - pow(r,2) - 2*b*k;
  
  double discriminant = pow(B,2) - 4*A*C;
  double x_intersect_1 = (-B + sqrt(discriminant)) / (2*A);
  double x_intersect_2 = (-B - sqrt(discriminant)) / (2*A);

  double y = slope*x_intersect_1 + b;
  double y_2 = slope*x_intersect_2 + b;
    
  // Get Min/Max values of line segment
  double x_min = l_p1.at(0);
  double x_max = l_p2.at(0);

  if(x_min > x_max)
  {
    double temp = x_min;
    x_min = x_max;
    x_max = temp;
  }
    
  // Get range of x for the arc
  uint16_t i_min = 0;
  uint16_t i_max = ob_trajectory.trajectory.points.size()-1;
    
  // Angles to starting and end points on arc
  double a = utility_.findAngleFromAToB(0, 0, ob_trajectory.trajectory.points[0].positions[0], ob_trajectory.trajectory.points[0].positions[1]);
  double b_angle = utility_.findAngleFromAToB(0, 0, ob_trajectory.trajectory.points[i_max].positions[0], ob_trajectory.trajectory.points[i_max].positions[1]);

  // Angle along circle that the arc begins at
  double starting_angle = utility_.findAngleFromAToB(h, k, ob_trajectory.trajectory.points.at(0).positions[0], ob_trajectory.trajectory.points[0].positions[1]);

  // Find the closest angle divisible by 90deg
  std::vector<double> angles;
  angles.push_back(0);
  angles.push_back(PI/2.f);
  angles.push_back(PI);
  angles.push_back(-PI/2.f);
  double next_axis_angle = angles[0];
  if( fabs(fmodf(starting_angle, (PI/2.f))) < 0.0001)
  {
   //ROS_INFO("In if");
    if(ob_trajectory.trajectory.points[0].velocities[2] > 0)
    {
      next_axis_angle = utility_.displaceAngle(starting_angle, (PI/2.f));
    }
    else
    {
      next_axis_angle = utility_.displaceAngle(starting_angle, -(PI/2.f));
    }
  }
  else
  {
    //ROS_INFO("In else");
    double min_dist = fabs(utility_.findDistanceBetweenAngles(starting_angle, next_axis_angle));
    //ROS_INFO("next_axis_angle: %f min_dist: %f", next_axis_angle, min_dist);
    for(int i=1;i<angles.size();i++)
    {
      //ROS_INFO("angles[%i]: %f", i, angles[i]);
      //ROS_INFO("fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))): %f", fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))));
      if(fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i))) < (min_dist-0.01))
      {
        min_dist = fabs(utility_.findDistanceBetweenAngles(starting_angle, angles.at(i)));
        next_axis_angle = angles.at(i);
      }
    }
  }
  //ROS_INFO("starting_angle: %f next_axis_angle: %f", starting_angle, next_axis_angle);

  std::vector<double> p;
  p.push_back(h + r*cos(next_axis_angle));
  p.push_back(k + r*sin(next_axis_angle));


  std::vector< std::vector<double> > minmax_points;
  minmax_points.push_back(p);
  minmax_points.push_back(ob_trajectory.trajectory.points[i_min].positions);
  minmax_points.push_back(ob_trajectory.trajectory.points[i_max].positions);

  double ob_x_min = minmax_points[0][0];
  double ob_x_max = minmax_points[0][0];
  double ob_y_min = minmax_points[0][1];
  double ob_y_max = minmax_points[0][1];
  for(int i=1;i<minmax_points.size();i++)
  {
    if(minmax_points[i][0] < ob_x_min)
    {
      ob_x_min = minmax_points[i][0];
    }
    if(minmax_points[i][0] > ob_x_max)
    {
      ob_x_max = minmax_points[i][0];
    }
    if(minmax_points[i][1] < ob_y_min)
    {
      ob_y_min = minmax_points[i][1];
    }
    if(minmax_points[i][1] > ob_y_max)
    {
      ob_y_max = minmax_points[i][1];
    }
  }
  
    

  ROS_INFO("isnan(x_intersect_1): %d isnan(x_intersect_2): %d", isnan(x_intersect_1), isnan(x_intersect_2));
  ROS_INFO("discriminant: %f x_intersect_1: %f x_intersect_2: %f y: %f y_2: %f", discriminant, x_intersect_1, x_intersect_2, y, y_2);
  ROS_INFO("Info: x_min: %f x_max: %f ob_x_min: %f ob_x_max: %f ob_y_min: %f ob_y_max: %f", x_min, x_max, ob_x_min, ob_x_max, ob_y_min, ob_y_max);
  
  //ROS_INFO("x_min: %f x_max: %f ob_x_min: %f ob_x_max: %f", x_min, x_max, ob_x_min, ob_x_max);
  if(
      (!isnan(x_intersect_1) && (x_intersect_1 >= x_min && x_intersect_1 <= x_max) && ((x_intersect_1 >= ob_x_min && x_intersect_1 <= ob_x_max) && (y >= ob_y_min && y <= ob_y_max)))  
      || (!isnan(x_intersect_2) && ((x_intersect_2 >= x_min && x_intersect_2 <= x_max) && ((x_intersect_2 >= ob_x_min && x_intersect_2 <= ob_x_max) && (y_2 >= ob_y_min && y_2 <= ob_y_max)))) 
    )
  {
    //ROS_INFO("Collision on segment %i", i);
    result = true;
  }
  else
  {
    result = false;
  }
}








void CollisionDetection::getCircleInfo(const ramp_msgs::RampTrajectory& traj, double& r, double& h, double& k) const
{
  
  // Get circle info
  double w = traj.trajectory.points[0].velocities[2];
  double v = sqrt((traj.trajectory.points[0].velocities[0] * traj.trajectory.points[0].velocities[0]) + (traj.trajectory.points[0].velocities[1]*traj.trajectory.points[0].velocities[1]) );
  r = fabs(v / w);

  ROS_INFO("v: %f w: %f r: %f", v, w, r);

  //ROS_INFO("w: %f v: %f r: %f", w, v, r);

  // Compute circle center!
  const trajectory_msgs::JointTrajectoryPoint* p1 = &traj.trajectory.points[0];
  const trajectory_msgs::JointTrajectoryPoint* p2 = &traj.trajectory.points[ traj.trajectory.points.size() / 2.f];
  const trajectory_msgs::JointTrajectoryPoint* p3 = &traj.trajectory.points[
      traj.trajectory.points.size()-1];


  double q = utility_.positionDistance(p1->positions, p2->positions);

  double x_mid = (p1->positions[0] + p2->positions[0]) / 2.f;
  double y_mid = (p1->positions[1] + p2->positions[1]) / 2.f;

  double x_dir = p1->positions[0] - p2->positions[0];
  double y_dir = p1->positions[1] - p2->positions[1];

  double x_dir_per = y_dir / q;
  double y_dir_per = -x_dir / q;
  
  double p0_x = x_mid;      double p0_y = y_mid;
  double r0_x = x_dir_per;  double r0_y = y_dir_per;

  // Get second mid line
  x_mid = (p2->positions.at(0) + p3->positions.at(0)) / 2.f;
  y_mid = (p2->positions.at(1) + p3->positions.at(1)) / 2.f;

  x_dir = p2->positions.at(0) - p3->positions.at(0);
  y_dir = p2->positions.at(1) - p3->positions.at(1);

  x_dir_per = y_dir / q;
  y_dir_per = -x_dir / q;
  
  double p1_x = x_mid;      double p1_y = y_mid;
  double r1_x = x_dir_per;  double r1_y = y_dir_per;
  
  // Find intersection of the two mid lines that are in vector form
  double s = (r0_y*(p1_x-p0_x) + r0_x*(p0_y-p1_y)) / (r1_y*r0_x - r0_y*r1_x);
  double t = ( p1_x - p0_x +s*r1_x) / r0_x;
  
  //ROS_INFO("s: %f t: %f", s, t);
  //ROS_INFO("a_x: %f b_x: %f a_y: %f b_y: %f", a_x, b_x, a_y, b_y);

  //ROS_INFO("q: %f mid: (%f, %f) dir: (%f, %f) dir_per: (%f, %f)", q, x_mid, y_mid, x_dir, y_dir, x_dir_per, y_dir_per);
  //ROS_INFO("h: %f k: %f r: %f", h, k, r);

  // Compute h and k
  h = p0_x + r0_x*t;
  k = p0_y + r0_y*t;
}


void CollisionDetection::LineArc(const ramp_msgs::RampTrajectory& trajectory, const int& segment, const ramp_msgs::RampTrajectory& ob_trajectory, QueryResult& qr) const
{
  //ROS_INFO("In CollisionDetection::LineArc");
  ros::Time t_start = ros::Time::now();


  // Get Line info
  std::vector<double> l_p1;
  l_p1.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[0]);
  l_p1.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment-1]].positions[1]);
  
  std::vector<double> l_p2;
  l_p2.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[0]);
  l_p2.push_back(trajectory.trajectory.points[ trajectory.i_knotPoints[segment]].positions[1]);


  double slope = (l_p2[1] - l_p1[1]) / (l_p2[0] - l_p1[0]);
  double b = l_p2[1] - (slope*l_p2[0]);

  LineArc(l_p1, l_p2, ob_trajectory, qr.collision_);

  ROS_INFO("qr.collision_: %s", qr.collision_ ? "True" : "False");

  //ros::Duration d_lineinfo = ros::Time::now() - t_start;
 

  
  /*ROS_INFO("d_firstthreecalls: %f", d_firstthreecalls.toSec());
  ROS_INFO("d_angle: %f", d_angle.toSec());
  ROS_INFO("d_lineinfo: %f", d_lineinfo.toSec());
  ROS_INFO("d_solve: %f", d_solve.toSec());
  ROS_INFO("d_swaps: %f", d_swaps.toSec());
  ROS_INFO("d_check: %f", d_check.toSec());
  ROS_INFO("d_circleinfo: %f", d_circleinfo.toSec());
  ROS_INFO("d_findangle: %f", d_findangle.toSec());
  ROS_INFO("d_findminmax: %f", d_findminmax.toSec());*/
  //ROS_INFO("LineArc time: %f", d_linearc.toSec());
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








