#ifndef BEZIER_CURVE
#define BEZIER_CURVE

#include "utility.h"
#include "reflexxes_data.h"

#define CYCLE_TIME_IN_SECONDS 0.1

class BezierCurve {

public:

  BezierCurve();
  ~BezierCurve();
  
  void init(const std::vector<ramp_msgs::MotionState> sp, const double lambda, const double theta, const double x_dot_0, const double y_dot_0, const double x_dot_dot_0, const double y_dot_dot_0, const double x_dot_max, const double y_dot_max, const double x_dot_dot_max, const double y_dot_dot_max, double u_0=0.);
  void init(const std::vector<ramp_msgs::MotionState> sp, const ramp_msgs::MotionState curveStart, const double theta, const double x_dot_0, const double y_dot_0, const double x_dot_dot_0, const double y_dot_dot_0, const double x_dot_max, const double y_dot_max, const double x_dot_dot_max, const double y_dot_dot_max, double u_0=0);

  const std::vector<ramp_msgs::MotionState> generateCurve();
  

  double A_, B_, C_, D_       ;
  double R_min_               ;
  double t_R_min_             ;
  double lambda_              ;
  std::vector<ramp_msgs::MotionState> segment_points_ ;
  std::vector<ramp_msgs::MotionState> control_points_ ;
  std::vector<ramp_msgs::MotionState> points_         ;

  void initControlPoints();
  void initControlPoints(const ramp_msgs::MotionState start);
  bool print_;

private:

  Utility       utility_            ;
  ReflexxesData reflexxesData_      ;
  bool          initialized_        ;
  bool          deallocated_        ;
  double        x_init_v_, y_init_v_;
  double        x_init_a_, y_init_a_;
  double        x_dot_max_, y_dot_max_;

  // Variables to manually track some motion info
  double        x_prev_, y_prev_;
  double        x_dot_prev_, y_dot_prev_;
  double        theta_prev_             ;
  double        theta_dot_prev_         ;

  void initReflexxes(const double x_dot_max, const double y_dot_max, const double x_dot_dot_max, const double y_dot_dot_max)    ;

  void calculateConstants() ;
  void calculateABCD()      ;
  void calculateT_R_min()     ;
  void calculateR_min()     ;

  const bool finalStateReached() const;

  const ramp_msgs::MotionState spinOnce();

  void dealloc();

  const bool satisfiesConstraints(const double u_dot_max, const double u_x, const double u_y) const;
  const double getUDotMax(const double u_dot_0) const;

  void printReflexxesInfo() const;

};

#endif
