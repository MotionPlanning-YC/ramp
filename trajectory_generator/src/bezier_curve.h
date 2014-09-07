#ifndef BEZIER_CURVE
#define BEZIER_CURVE

#include "utility.h"
#include "reflexxes_data.h"

#define CYCLE_TIME_IN_SECONDS 0.1

struct BezierInitializer {
  // Normal
  std::vector<ramp_msgs::MotionState> sp;
  std::vector<ramp_msgs::MotionState> cp;
  ramp_msgs::MotionState cp_0;
  ramp_msgs::MotionState maxState;
  double lambda;
  double theta;

  // Sub-curve
  double u;


  // Make a constructor to set flags
  BezierInitializer() : lambda(-1), theta(-99), u(0) {}
  BezierInitializer(const ramp_msgs::BezierInfo bi) {
    sp = bi.segmentPoints;
    cp = bi.controlPoints;
  }
};

class BezierCurve {

public:

  BezierCurve();
  ~BezierCurve();

  void init(const BezierInitializer bi);
 
  void init(const std::vector<ramp_msgs::MotionState> sp, const double lambda, const double theta, const ramp_msgs::MotionState initState, const ramp_msgs::MotionState max, double u_0=0.);
 
  void init(const std::vector<ramp_msgs::MotionState> sp, const ramp_msgs::MotionState curveStart, const double theta, const ramp_msgs::MotionState initState, const ramp_msgs::MotionState max, double u_0=0.);
 

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
  ramp_msgs::MotionState ms_init_;
  ramp_msgs::MotionState ms_max_;


  // Variables to manually track some motion info
  double        x_prev_, y_prev_;
  double        x_dot_prev_, y_dot_prev_;
  double        theta_prev_             ;
  double        theta_dot_prev_         ;

  double u_0_;

  void initReflexxes()    ;

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

  const ramp_msgs::MotionState getMS(const double u) const;

};

#endif
