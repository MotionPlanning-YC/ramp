#ifndef BEZIER_CURVE
#define BEZIER_CURVE

#include "utility.h"
#include "reflexxes_data.h"

#define CYCLE_TIME_IN_SECONDS 0.1

class BezierCurve {

public:

  BezierCurve();
  ~BezierCurve();
  
  void init(const std::vector<ramp_msgs::MotionState> sp, const double lambda);
  const std::vector<ramp_msgs::MotionState> generateCurve();
  

  double A_, B_, C_, D_ ;
  double t_min_         ;
  double R_min_         ;
  std::vector<ramp_msgs::MotionState> segment_points_;
  std::vector<ramp_msgs::MotionState> control_points_;
  std::vector<ramp_msgs::MotionState> points_;


private:

  Utility       utility_      ;
  ReflexxesData reflexxesData_;
  double        lambda_       ;
  bool          initialized_  ;

  void initReflexxes()    ;
  void initControlPoints();

  void calculateConstants() ;
  void calculateABCD()      ;
  void calculateT_min()     ;
  void calculateR_min()     ;

  const bool finalStateReached() const;

  const ramp_msgs::MotionState spinOnce();

};

#endif
