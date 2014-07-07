#include "bezier_curve.h"


BezierCurve::BezierCurve() {
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters  = 0;
  reflexxesData_.outputParameters = 0;
}


BezierCurve::~BezierCurve() {
  
  if(reflexxesData_.rml != 0) {
    delete reflexxesData_.rml;
    reflexxesData_.rml = 0;
  }

  if(reflexxesData_.inputParameters != 0) {
    delete reflexxesData_.inputParameters;
    reflexxesData_.inputParameters = 0;
  }
  
  if(reflexxesData_.outputParameters != 0) {
    delete reflexxesData_.outputParameters;
    reflexxesData_.outputParameters = 0;
  }
}




void BezierCurve::init(const std::vector<ramp_msgs::MotionState> cp, const double lambda, const double theta) {
  segment_points_ = cp;
  lambda_ = lambda;
  
  initControlPoints();
  
  calculateConstants();

  initReflexxes();

  theta_prev_ = theta;
  
  initialized_ = true;
}




void BezierCurve::initReflexxes() {
  reflexxesData_.rml = new ReflexxesAPI( 1, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters = new RMLPositionInputParameters( 1 );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( 1 );
  
  reflexxesData_.flags.SynchronizationBehavior = RMLPositionFlags::NO_SYNCHRONIZATION;

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;
  
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]     = fabs(0.33 / A_+C_);
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = fabs(0.33 / B_);
  reflexxesData_.inputParameters->MaxJerkVector->VecData[0]         = 1;
  
  std::cout<<"\nA: "<<A_<<" B: "<<B_<<" C: "<<C_<<" D: "<<D_;
  std::cout<<"\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[0]: "<<reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[0]: "<<reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0];
  std::cin.get();

  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0]     = 0.;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]     = 0.;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 0.;

  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = 1;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = 0;
  
  reflexxesData_.resultValue = 0;
  
}




void BezierCurve::initControlPoints() {
  ramp_msgs::MotionState X0;
  ramp_msgs::MotionState X2;

  ramp_msgs::MotionState p0 = segment_points_.at(0);
  ramp_msgs::MotionState p1 = segment_points_.at(1);
  ramp_msgs::MotionState p2 = segment_points_.at(2);

  /** Positions */
  X0.positions.push_back( (1-lambda_)*p0.positions.at(0) + lambda_*p1.positions.at(0) );
  X0.positions.push_back( (1-lambda_)*p0.positions.at(1) + lambda_*p1.positions.at(1) );
  X0.positions.push_back(0);

  X2.positions.push_back( (1-lambda_)*p1.positions.at(0) + lambda_*p2.positions.at(0) );
  X2.positions.push_back( (1-lambda_)*p1.positions.at(1) + lambda_*p2.positions.at(1) );
  X2.positions.push_back(0);


  /** Velocities */

  double x_dot, y_dot;
  double x_diff = fabs(p1.positions.at(0) - p0.positions.at(0));
  double y_diff = fabs(p1.positions.at(1) - p0.positions.at(1));

  if(x_diff > y_diff) {
    if( (x_diff < 0 && y_diff > 0) || (x_diff > 0 && y_diff < 0) ) {
      x_dot =  -reflexxesData_.inputParameters->MaxVelocityVector->VecData[1];
      y_dot =   reflexxesData_.inputParameters->MaxVelocityVector->VecData[1];
    } 
    else {
      x_dot =   reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
      y_dot =  -reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
    }
  }

  X0.velocities.push_back(x_dot);
  X0.velocities.push_back(y_dot);
  X0.velocities.push_back(0);


  /** Accelerations */

  X0.accelerations.push_back(0);
  X0.accelerations.push_back(0);
  X0.accelerations.push_back(0);



  control_points_.push_back(X0);
  control_points_.push_back(segment_points_.at(1));
  control_points_.push_back(X2);

}


const bool BezierCurve::finalStateReached() const {
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}


void BezierCurve::calculateABCD() {
  ramp_msgs::MotionState p0 = control_points_.at(0);
  ramp_msgs::MotionState p1 = control_points_.at(1);
  ramp_msgs::MotionState p2 = control_points_.at(2);

  // A = 2(X0-2X1+X2)
  A_ = 2 * (p0.positions.at(0) - (2*p1.positions.at(0)) + p2.positions.at(0));

  // B = 2(Y0-2Y1+Y2)
  B_ = 2 * (p0.positions.at(1) - (2*p1.positions.at(1)) + p2.positions.at(1));

  // C = 2(X1-X0)
  C_ = 2 * (p1.positions.at(0) - p0.positions.at(0));

  // D = 2(Y1-Y0)
  D_ = 2 * (p1.positions.at(1) - p0.positions.at(1));
}




void BezierCurve::calculateR_min() {

  double numerator_term_one   = ((A_*A_) + (B_*B_)) * (t_min_*t_min_);
  double numerator_term_two   = 2 * ((A_*C_)+(B_*D_)) * t_min_;
  double numerator_term_three = (C_*C_) + (D_*D_);
  double numerator            = pow(numerator_term_one + numerator_term_two + numerator_term_three, 3); 

  double denominator          = pow((B_*C_) - (A_*D_), 2);
 
  R_min_                      = sqrt( numerator / denominator );
}


void BezierCurve::calculateT_min() {
  t_min_ = -((A_*C_) + (B_*D_)) / ((A_*A_) + (B_*B_));
}



void BezierCurve::calculateConstants() {
  calculateABCD();
  calculateT_min();
  calculateR_min();
}




const std::vector<ramp_msgs::MotionState> BezierCurve::generateCurve() {
  std::vector<ramp_msgs::MotionState> result;

  reflexxesData_.resultValue = 0;
 
  result.push_back(control_points_.at(0));

  while(!finalStateReached()) {
    result.push_back(spinOnce());
  }


  return result;
}




const ramp_msgs::MotionState BezierCurve::spinOnce() {
  ramp_msgs::MotionState result;

  reflexxesData_.resultValue = reflexxesData_.rml->RMLPosition( 
                                 *reflexxesData_.inputParameters, 
                                  reflexxesData_.outputParameters, 
                                  reflexxesData_.flags );
  
  // Set u's p, v, a
  double u_prev          = reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  double u_dot_prev      = reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];
  double u_dot_dot_prev  = reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0];
  
  // Set u's p, v, a
  double u          = reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  double u_dot      = reflexxesData_.outputParameters->NewVelocityVector->VecData[0];
  double u_dot_dot  = reflexxesData_.outputParameters->NewAccelerationVector->VecData[0];
  std::cout<<"\nu: "<<u<<" u_dot: "<<u_dot<<" u_dot_dot: "<<u_dot_dot;


  ramp_msgs::MotionState X0 = control_points_.at(0);
  ramp_msgs::MotionState X1 = control_points_.at(1);
  ramp_msgs::MotionState X2 = control_points_.at(2);

  // Position
  double x = (pow((1-u),2) * X0.positions.at(0)) + ((2*u)*(1-u)*X1.positions.at(0)) + (pow(u,2)*X2.positions.at(0));
  double y = (pow((1-u),2) * X0.positions.at(1)) + ((2*u)*(1-u)*X1.positions.at(1)) + (pow(u,2)*X2.positions.at(1));
  double x_prev = (pow((1-u_prev),2) * X0.positions.at(0)) + ((2*u_prev)*(1-u_prev)*X1.positions.at(0)) + (pow(u_prev,2)*X2.positions.at(0));
  double y_prev = (pow((1-u_prev),2) * X0.positions.at(1)) + ((2*u_prev)*(1-u_prev)*X1.positions.at(1)) + (pow(u_prev,2)*X2.positions.at(1));

  double theta  = utility_.findAngleFromAToB(x_prev, y_prev, x, y);
  double theta_dot = (theta - theta_prev_) / CYCLE_TIME_IN_SECONDS;
  // TODO: Fix theta acceleration
  double theta_dot_dot = (theta_dot - theta_dot_prev_) / CYCLE_TIME_IN_SECONDS;

  theta_prev_ = theta;
  theta_dot_prev_ = theta_dot;
  
  // Velocity
  double x_dot = ((A_*u) + C_)*u_dot;
  double y_dot = ((B_*u) + D_)*u_dot;

  // Acceleration
  double x_dot_dot = A_ * u_dot_dot;
  double y_dot_dot = B_ * u_dot_dot;

  // Push values onto MotionState
  result.positions.push_back(x);
  result.positions.push_back(y);
  result.positions.push_back(theta);

  result.velocities.push_back(x_dot);
  result.velocities.push_back(y_dot);
  result.velocities.push_back(theta_dot);

  result.accelerations.push_back(x_dot_dot);
  result.accelerations.push_back(y_dot_dot);
  result.accelerations.push_back(theta_dot_dot);
  
  // Set current vectors the output 
  *reflexxesData_.inputParameters->CurrentPositionVector = *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = *reflexxesData_.outputParameters->NewAccelerationVector;


  return result;
}
