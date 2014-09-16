#include "bezier_curve.h"


BezierCurve::BezierCurve() : initialized_(false), deallocated_(false) {
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters  = 0;
  reflexxesData_.outputParameters = 0;
}

BezierCurve::~BezierCurve() {
  if(!deallocated_) {
    dealloc(); 
  }
}


void BezierCurve::dealloc() {
  if(!deallocated_) {
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

    deallocated_ = true;
  }
}



void BezierCurve::init(const ramp_msgs::BezierInfo bi, const ramp_msgs::MotionState ms_current) {
  segmentPoints_  = bi.segmentPoints;
  lambda_         = bi.lambda;
  theta_prev_     = utility_.findAngleFromAToB(
                                  segmentPoints_.at(0).positions, 
                                  segmentPoints_.at(1).positions);
 
  ms_max_     = bi.ms_maxVA;
  ms_current_ = ms_current;

  if(bi.ms_initialVA.velocities.size() > 0) {
    ms_init_ = bi.ms_initialVA;
  }
  else {
    ms_init_ = getInitialState();
  }
  std::cout<<"\nAfter setting initial state\n";

  u_0_ = bi.u_0;
  u_dot_0_ = bi.u_dot_0; 

  if(bi.controlPoints.size() > 0) {
    initControlPoints(bi.controlPoints.at(0));
  }
  else {
    initControlPoints();
  }
  std::cout<<"\nAfter setting control points\n";

  calculateConstants();

  // Set where the robot begins the curve
  if(bi.ms_begin.positions.size() > 0) {
    std::cout<<"\nSetting ms_begin to bi.ms_begin\n";
    std::cout<<utility_.toString(bi.ms_begin);
    ms_begin_ = bi.ms_begin;
  }
  else {
    std::cout<<"\nSetting ms_begin to first control point\n";
    ms_begin_ = controlPoints_.at(0);
  }
  std::cout<<"\nAfter setting ms_begin_\n";
  x_prev_         = ms_begin_.positions.at(0);
  y_prev_         = ms_begin_.positions.at(1);
  x_dot_prev_ = ms_begin_.velocities.at(0);
  y_dot_prev_ = ms_begin_.velocities.at(1);
  theta_prev_ = ms_begin_.positions.at(2);
  theta_dot_prev_ = ms_begin_.velocities.at(2);


  // If both C and D == 0, the first two points are the same
  if(fabs(C_) > 0.0001 && fabs(D_) > 0.0001) {
  
    initReflexxes();
    
    initialized_ = true;
  } 

  else {
    std::cout<<"\nThe 2 points are the same:\n";
    std::cout<<"\nSegment points: ";
    for(int i=0;segmentPoints_.size();i++) {
      std::cout<<"\n"<<i<<": "<<utility_.toString(segmentPoints_.at(i));
    }
    std::cout<<"\nControl points: ";
    for(int i=0;controlPoints_.size();i++) {
      std::cout<<"\n"<<i<<": "<<utility_.toString(controlPoints_.at(i));
    }
    std::cout<<"\n";
  }
}




void BezierCurve::printReflexxesInfo() const {

  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];

  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[0]: "<<
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0];

  std::cout<<"\n\nreflexxesData_.inputParameters->TargetPositionVector->VecData[0]: "<<
    reflexxesData_.inputParameters->TargetPositionVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->TargetVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->TargetVelocityVector->VecData[0]<<"\n";
} // End printReflexxesInfo




const double BezierCurve::findVelocity(const uint8_t i, const double s) const {
  // s = s_0 + v_0*t + 1/2*a*t^2
  // t = (v - v_0) / a;
  
  // Use 2/3 of max acceleration
  double a = (2.*ms_max_.accelerations.at(i)/3.);

  // Use the current velocity as initial
  double v_0 = ms_current_.velocities.size() > 0 ?
                ms_current_.velocities.at(i) : 0;

  double radicand = (2*a*s) + pow(v_0, 2);
  double v = sqrt(radicand);

  // Check for bounds
  if(v > ms_max_.velocities.at(i)) {
    v = ms_max_.velocities.at(i);
  }
  if(v < -ms_max_.velocities.at(i)) {
    v = -ms_max_.velocities.at(i);
  }

  return v;
} // End findVelocity





const ramp_msgs::MotionState BezierCurve::getInitialState() {
  //std::cout<<"\nIn getInitialState\n";

  ramp_msgs::MotionState result;
  for(uint8_t i=0;i<3;i++) {
    result.velocities.push_back(0);
  }

  // Find the slope
  double ryse = segmentPoints_.at(1).positions.at(1) - 
                segmentPoints_.at(0).positions.at(1);
  double run  = segmentPoints_.at(1).positions.at(0) - 
                segmentPoints_.at(0).positions.at(0);
  double slope  = (run != 0) ? ryse / run : ryse;
  if(print_) {
    std::cout<<"\nryse: "<<ryse<<" run: "<<run;
    std::cout<<"\nslope: "<<slope<<"\n";
  }
  
  // Segment 1 size
  double s = lambda_ * utility_.positionDistance(
      segmentPoints_.at(0).positions, 
      segmentPoints_.at(1).positions);


  // If change in y is greater
  if( (slope >= 1)  ||
      (slope == -1 && run < 0) ||
      (slope < -1) ) 
  {
    result.velocities.at(1) = findVelocity(1, s);
    result.velocities.at(0) = result.velocities.at(1) / slope;  
  }
  // if slope == -1 && ryse < 0
  // if slope < 0
  // else
  else {
    result.velocities.at(0) = findVelocity(0, s);
    result.velocities.at(1) = result.velocities.at(0) * slope;
  }

  result.accelerations.push_back(0);
  result.accelerations.push_back(0);
  

  return result;
} // End getInitialState




/** Returns true if u_dot satisfies the motion constraints 
 *  given a u value - they may be different when testing for u_dot_max */
const bool BezierCurve::satisfiesConstraints(const double u_dot, const double u_x, const double u_y) const {
  if(print_) {
    std::cout<<"\nTesting constraints for "<<u_dot;
    std::cout<<"\n(A_*u_x+C_)*u_dot: "<<(A_*u_x+C_)*u_dot<<" x_dot_max: "<<ms_max_.velocities.at(0);
    std::cout<<"\n(B_*u_y+D_)*u_dot: "<<(B_*u_y+D_)*u_dot<<" y_dot_max: "<<ms_max_.velocities.at(1);
  }
  
  // Square them in case they are negative 
  // Add .0001 because I was getting 0.33 > 0.33
  if( pow( (A_*u_x+C_)*u_dot,2) > pow((ms_max_.velocities.at(0))+0.0001,2) ||
      pow( (B_*u_y+D_)*u_dot,2) > pow((ms_max_.velocities.at(1))+0.0001,2) )
  {
    return false;
  }

  return true;
}




const double BezierCurve::getUDotMax(const double u_dot_0) const {
  //std::cout<<"\n\n***** Calculating u_dot_max *****\n";
  double x_dot_max = ms_max_.accelerations.at(0);
  double y_dot_max = ms_max_.accelerations.at(1);
  //std::cout<<"\nx_dot_max: "<<x_dot_max<<" y_dot_max: "<<y_dot_max;

  // Initialize variables
  double u_dot_max;
  double u_x = ( fabs(A_+C_) > fabs(C_) ) ? 1 : 0;
  double u_y = ( fabs(B_+D_) > fabs(D_) ) ? 1 : 0;
  double u_dot_max_y = B_*u_y + D_ == 0 ? 0 : fabs(x_dot_max / (B_*u_y+D_));
  double u_dot_max_x = A_*u_x + C_ == 0 ? 0 : fabs(y_dot_max / (A_*u_x+C_));


  if(print_) {
    std::cout<<"\nu_x: "<<u_x<<" u_y: "<<u_y;
    std::cout<<"\nu_dot_max_x: "<<u_dot_max_x<<" u_dot_max_y: "<<u_dot_max_y;
  }

  // Set a greater and lesser value
  double greater, lesser;
  if(u_dot_max_x > u_dot_max_y) {
    greater = u_dot_max_x;
    lesser = u_dot_max_y;
  }
  else {
    greater = u_dot_max_y;
    lesser = u_dot_max_x;
  }


  /** Set u_dot_max*/

  // If both are zero
  if(u_dot_max_x == 0 && u_dot_max_y == 0) {
    ROS_ERROR("u_dot_max_x == 0 && u_dot_max_y == 0");
    u_dot_max = 0;
  }

  // Test greater
  else if(satisfiesConstraints(greater, u_x, u_y)) {
    u_dot_max = greater;
  }

  // If greater too large, test lesser
  else if(satisfiesConstraints(lesser, u_x, u_y)) {
    u_dot_max = lesser;    
  }

  // Else, set it to initial u_dot
  else {
    u_dot_max = u_dot_0;
  }



  return u_dot_max;
} // End getUDotMax




const double BezierCurve::getUDotInitial() const {
  std::cout<<"\n***** Calculating u_dot_0 *****\n";
  std::cout<<"\nms_begin: "<<utility_.toString(ms_begin_);
  std::cout<<"\nms_initVA: "<<utility_.toString(ms_init_);
  double x_dot_0        = (ms_begin_.velocities.size() > 0) ? ms_begin_.velocities.at(0)      : 
                            ms_init_.velocities.at(0);
  double y_dot_0        = (ms_begin_.velocities.size() > 0) ? ms_begin_.velocities.at(1)      : 
                            ms_init_.velocities.at(1);
  std::cout<<"\nx_dot_0: "<<x_dot_0<<" y_dot_0: "<<y_dot_0;
  std::cout<<"\nu_0: "<<u_0_<<" u_dot_0: "<<u_dot_0_;
  
  double u_dot_0_x = fabs(x_dot_0 / (A_*u_0_+C_));
  double u_dot_0_y = fabs(y_dot_0 / (B_*u_0_+D_));
  std::cout<<"\nu_dot_0_x: "<<u_dot_0_x;
  std::cout<<"\nu_dot_0_y: "<<u_dot_0_y;

  // Set a greater and lesser value
  double greater, lesser;
  if(u_dot_0_x > u_dot_0_y) {
    greater = u_dot_0_x;
    lesser = u_dot_0_y;
  }
  else {
    greater = u_dot_0_y;
    lesser = u_dot_0_x;
  }

  // If both are zero
  if(u_dot_0_x == 0 && u_dot_0_y == 0) {
    ROS_ERROR("u_dot_0_x == 0 && u_dot_0_y == 0");
    return 0;
  }

  // Test greater
  else if(satisfiesConstraints(greater, u_0_, u_0_)) {
    return greater;
  }

  // If greater too large, test lesser
  else if(satisfiesConstraints(lesser, u_0_, u_0_)) {
    return lesser;    
  }

  else {
    ROS_ERROR("Neither u_dot_0 values satisfy constraints");
    return 0;
  }
} // End getUDotInitial



/** This method initializes the necessary Reflexxes variables */
void BezierCurve::initReflexxes() {
  std::cout<<"\nIn initReflexxes\n";

  // Set some variables for readability
  double x_dot_0        = ms_begin_.velocities.at(0);
  double y_dot_0        = ms_begin_.velocities.at(1);
  double x_dot_max      = ms_max_.velocities.at(0);
  double y_dot_max      = ms_max_.velocities.at(1);
  double x_dot_dot_max  = ms_max_.accelerations.at(0);
  double y_dot_dot_max  = ms_max_.accelerations.at(1);

  // Initialize Reflexxes variables
  reflexxesData_.rml              = new ReflexxesAPI( 1, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters  = new RMLPositionInputParameters( 1 );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( 1 );

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;


  // Get initial and maximum velocity of Bezier parameter 
  double u_dot_0 = getUDotInitial(); 
  if(u_dot_0_ != 0) {
    u_dot_0 = u_dot_0_;
  }
  else {
    u_dot_0_ = u_dot_0;
  }
  

  double u_dot_max  = getUDotMax(u_dot_0);
  if(print_) {
    std::cout<<"\nx_dot0: "<<x_dot_0<<" y_dot0: "<<y_dot_0;
    std::cout<<"\nu_dot_0: "<<u_dot_0<<" u_dot_max: "<<u_dot_max<<"\n";
  }


  // Set the position and velocity Reflexxes variables
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0]     = u_0_;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]     = u_dot_0_;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]         = u_dot_max;
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[0]:"<<reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]:"<<reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];

  // Set u max acceleration
  // We don't actually use this, but it's necessary for Reflexxes to work
  if(A_ + C_ != 0) {
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 
      fabs( (y_dot_dot_max - B_*u_dot_max) / (B_+D_) );
  }
  else if (B_ + D_ != 0) {
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 
      fabs( (x_dot_dot_max - A_*u_dot_max) / (A_+C_) );
  }
  else {
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 0.1;
  }

  
  // Set targets
  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = 1;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = 
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
  

  if(print_) {
    printReflexxesInfo();
    std::cin.get();
  }


  reflexxesData_.resultValue = 0;
  std::cout<<"\nLeaving initReflexxes\n";
} // End initReflexxes



/** Initialize control points 
 *  Sets the first control point and then calls overloaded initControlPoints */
void BezierCurve::initControlPoints() {
  ramp_msgs::MotionState C0, p0, p1;

  // Set segment points
  p0 = segmentPoints_.at(0);
  p1 = segmentPoints_.at(1);

  // Set orientation of the two segments
  double theta_s1 = utility_.findAngleFromAToB( p0.positions, 
                                                p1.positions);

  /** Positions */
  C0.positions.push_back( (1-lambda_)*p0.positions.at(0) + lambda_*p1.positions.at(0) );
  C0.positions.push_back( (1-lambda_)*p0.positions.at(1) + lambda_*p1.positions.at(1) );
  C0.positions.push_back(theta_s1);

  initControlPoints(C0);
} // End initControlPoints






/** Initialize the control points of the Bezier curve given the first one */
void BezierCurve::initControlPoints(const ramp_msgs::MotionState cp_0) {
  ramp_msgs::MotionState C0, C1, C2, p0, p1, p2;


  // Set segment points
  p0 = segmentPoints_.at(0);
  p1 = segmentPoints_.at(1);
  p2 = segmentPoints_.at(2);

  // Set orientation of the two segments
  double theta_s1 = utility_.findAngleFromAToB( p0.positions, 
                                                p1.positions);
  double theta_s2 = utility_.findAngleFromAToB( p1.positions, 
                                                p2.positions);

  // Control point 0 is passed in
  // Control Point 1 is the 2nd segment point
  C0 = cp_0;
  C1 = segmentPoints_.at(1);
  C1.positions.at(2) = theta_s1;

  /** Set 3rd control point */
  // s1 = segment distance between first two control points
  double s1 = sqrt( pow(C1.positions.at(0) - C0.positions.at(0), 2) +
                    pow(C1.positions.at(1) - C0.positions.at(1), 2) );

  // Get x,y positions of the 3rd control point
  double x = C1.positions.at(0) + s1*cos(theta_s2);
  double y = C1.positions.at(1) + s1*sin(theta_s2);

  // Length of second segment
  double l2 = sqrt( pow(p2.positions.at(0) - p1.positions.at(0), 2) +
                    pow(p2.positions.at(1) - p1.positions.at(1), 2) );

  // If s1 is greater than entire 2nd segment,
  // set 3rd control point to end of 2nd segment
  if(s1 > l2) {
    C2.positions.push_back(p2.positions.at(0));  
    C2.positions.push_back(p2.positions.at(1));
  }
  else {
    C2.positions.push_back(x);  
    C2.positions.push_back(y);
  }
  C2.positions.push_back(theta_s2);


  /** C0 Velocities */
  if(C0.velocities.size() == 0) {
    C0.velocities.push_back(ms_init_.velocities.at(0));
    C0.velocities.push_back(ms_init_.velocities.at(1));
    C0.velocities.push_back(0);
  }
  /** C0 Accelerations */
  if(C0.accelerations.size() == 0) {
    C0.accelerations.push_back(0);
    C0.accelerations.push_back(0);
    C0.accelerations.push_back(0);
  }



  // Push on all the points
  controlPoints_.push_back(C0);
  controlPoints_.push_back(C1);
  controlPoints_.push_back(C2);
  

  //if(print_) {
    std::cout<<"\nSegment Points:";
    for(int i=0;i<segmentPoints_.size();i++) {
      std::cout<<"\n"<<utility_.toString(segmentPoints_.at(i));
    }
    std::cout<<"\nControl Points:";
    for(int i=0;i<controlPoints_.size();i++) {
      std::cout<<"\n"<<utility_.toString(controlPoints_.at(i));
    }
  //}
} // End initControlPoints





/** Returns true when Reflexxes has reached its targets */
const bool BezierCurve::finalStateReached() const {
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}


void BezierCurve::calculateABCD() {
  ramp_msgs::MotionState p0 = controlPoints_.at(0);
  ramp_msgs::MotionState p1 = controlPoints_.at(1);
  ramp_msgs::MotionState p2 = controlPoints_.at(2);

  // A = 2(X0-2X1+X2)
  A_ = 2 * (p0.positions.at(0) - (2*p1.positions.at(0)) + p2.positions.at(0));

  // B = 2(Y0-2Y1+Y2)
  B_ = 2 * (p0.positions.at(1) - (2*p1.positions.at(1)) + p2.positions.at(1));

  // C = 2(X1-X0)
  C_ = 2 * (p1.positions.at(0) - p0.positions.at(0));

  // D = 2(Y1-Y0)
  D_ = 2 * (p1.positions.at(1) - p0.positions.at(1));
}




/** Calculate the minimum radius along the curve */
void BezierCurve::calculateR_min() {

  double numerator_term_one   = ((A_*A_) + (B_*B_)) * (t_R_min_*t_R_min_);
  double numerator_term_two   = 2 * ((A_*C_)+(B_*D_)) * t_R_min_;
  double numerator_term_three = (C_*C_) + (D_*D_);
  double numerator            = pow(numerator_term_one + numerator_term_two + numerator_term_three, 3); 

  double denominator          = pow((B_*C_) - (A_*D_), 2);
 
  R_min_                      = sqrt( numerator / denominator );
}


/** Calculate time when minimum radius occurs along the curve */
void BezierCurve::calculateT_R_min() {
  t_R_min_ = -((A_*C_) + (B_*D_)) / ((A_*A_) + (B_*B_));
}


/** Calculate A,B,C,D, minimum radius, and time of minimum radius */
void BezierCurve::calculateConstants() {
  calculateABCD();
  calculateT_R_min();
  calculateR_min();
  
  //if(print_) {
    std::cout<<"\nA: "<<A_<<" B: "<<B_<<" C: "<<C_<<" D: "<<D_<<"\n";
  //}
}



/** Generate all the motion states on the curve */
const std::vector<ramp_msgs::MotionState> BezierCurve::generateCurve() {
  std::cout<<"\nIn generateCurve\n";
  if(initialized_) {

    reflexxesData_.resultValue = 0;
   
    points_.push_back(ms_begin_);
    std::cout<<"\nCurrent Position: "<<reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
    std::cout<<"\nCurrent Velocity: "<<reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]<<"\n";

    while(!finalStateReached()) {
      points_.push_back(spinOnce());
    }

    dealloc();
  }

  std::cout<<"\nLeaving generateCurve\n";
  return points_;
} // End generateCurve







/** Call Reflexxes once and return the next motion state */
// TODO: Clean up?
const ramp_msgs::MotionState BezierCurve::spinOnce() {
  ramp_msgs::MotionState result;


  // Call Reflexxes
  reflexxesData_.resultValue = reflexxesData_.rml->RMLPosition( 
                                 *reflexxesData_.inputParameters, 
                                  reflexxesData_.outputParameters, 
                                  reflexxesData_.flags );
  
  // Set variables to make equations more readable
  double u          = reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  double u_dot      = reflexxesData_.outputParameters->NewVelocityVector->VecData[0];
  double u_dot_dot  = reflexxesData_.outputParameters->NewAccelerationVector->VecData[0];
  double X0         = controlPoints_.at(0).positions.at(0);
  double X1         = controlPoints_.at(1).positions.at(0);
  double X2         = controlPoints_.at(2).positions.at(0);
  double Y0         = controlPoints_.at(0).positions.at(1);
  double Y1         = controlPoints_.at(1).positions.at(1);
  double Y2         = controlPoints_.at(2).positions.at(1);

  /** Create new point */
  // Position
  double x      = (pow((1-u),2) * X0) + ((2*u)*(1-u)*X1) + (pow(u,2)*X2);
  double y      = (pow((1-u),2) * Y0) + ((2*u)*(1-u)*Y1) + (pow(u,2)*Y2);
  double theta  = utility_.findAngleFromAToB(x_prev_, y_prev_, x, y);

  
  // Velocity
  double x_dot = ((A_*u) + C_)*u_dot;
  double y_dot = (x_dot*(B_*u+D_)) / (A_*u+C_);
  double theta_dot      = (theta - theta_prev_) / CYCLE_TIME_IN_SECONDS;

  // Acceleration
  double x_dot_dot = (x_dot - x_dot_prev_) / CYCLE_TIME_IN_SECONDS;
  double y_dot_dot = (y_dot - y_dot_prev_) / CYCLE_TIME_IN_SECONDS;
  double theta_dot_dot  = (theta_dot - theta_dot_prev_) / CYCLE_TIME_IN_SECONDS;
    //double x_dot_dot = u_dot_dot*(A_*u+C_) + A_*u_dot;
    //double y_dot_dot = u_dot_dot*(B_*u+D_) + B_*u_dot;


  // Set previous motion values 
  x_prev_         = x;
  y_prev_         = y;
  x_dot_prev_     = x_dot;
  y_dot_prev_     = y_dot;
  theta_prev_     = theta;
  theta_dot_prev_ = theta_dot;
  
  if(print_) {
    std::cout<<"\n\nu: "<<u<<" u_dot: "<<u_dot<<" u_dot_dot: "<<u_dot_dot;
    std::cout<<"\nx: "<<x<<"           y: "<<y;
    std::cout<<"\nx_dot: "<<x_dot<<"      y_dot: "<<y_dot;
    std::cout<<"\nx_dot_dot: "<<x_dot_dot<<" y_dot_dot: "<<y_dot_dot;
  }

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
  
  // Set current vectors to the output 
  *reflexxesData_.inputParameters->CurrentPositionVector = *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = *reflexxesData_.outputParameters->NewAccelerationVector;


  return result;
} // End spinOnce
