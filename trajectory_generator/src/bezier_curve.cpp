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


void BezierCurve::init(const std::vector<ramp_msgs::MotionState> sp, const double lambda, const double theta, const double x_dot_0, const double y_dot_0, const double x_dot_dot_0, const double y_dot_dot_0, const double x_dot_max, const double y_dot_max, const double x_dot_dot_max, const double y_dot_dot_max) {
  segment_points_ = sp;
  lambda_         = lambda;
  x_init_v_       = x_dot_0;
  y_init_v_       = y_dot_0;
  x_init_a_       = x_dot_dot_0;
  y_init_a_       = y_dot_dot_0;
  x_dot_max_      = x_dot_max;
  y_dot_max_      = y_dot_max;
  
  initControlPoints();
  calculateConstants();

  // If both C and D == 0, the first two points are the same
  if(C_ > 0.0001 && D_ > 0.0001) {
  
    initReflexxes(x_dot_max, y_dot_max, x_dot_dot_max, y_dot_dot_max);

    theta_prev_ = theta;
    
    initialized_ = true;
  } 

  else {
    std::cout<<"\nThe 2 points are the same:\n";
    std::cout<<"\nSegment points: ";
    for(int i=0;segment_points_.size();i++) {
      std::cout<<"\n"<<i<<": "<<utility_.toString(segment_points_.at(i));
    }
    std::cout<<"\nControl points: ";
    for(int i=0;control_points_.size();i++) {
      std::cout<<"\n"<<i<<": "<<utility_.toString(control_points_.at(i));
    }
    std::cout<<"\n";
  }
}
  

//TODO: Test throughly
void BezierCurve::init(const std::vector<ramp_msgs::MotionState> sp, const ramp_msgs::MotionState curveStart, const double theta, const double x_dot_0, const double y_dot_0, const double x_dot_dot_0, const double y_dot_dot_0, const double x_dot_max, const double y_dot_max, const double x_dot_dot_max, const double y_dot_dot_max) {
  segment_points_ = sp;
  x_init_v_       = x_dot_0;
  y_init_v_       = y_dot_0;
  x_init_a_       = x_dot_dot_0;
  y_init_a_       = y_dot_dot_0;
  x_dot_max_      = x_dot_max;
  y_dot_max_      = y_dot_max;
  
  initControlPoints(curveStart);
  calculateConstants();

  // If both C and D == 0, the first two points are the same
  if(C_ > 0.0001 && D_ > 0.0001) {
  
    initReflexxes(x_dot_max, y_dot_max, x_dot_dot_max, y_dot_dot_max);

    theta_prev_ = theta;
    
    initialized_ = true;
  } 

  else {
    std::cout<<"\nThe 2 points are the same\n";
    std::cout<<"\nC: "<<C_<<" D: "<<D_;
    std::cout<<"\ncontrol points size: "<<control_points_.size();
    for(int i=0;control_points_.size();i++) {
      std::cout<<"\n"<<i<<": "<<utility_.toString(control_points_.at(i));
    }
    std::cout<<"\nSegment points: ";
    for(int i=0;segment_points_.size();i++) {
      std::cout<<"\n"<<i<<": "<<utility_.toString(segment_points_.at(i))<<"\n";
    }
    std::cout<<"\nLeaving else\n";
  }

}



void BezierCurve::printReflexxesInfo() const {
  
  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->MaxVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];

  std::cout<<"\n\nreflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0]: "<<
    reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0];

  std::cout<<"\nreflexxesData_.inputParameters->MaxAccelerationVector->VecData[0]: "<<
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0];

  std::cout<<"\n\nreflexxesData_.inputParameters->TargetPositionVector->VecData[0]: "<<
    reflexxesData_.inputParameters->TargetPositionVector->VecData[0]<<"\n";

  std::cout<<"\nreflexxesData_.inputParameters->TargetVelocityVector->VecData[0]: "<<
    reflexxesData_.inputParameters->TargetVelocityVector->VecData[0]<<"\n";
}





/** Returns true if u_dot_max satisfies the motion constraints for maximum x,y velocity */
const bool BezierCurve::satisfiesConstraints(const double u_dot_max, const double u_x, const double u_y) const {
  if(print_) {
    std::cout<<"\nTesting constraints for "<<u_dot_max;
    std::cout<<"\nx_dot_max: "<<x_dot_max_;
    std::cout<<"\ny_dot_max: "<<y_dot_max_;
    std::cout<<"\n(A_*u_x+C_)*u_dot_max: "<<(A_*u_x+C_)*u_dot_max<<" x_dot_max: "<<x_dot_max_;
    std::cout<<"\n(B_*u_y+D_)*u_dot_max: "<<(B_*u_y+D_)*u_dot_max<<" y_dot_max: "<<y_dot_max_;
  }
  
  // Square them in case they are negative 
  // Add .0001 because I was getting 0.33 > 0.33
  if( pow( (A_*u_x+C_)*u_dot_max,2) > pow((x_dot_max_)+0.0001,2) ||
      pow( (B_*u_y+D_)*u_dot_max,2) > pow((y_dot_max_)+0.0001,2) )
  {
    return false;
  }

  return true;
}




const double BezierCurve::getUDotMax(const double u_dot_0) const {
  
  // Initialize variables
  double u_dot_max;
  double u_x = ( fabs(A_+C_) > fabs(C_) ) ? 1 : 0;
  double u_y = ( fabs(B_+D_) > fabs(D_) ) ? 1 : 0;
  double u_dot_max_y = B_*u_y + D_ == 0 ? 0 : fabs(x_dot_max_ / (B_*u_y+D_));
  double u_dot_max_x = A_*u_x + C_ == 0 ? 0 : fabs(y_dot_max_ / (A_*u_x+C_));


  if(print_) {
    std::cout<<"\nu_x: "<<u_x<<" u_y: "<<u_y;
    std::cout<<"\nu_dot_max_x: "<<u_dot_max_x<<" u_dot_max_y: "<<u_dot_max_y;
  }


  // If both are zero
  if(u_dot_max_x == 0 && u_dot_max_y == 0) {
    ROS_ERROR("u_dot_max_x == 0 && u_dot_max_y == 0");
    u_dot_max = 0;
  }

  // If x is greater
  else if ( u_dot_max_x > u_dot_max_y ) {

    // If x satisfies motion constraints
    if(satisfiesConstraints(u_dot_max_x, u_x, u_y) && u_dot_max_x != 0) {
      u_dot_max = u_dot_max_x;
    }

    // else, test y
    else if(satisfiesConstraints(u_dot_max_y, u_x, u_y) && u_dot_max_y != 0) {
      u_dot_max = u_dot_max_y;
    }

    // else, set it to initial u_dot
    // TODO: Something else?
    else {  
      u_dot_max = u_dot_0;
    }
  } // end if x is greater

  // If y is greater, test it against constraints 
  else if(satisfiesConstraints(u_dot_max_y, u_x, u_y) && u_dot_max_y != 0) {
      u_dot_max = u_dot_max_y;
  }

  // If y is greater and does not satisfy constraints, test x
  else if(satisfiesConstraints(u_dot_max_x, u_x, u_y) && u_dot_max_x != 0) {
    u_dot_max = u_dot_max_x;
  }

  // Else, set u_dot_max to initial u_dot
  // TODO: Something else?
  else {
    u_dot_max = u_dot_0;
  }



  // *** Testing ***
  //u_dot_max = u_dot_max_y;
  //u_dot_max = (x_dot_max2_ - C_) / A_;
  //u_dot_max = (y_dot_max2_ - D_) / B_;
  return u_dot_max;
}



/** This method initializes the necessary Reflexxes variables */
void BezierCurve::initReflexxes(const double x_dot_max, const double y_dot_max, const double x_dot_dot_max, const double y_dot_dot_max) {

  // Initialize Reflexxes variables
  reflexxesData_.rml              = new ReflexxesAPI( 1, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters  = new RMLPositionInputParameters( 1 );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( 1 );

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;



  // Get initial and maximum velocity of Bezier parameter 
  double u_dot_0    = (D_*D_ > C_*C_) ? fabs(y_init_v_ / D_) : fabs(x_init_v_ / C_);
  double u_dot_max  = getUDotMax(u_dot_0);
  if(print_) {
    std::cout<<"\nu_dot_0: "<<u_dot_0<<" u_dot_max: "<<u_dot_max;
  }

  // Set the position and velocity Reflexxes variables
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0]     = 0.;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]     = u_dot_0;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]         = u_dot_max;

  // Set u max acceleration
  // We don't actually use this, but it's necessary for Reflexxes to work
  if(A_ + C_ != 0) {
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = fabs( (y_dot_dot_max - B_*u_dot_max) / (B_+D_) );
  }
  else if (B_ + D_ != 0) {
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = fabs( (x_dot_dot_max - A_*u_dot_max) / (A_+C_) );
  }
  else {
    reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 0.1;
  }

  
  // Set targets
  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = 1;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = reflexxesData_.inputParameters->MaxVelocityVector->VecData[0];
  

  if(print_) {
    printReflexxesInfo();
    std::cin.get();
  }


  reflexxesData_.resultValue = 0;
  //std::cout<<"\nLeaving initReflexxes\n";
} // End initReflexxes


void BezierCurve::initControlPoints() {
  ramp_msgs::MotionState C0, p0, p1, p2;

  // Set segment points
  p0 = segment_points_.at(0);
  p1 = segment_points_.at(1);
  p2 = segment_points_.at(2);

  // Set orientation of the two segments
  double theta_s1 = utility_.findAngleFromAToB(p0.positions, p1.positions);

  /** Positions */
  C0.positions.push_back( (1-lambda_)*p0.positions.at(0) + lambda_*p1.positions.at(0) );
  C0.positions.push_back( (1-lambda_)*p0.positions.at(1) + lambda_*p1.positions.at(1) );
  C0.positions.push_back(theta_s1);

  initControlPoints(C0);

}






/** Initialize the control points of the Bezier curve */
void BezierCurve::initControlPoints(const ramp_msgs::MotionState start) {
  ramp_msgs::MotionState C0, C1, C2, p0, p1, p2;


  // Set segment points
  p0 = segment_points_.at(0);
  p1 = segment_points_.at(1);
  p2 = segment_points_.at(2);

  // Set orientation of the two segments
  double theta_s1 = utility_.findAngleFromAToB( p0.positions, 
                                                p1.positions);
  double theta_s2 = utility_.findAngleFromAToB( p1.positions, 
                                                p2.positions);

  // Control point 0 is passed in
  // Control Point 1 is the 2nd segment point
  C0 = start;
  C1 = segment_points_.at(1);
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
    C0.velocities.push_back(x_init_v_);
    C0.velocities.push_back(y_init_v_);
    C0.velocities.push_back(0);
  }
  /** C0 Accelerations */
  if(C0.accelerations.size() == 0) {
    C0.accelerations.push_back(0);
    C0.accelerations.push_back(0);
    C0.accelerations.push_back(0);
  }



  // Push on all the points
  control_points_.push_back(C0);
  control_points_.push_back(C1);
  control_points_.push_back(C2);
  

  if(print_) {
    std::cout<<"\nControl Points:";
    for(int i=0;i<control_points_.size();i++) {
      std::cout<<"\n"<<utility_.toString(control_points_.at(i));
    }
  }
} // End initControlPoints





/** Returns true when Reflexxes has reached its targets */
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




/** Calculate the minimum radius along the curve */
void BezierCurve::calculateR_min() {

  double numerator_term_one   = ((A_*A_) + (B_*B_)) * (t_min_*t_min_);
  double numerator_term_two   = 2 * ((A_*C_)+(B_*D_)) * t_min_;
  double numerator_term_three = (C_*C_) + (D_*D_);
  double numerator            = pow(numerator_term_one + numerator_term_two + numerator_term_three, 3); 

  double denominator          = pow((B_*C_) - (A_*D_), 2);
 
  R_min_                      = sqrt( numerator / denominator );
}


/** Calculate time when minimum radius occurs along the curve */
void BezierCurve::calculateT_min() {
  t_min_ = -((A_*C_) + (B_*D_)) / ((A_*A_) + (B_*B_));
}


/** Calculate A,B,C,D, minimum radius, and time of minimum radius */
void BezierCurve::calculateConstants() {
  calculateABCD();
  calculateT_min();
  calculateR_min();
  
  if(print_) {
    std::cout<<"\nA: "<<A_<<" B: "<<B_<<" C: "<<C_<<" D: "<<D_<<"\n";
  }
}



/** Generate all the motion states on the curve */
const std::vector<ramp_msgs::MotionState> BezierCurve::generateCurve() {
  if(initialized_) {

    reflexxesData_.resultValue = 0;
   
    points_.push_back(control_points_.at(0));

    while(!finalStateReached()) {
      points_.push_back(spinOnce());
    }

    dealloc();
  }

  return points_;
} // End generateCurve




/** Call Reflexxes once and return the next motion state */
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
  double X0         = control_points_.at(0).positions.at(0);
  double X1         = control_points_.at(1).positions.at(0);
  double X2         = control_points_.at(2).positions.at(0);
  double Y0         = control_points_.at(0).positions.at(1);
  double Y1         = control_points_.at(1).positions.at(1);
  double Y2         = control_points_.at(2).positions.at(1);

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
    std::cout<<"\nu: "<<u<<" u_dot: "<<u_dot<<" u_dot_dot: "<<u_dot_dot;
    std::cout<<"\nx_dot: "<<x_dot<<" y_dot: "<<y_dot;
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
}
