#include "circle.h"

Circle::Circle() {
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters = 0;
  reflexxesData_.outputParameters = 0;
  
}

Circle::~Circle() {
  if(reflexxesData_.rml != 0) {
    delete reflexxesData_.rml;
    reflexxesData_.rml = 0;
  }
  if(reflexxesData_.inputParameters) {
    delete reflexxesData_.inputParameters;
    reflexxesData_.inputParameters = 0;
  }
  if(reflexxesData_.outputParameters != 0) {
    delete reflexxesData_.outputParameters;
    reflexxesData_.outputParameters = 0;
  }
}


const bool Circle::finalStateReached() {
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}

void Circle::init(const ramp_msgs::MotionState s) {
  std::cout<<"\nIn init\n";
  start_ = s;
  std::cout<<"\nstart: "<<utility_.toString(start_)<<"\n";
  
  //v=wr
  w_ = s.velocities.at(2);
  v_ = sqrt( pow(s.velocities.at(0),2) + pow(s.velocities.at(1),2) );
  r_ = v_ / w_;
  std::cout<<"\nw: "<<w_<<" v: "<<v_<<" r: "<<r_;


  std::vector<double> zero;
  zero.push_back(0); zero.push_back(0);
  double theta = utility_.findAngleFromAToB(zero, s.positions);
  std::cout<<"\ntheta: "<<theta;

  // Center issue Might have something to do with t
  double t = utility_.displaceAngle(theta, -start_.positions.at(2));
  std::cout<<"\nt: "<<t;
  
  double angle = utility_.displaceAngle(PI/2, -start_.positions.at(2));
  std::cout<<"\nangle: "<<angle;

   

  center_.positions.push_back(s.positions.at(0) - r_*cos(angle));
  center_.positions.push_back(s.positions.at(1) + r_*sin(angle));
  std::cout<<"\nr_*cos(angle): "<<r_*cos(angle);
  std::cout<<"\nr_*sin(angle): "<<r_*sin(angle);
  std::cout<<"\ncenter: ("<<center_.positions.at(0)<<", "<<center_.positions.at(1)<<")";

  double startingTheta = utility_.findAngleFromAToB(center_.positions, start_.positions);
  std::cout<<"\nstartingTheta: "<<startingTheta;

  initReflexxes();
  std::cout<<"\nLeaving init\n";
}



void Circle::initReflexxes() {
  std::cout<<"\nIn initReflexxes\n";

  // Initialize Reflexxes variables
  reflexxesData_.rml              = new ReflexxesAPI( 1, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters  = new RMLPositionInputParameters( 1 );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( 1 );

  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = 0;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = fabs(w_); //start_.velocities.at(2);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]     = fabs(w_); //start_.velocities.at(2);
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 1;

  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = 2*PI;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = fabs(w_);

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;

  reflexxesData_.resultValue = 0;
 
  std::cout<<"\nTarget Pos: "<<reflexxesData_.inputParameters->TargetPositionVector->VecData[0];
  std::cout<<"\nTarget Vel: "<<reflexxesData_.inputParameters->TargetVelocityVector->VecData[0];
  std::cout<<"\nLeaving initReflexxes\n";
}


const ramp_msgs::MotionState Circle::buildMotionState(const ReflexxesData data) {
  ramp_msgs::MotionState result;

  std::vector<double> zero;
  zero.push_back(0); zero.push_back(0);
  double startingTheta = utility_.findAngleFromAToB(center_.positions, start_.positions);
  double s_theta = utility_.findAngleFromAToB(zero, start_.positions);

  double theta;
  if(w_ > 0) {
    theta = utility_.displaceAngle(startingTheta, 
        data.outputParameters->NewPositionVector->VecData[0]);
  }
  else {
    //std::cout<<"\ndata.outputParameters->NewPositionVector->VecData[0]: "<<data.outputParameters->NewPositionVector->VecData[0];
    theta = utility_.displaceAngle(startingTheta, 
        -data.outputParameters->NewPositionVector->VecData[0]);
  }
  //double theta = utility_.displaceAngle(start_.positions.at(2), 
    //data.outputParameters->NewPositionVector->VecData[0]);
    //
  

  //x^2 + y^2 = (w*r)^2
  //x = sqrt( (w*r)^2 - y^2 )
  double x = r_*cos(theta);
  double y = r_*sin(theta);
  result.positions.push_back(x+center_.positions.at(0));
  result.positions.push_back(y+center_.positions.at(1));
  result.positions.push_back(theta);

  
  
  double phi = data.outputParameters->NewPositionVector->VecData[0];
  double t = utility_.findAngleFromAToB(zero, result.positions);
  
  double x_dot = v_*cos(phi)*cos(t);
  double y_dot = v_*cos(phi)*sin(t);

  /*std::cout<<"\ntheta: "<<theta<<" (x,y): ("<<x<<","<<y<<")";
  std::cout<<"\nx+center_.positions.at(0): "<<x+center_.positions.at(0);
  std::cout<<"\ny+center_.positions.at(1): "<<y+center_.positions.at(1);*/

  result.velocities.push_back(x_dot);
  result.velocities.push_back(y_dot);
  result.velocities.push_back(data.inputParameters->CurrentVelocityVector->VecData[0]);

  result.accelerations.push_back(0);
  result.accelerations.push_back(0);
  result.accelerations.push_back(0);

  result.time = timeFromStart_.toSec();
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  return result;
}


// v = w*r
// sqrt(x^2 + y^2) = theta_dot * r
const ramp_msgs::MotionState Circle::spinOnce() {
  // Calling the Reflexxes OTG algorithm
  reflexxesData_.resultValue = 
    reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, 
                                    reflexxesData_.outputParameters, 
                                    reflexxesData_.flags);

  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  ramp_msgs::MotionState result = buildMotionState(reflexxesData_);
  //std::cout<<"\nresult: "<<utility_.toString(result);


  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = 
      *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = 
      *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = 
      *reflexxesData_.outputParameters->NewAccelerationVector;

  return result;
}

const std::vector<ramp_msgs::MotionState> Circle::generatePoints() {
  std::cout<<"\nIn generatePoints\n";
  std::vector<ramp_msgs::MotionState> result;


  result.push_back(start_);
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);

  reflexxesData_.resultValue = 0;

  while(!finalStateReached()) {
    result.push_back(spinOnce());
  }

  return result;
}


