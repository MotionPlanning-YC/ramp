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
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = start_.velocities.at(2);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0]     = start_.velocities.at(2);
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 1;

  reflexxesData_.inputParameters->TargetPositionVector->VecData[0] = 2*PI;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[0] = w_;

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;

  reflexxesData_.resultValue = 0;
 
  //std::cout<<"\nTarget Pos: "<<reflexxesData_.inputParameters->TargetPositionVector->VecData[0];
  //std::cout<<"\nTarget Vel: "<<reflexxesData_.inputParameters->TargetVelocityVector->VecData[0];
  std::cout<<"\nLeaving initReflexxes\n";
}


const ramp_msgs::MotionState Circle::buildMotionState(const ReflexxesData data) {
  ramp_msgs::MotionState result;

  // v = w*r
  double theta = utility_.displaceAngle(start_.positions.at(2), 
      data.outputParameters->NewPositionVector->VecData[0]);
  //x^2 + y^2 = (w*r)^2
  //x = sqrt( (w*r)^2 - y^2 )
  double x = r_*cos(theta);
  double y = r_*sin(theta);
  double x_dot = v_*cos(theta);
  double y_dot = v_*sin(theta);

  std::cout<<"\ntheta: "<<theta<<" (x,y): ("<<x<<","<<y<<")";

  result.positions.push_back(x);
  result.positions.push_back(y);
  result.positions.push_back(theta);

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

  reflexxesData_.resultValue = 0;

  while(!finalStateReached()) {
    /*std::cout<<"\nPosition: "<<reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
    std::cout<<"\nVelocity: "<<reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];
    std::cout<<"\nTarget Pos: "<<reflexxesData_.inputParameters->TargetPositionVector->VecData[0];
    std::cout<<"\nTarget Vel: "<<reflexxesData_.inputParameters->TargetVelocityVector->VecData[0];*/
    result.push_back(spinOnce());
  }

  return result;
}


