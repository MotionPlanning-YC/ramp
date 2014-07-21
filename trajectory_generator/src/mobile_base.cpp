#include "mobile_base.h"





/** Constructor */
MobileBase::MobileBase() {
  
  // Set DOF
  reflexxesData_.NUMBER_OF_DOFS = 3;

  t_control_points_ = 0.5;

  // Creating all relevant objects of the Type II Reflexxes Motion Library
  reflexxesData_.rml = new ReflexxesAPI( reflexxesData_.NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );
  reflexxesData_.inputParameters = new RMLPositionInputParameters( reflexxesData_.NUMBER_OF_DOFS );
  reflexxesData_.outputParameters = new RMLPositionOutputParameters( reflexxesData_.NUMBER_OF_DOFS );
  
  // Use time synchronization so the robot drives in a straight line towards goal 
  reflexxesData_.flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;


  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[2] = PI/4;
  

  // Maximum acceleration
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 0.33;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1] = 0.33;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[2] = PI/4;
  

  // As the maximum jerk values are not known, this is just to try
  reflexxesData_.inputParameters->MaxJerkVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[1] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[2] = PI/3;


  for(uint8_t i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    timeToReachV_max_.push_back( reflexxesData_.inputParameters->MaxVelocityVector->VecData[i] / 
                                 reflexxesData_.inputParameters->MaxAccelerationVector->VecData[i] );
  }
  
  
  // Result
  resultValue = 0;


  // Set the time to cutoff generating points
  timeCutoff_ = ros::Duration(3.5);
} 




/** Destructor */
MobileBase::~MobileBase() {
  delete reflexxesData_.rml;
  delete reflexxesData_.inputParameters;
  delete reflexxesData_.outputParameters;
}





/** This method sets the new target of Reflexxes */
void MobileBase::setTarget(const ramp_msgs::MotionState ms) {
  
  // For each DOF, set the targets for the knot point
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->TargetPositionVector->VecData[i] = ms.positions.at(i);
    if(ms.velocities.size() > 0) {
      reflexxesData_.inputParameters->TargetVelocityVector->VecData[i] = ms.velocities.at(i);
    }
  }  
} // End setTarget







/** This method sets the SelectionVector based on the path p */
void MobileBase::setSelectionVector() {

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[2] = false;
}




void MobileBase::setSelectionVectorRotation() {
  reflexxesData_.inputParameters->SelectionVector->VecData[0] = false;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = false;
  reflexxesData_.inputParameters->SelectionVector->VecData[2] = true;
}


/**
 * Initialize variables just after receiving a service request
 * Set-up the input parameters
 * The first degree of freedom is x position
 * The second degree of freedom is y position
 **/
void MobileBase::setInitialConditions() {
  
  // Initialise the time
  timeFromStart_ = ros::Duration(0);
  

  // Set the position and orientation of the robot as Reflexxes input
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(i);
  }

  // Set the current velocities, in the reference frame, as Reflexxes input
  // This is the latest velocity value got from the path
  if(path_.points.at(0).motionState.velocities.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = path_.points.at(0).motionState.velocities.at(i);
    }
  }
  else {//log some error
  }

  // Set the current accelerations, in the reference frame, as Reflexxes input
  // This is the latest acceleration value got from the path
  if(path_.points.at(0).motionState.accelerations.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = path_.points.at(0).motionState.accelerations.at(i);
    }
  }
  else {//log some error
  }
} // End setInitialConditions





// Compute the orientation needed to reach the target, given an initial position
double MobileBase::computeTargetOrientation(double initial_x, double initial_y, double target_x, double target_y) {
  //We need to recalculate the target orientation
  // For that we first need to create a vector for the current position and the target position
  std::vector<double> current_position;
  current_position.push_back(initial_x);
  current_position.push_back(initial_y);

  std::vector<double> target_position;
  target_position.push_back(target_x);
  target_position.push_back(target_y);

  double angle = utility_.findAngleFromAToB(current_position, target_position);

  return angle;
}





void MobileBase::init(const ramp_msgs::TrajectoryRequest::Request req) {

  // Store the path
  path_ = req.path;
  
  // Set the initial conditions of the reflexxes library
  setInitialConditions();

  // Set SelectionVector
  setSelectionVector();

}

void MobileBase::insertPoint(const ramp_msgs::MotionState ms, ramp_msgs::TrajectoryRequest::Response& res) {
  trajectory_msgs::JointTrajectoryPoint jp = utility_.getTrajectoryPoint(ms);
  jp.time_from_start = timeFromStart_;
  insertPoint(jp, res);
}


void MobileBase::insertPoint(const trajectory_msgs::JointTrajectoryPoint jp, ramp_msgs::TrajectoryRequest::Response& res) {
  res.trajectory.trajectory.points.push_back(jp);

  /** Update Reflexxes */
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);
  
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[0] = jp.positions.at(0);
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[1] = jp.positions.at(1);
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = jp.positions.at(2);
  
  
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0] = jp.velocities.at(0);
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1] = jp.velocities.at(1);
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2] = jp.velocities.at(2);
  
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[0] = jp.accelerations.at(0);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[1] = jp.accelerations.at(1);
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2] = jp.accelerations.at(2);

}



const double MobileBase::findVelocity(const uint8_t i, const double s) const {
  
  double a = reflexxesData_.inputParameters->MaxAccelerationVector->VecData[i];

  double x_dot_init = 0;
  double y_dot_init = 0;

  double radicand = (2*a*s) + pow(x_dot_init, 2);
  double v = sqrt(radicand);
  
  double t = (v - x_dot_init) / a;
  
  std::cout<<"\ni: "<<(int)i;
  std::cout<<"\ns: "<<s<<" a: "<<a<<" x_dot_init: "<<x_dot_init;
  std::cout<<"\nradicand: "<<radicand;
  std::cout<<"\nv: "<<v;
  std::cout<<"\nt: "<<t;
    

  // Check for bounds
  if(v > reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]) {
    v = reflexxesData_.inputParameters->MaxVelocityVector->VecData[i];
  }
  if(v < -reflexxesData_.inputParameters->MaxVelocityVector->VecData[i]) {
    v = -reflexxesData_.inputParameters->MaxVelocityVector->VecData[i];
  }


  return v;
}



const bool MobileBase::lambasOkay(const std::vector<ramp_msgs::MotionState> segment_points, const double lambda_x0, const double lambda_x2) const {
  ramp_msgs::MotionState X0, X1, X2, p0, p1, p2;

  p0 = segment_points.at(0);
  p1 = segment_points.at(1);
  p2 = segment_points.at(2);
  
  X1 = segment_points.at(1);
  
  X0.positions.push_back( (1-lambda_x0)*p0.positions.at(0) + lambda_x0*p1.positions.at(0) );
  X0.positions.push_back( (1-lambda_x0)*p0.positions.at(1) + lambda_x0*p1.positions.at(1) );
  X0.positions.push_back(utility_.findAngleFromAToB(p0.positions, p1.positions));

  X2.positions.push_back( (1-lambda_x2)*p1.positions.at(0) + lambda_x2*p2.positions.at(0) );
  X2.positions.push_back( (1-lambda_x2)*p1.positions.at(1) + lambda_x2*p2.positions.at(1) );
  X2.positions.push_back(utility_.findAngleFromAToB(p1.positions, p2.positions));


  if(X1.positions.at(0) == ( (X0.positions.at(0) + X2.positions.at(0)) / 2. ) &&
      X1.positions.at(1) == ( (X0.positions.at(1) + X2.positions.at(1)) / 2. )) {
    return false;
  }

  return true;
}





const std::vector<double> MobileBase::getControlPointLambas(const std::vector<ramp_msgs::MotionState> segment_points) const {
  std::vector<double> result;

  double lambda_x0 = 0.5;
  double lambda_x2 = lambda_x0;

  std::cout<<"\nlambda_x2: "<<lambda_x2;
  while(!lambasOkay(segment_points, lambda_x0, lambda_x2)) {
    lambda_x2+=0.1; 
    std::cout<<"\nlambda_x2: "<<lambda_x2;

    if(lambda_x2 > 0.9) {
      lambda_x2 = 0.1;
    }
    else {
      std::cout<<"\nIn else";
    }
    
    std::cin.get();
  }

  result.push_back(lambda_x0);
  result.push_back(lambda_x2);

  return result;
}





const std::vector<BezierCurve> MobileBase::bezier(ramp_msgs::Path& p) {
  std::vector<BezierCurve> result;
  double lambda_x0 = 0.5;
  double lambda_x2 = 0.5;

  ramp_msgs::Path p_copy = p;

  std::cout<<"\np.points.size(): "<<p.points.size()<<"\n";
  for(uint8_t i=1;i<p_copy.points.size()-1;i++) {
    std::cout<<"\n---i: "<<(int)i<<"---\n";
    BezierCurve bc;

    std::vector<ramp_msgs::MotionState> segment_points;
    segment_points.push_back(p_copy.points.at(i-1).motionState);
    segment_points.push_back(p_copy.points.at(i).motionState);
    segment_points.push_back(p_copy.points.at(i+1).motionState);

    std::cout<<"\nSegment points: ";
    for(uint8_t a=0;a<segment_points.size();a++) {
      std::cout<<"\n "<<a<<"\n"<<utility_.toString(segment_points.at(a));
    }


    // Find the slope
    double ryse = segment_points.at(1).positions.at(1) - segment_points.at(0).positions.at(1);
    double run  = segment_points.at(1).positions.at(0) - segment_points.at(0).positions.at(0);

    double slope = (run != 0) ? ryse / run : ryse;
    double a, b;
    uint8_t i_max;
    std::cout<<"\nBefore s";
    std::cout<<"\nsegment_points.at(i-1).positions.size(): "<<segment_points.at(0).positions.size();
    std::cout<<"\nsegment_points.at(i).positions.size(): "<<segment_points.at(1).positions.size()<<"\n";

    // Segment points size != path size
    double s = lambda_x0 * utility_.positionDistance(segment_points.at(0).positions, segment_points.at(1).positions);
    std::cout<<"\nAfter s\n";

    // If the y is greater
    if(slope >= 1) {
      b = findVelocity(1, s);
      a = b / slope;  
      i_max = 1;
    }
    else if(slope < 0 && ryse < 0) {
      a = findVelocity(0, s);
      b = a / slope;
      i_max = 0;
    } 
    else if(slope < 0 && run < 0) {
      b = findVelocity(1, s);
      a = b / slope;
      i_max = 1;
    }
    else {
      a = findVelocity(0, s);
      b = a / slope;  
      i_max = 0;
    }
    
    
    std::cout<<"\nslope: "<<slope;
    std::cout<<"\na: "<<a<<" b: "<<b;

    double theta = utility_.findAngleFromAToB(segment_points.at(0).positions, segment_points.at(1).positions);



    std::vector<double> lambdas = getControlPointLambas(segment_points);
    std::cout<<"\nlambda_x0: "<<lambdas.at(0);
    std::cout<<"\nlambda_x2: "<<lambdas.at(1)<<"\n";
    


    bc.init(segment_points, lambdas.at(0), lambdas.at(1), theta, a, b,
        reflexxesData_.inputParameters->MaxVelocityVector->VecData[0],
        reflexxesData_.inputParameters->MaxVelocityVector->VecData[1],
        reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0],
        reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1]);
    std::cout<<"\nAfter bc.init\n";
    std::cin.get();
    bc.generateCurve();
    std::cout<<"\nAfter generateCurve\n";
    
    std::cout<<"\nErasing at index: "<<i<<": "<<utility_.toString(p.points.at(i))<<"\n";
    p.points.erase(p.points.begin()+i);
    std::cout<<"\nInserting at index: "<<i<<": "<<utility_.toString(bc.points_.at(0))<<"\n";
    p.points.insert(p.points.begin()+i, utility_.getKnotPoint(bc.points_.at(0)));

    if(i == p_copy.points.size()-2) {
      std::cout<<"\nInserting at index: "<<i<<": "<<utility_.toString(bc.points_.at(bc.points_.size()-1))<<"\n";
      p.points.insert(p.points.begin()+i+1, utility_.getKnotPoint(bc.points_.at(bc.points_.size()-1)));
    }

    result.push_back(bc);
    std::cout<<"\nAfter push_back\n";
  }

  

  //std::cout<<"\nPath after Bezier: "<<utility_.toString(result)<<"\n";
  return result;
} // End bezier





/** Execute one iteration of the Reflexxes control function */
const trajectory_msgs::JointTrajectoryPoint MobileBase::spinOnce() {

  // Calling the Reflexxes OTG algorithm
  resultValue = reflexxesData_.rml->RMLPosition(*reflexxesData_.inputParameters, reflexxesData_.outputParameters, reflexxesData_.flags);


  /** Build the JointTrajectoryPoint object that will be used to build the trajectory */
  trajectory_msgs::JointTrajectoryPoint point = buildTrajectoryPoint(*reflexxesData_.inputParameters, *reflexxesData_.outputParameters);
  
  /*std::cout<<"\nCalled reflexxes with input:";
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[0]: "<<reflexxesData_.inputParameters->CurrentPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentPositionVector->VecData[1]: "<<reflexxesData_.inputParameters->CurrentPositionVector->VecData[1];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[0]: "<<reflexxesData_.inputParameters->CurrentVelocityVector->VecData[0];
  std::cout<<"\nreflexxesData_.inputParameters->CurrentVelocityVector->VecData[1]: "<<reflexxesData_.inputParameters->CurrentVelocityVector->VecData[1];
  std::cout<<"\nOutput: ";
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[0]: "<<reflexxesData_.outputParameters->NewPositionVector->VecData[0];
  std::cout<<"\nreflexxesData_.outputParameters->NewPositionVector->VecData[1]: "<<reflexxesData_.outputParameters->NewPositionVector->VecData[1];*/


  // The input of the next iteration is the output of this one
  *reflexxesData_.inputParameters->CurrentPositionVector = *reflexxesData_.outputParameters->NewPositionVector;
  *reflexxesData_.inputParameters->CurrentVelocityVector = *reflexxesData_.outputParameters->NewVelocityVector;
  *reflexxesData_.inputParameters->CurrentAccelerationVector = *reflexxesData_.outputParameters->NewAccelerationVector;

  return point;
} // End spinOnce







/** This method will return a JointTrajectoryPoint given some output parameters from Reflexxes */
const trajectory_msgs::JointTrajectoryPoint MobileBase::buildTrajectoryPoint(const RMLPositionInputParameters input, const RMLPositionOutputParameters output) {
  trajectory_msgs::JointTrajectoryPoint point;

  
  // Push on the p, v, and a vectors
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    if(reflexxesData_.inputParameters->SelectionVector->VecData[i]) {
      point.positions.push_back(output.NewPositionVector->VecData[i]);
      point.velocities.push_back(output.NewVelocityVector->VecData[i]);
      point.accelerations.push_back(output.NewAccelerationVector->VecData[i]);
    }

    else if(i == 2) {
      
      double p = computeTargetOrientation(input.CurrentPositionVector->VecData[0],
                                          input.CurrentPositionVector->VecData[1],
                                          output.NewPositionVector->VecData[0],
                                          output.NewPositionVector->VecData[1]);
      point.positions.push_back(p);

      double v = (p - input.CurrentPositionVector->VecData[2]) / CYCLE_TIME_IN_SECONDS;
      point.velocities.push_back(v);
      point.accelerations.push_back(0);


      reflexxesData_.outputParameters->NewPositionVector->VecData[2] = p;
      reflexxesData_.outputParameters->NewVelocityVector->VecData[2] = v;
     
    }

    else {
      point.positions.push_back(reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
      point.velocities.push_back(reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i]);
    }
  }


  // The timeFromStart_ is the time of the previous point plus the cycle period
  point.time_from_start = timeFromStart_;
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint








/** 
 *  This method will return a JointTrajectoryPoint given some input parameters from Reflexxes 
 *  This is used to generate the first point on a trajectory
 **/
const trajectory_msgs::JointTrajectoryPoint MobileBase::buildTrajectoryPoint(const RMLPositionInputParameters my_inputParameters) {
  trajectory_msgs::JointTrajectoryPoint point;

  
  // Push on the p, v, and a vectors
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    if(reflexxesData_.inputParameters->SelectionVector->VecData[i]) {
      point.positions.push_back(my_inputParameters.CurrentPositionVector->VecData[i]);
      point.velocities.push_back(my_inputParameters.CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(my_inputParameters.CurrentAccelerationVector->VecData[i]);
    }

    else {
      point.positions.push_back(reflexxesData_.inputParameters->CurrentPositionVector->VecData[i]);
      point.velocities.push_back(reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i]);
      point.accelerations.push_back(reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i]);
    }
  }


  // The timeFromStart_ is the time of the previous point plus the cycle period
  point.time_from_start = timeFromStart_;
  timeFromStart_ += ros::Duration(CYCLE_TIME_IN_SECONDS);


  return point;
} // End buildTrajectoryPoint






// Service callback, the input is a path and the output a trajectory
bool MobileBase::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req, ramp_msgs::TrajectoryRequest::Response& res) {
  //std::cout<<"\nReceived request: "<<utility_.toString(req)<<"\n";
  //std::cin.get();
  
  // Initialize Reflexxes with request
  init(req);

  std::vector<BezierCurve> curves;

  // Use Bezier curves to smooth path
  if(bezier_) {
    std::cout<<"\nPath before Bezier: "<<utility_.toString(path_);
    curves = bezier(path_);
    std::cout<<"\n*******************Path after Bezier: "<<utility_.toString(path_)<<"\n";
  }

  for(int c=0;c<curves.size();c++) {
    std::cout<<"\nCurve "<<c<<": ";
    for(int p=0;p<curves.at(c).points_.size();p++) {
      std::cout<<"\n"<<utility_.toString(curves.at(c).points_.at(p));
    }
  }
  

  // Push 0 onto knot point indices
  res.trajectory.index_knot_points.push_back(0);

  
  // Go through every knotpoint in the path
  // (or until timeCutoff has been reached)
  for (i_kp_ = 1; i_kp_<path_.points.size(); i_kp_++) {
      resultValue = 0;
        
      // Push the initial state onto trajectory
      if(i_kp_ == 1)
        res.trajectory.trajectory.points.push_back(buildTrajectoryPoint(*reflexxesData_.inputParameters));


      // If at a Bezier point
      if(bezier_ && i_kp_ > 1 && i_kp_ < path_.points.size()-1) {

        // Insert all points on the curves into the trajectory
        for(uint32_t p=1;p<curves.at(i_kp_-2).points_.size();p++) {
          insertPoint(curves.at(i_kp_-2).points_.at(p), res);

          // If it's the first or last point on the curve, push the index to knot point vector
          if( p==curves.at(i_kp_-2).points_.size()-1) {
            //std::cout<<"\nChanging orientation "<<res.trajectory.trajectory.points.at( res.trajectory.trajectory.points.size()-1 ).positions.at(2)<<" to ";
            //res.trajectory.trajectory.points.at( res.trajectory.trajectory.points.size()-1 ).positions.at(2) = 
              //  utility_.findAngleFromAToB(res.trajectory.trajectory.points.at( res.trajectory.trajectory.points.size()-2 ),
                //                           res.trajectory.trajectory.points.at( res.trajectory.trajectory.points.size()-1 ));
            //std::cout<<res.trajectory.trajectory.points.at( res.trajectory.trajectory.points.size()-1 ).positions.at(2)<<"\n";
            res.trajectory.index_knot_points.push_back(res.trajectory.trajectory.points.size() - 1);
          } // end if knot point
        } // end for
      } // end if bezier

      // Else if not bezier or 1st/last segment with bezier
      else {

        // if not Bezier, get rotation
        if(!bezier_) {
          double trajec_size = res.trajectory.trajectory.points.size();
          trajectory_msgs::JointTrajectoryPoint last = res.trajectory.trajectory.points.at(trajec_size-1);
          trajectory_msgs::JointTrajectoryPoint next_knot = utility_.getTrajectoryPoint(path_.points.at(i_kp_).motionState);
          std::vector<trajectory_msgs::JointTrajectoryPoint> rotate_points = rotate(last.positions.at(2), utility_.findAngleFromAToB(last, next_knot));
          for(uint8_t p=0;p<rotate_points.size();p++) {
            res.trajectory.trajectory.points.push_back(rotate_points.at(p));
          }

          setSelectionVector();
          resultValue = 0;
        } // end if not bezier, get rotation


       
        // Set target to next knot point
        setTarget(path_.points.at(i_kp_).motionState);
        std::cout<<"\nTarget: "<<utility_.toString(path_.points.at(i_kp_).motionState)<<"\n";

        // We go to the next knotpoint only once we reach this one
        while (!finalStateReached()) {

          trajectory_msgs::JointTrajectoryPoint p = spinOnce();

          // Compute the motion state at t+1 and save it in the trajectory
          res.trajectory.trajectory.points.push_back(p);

        } // end while

        // Once we reached the target, we set that the latest point is a knotpoint
        res.trajectory.index_knot_points.push_back(res.trajectory.trajectory.points.size() - 1);
      } // end else
  } // end for

  // Set the trajectory's resolution rate
  res.trajectory.resolution_rate = CYCLE_TIME_IN_SECONDS;

  //std::cout<<"\nReturning: "<<utility_.toString(res.trajectory)<<"\n";
  //std::cin.get();
  return true;
} // End trajectoryRequest callback



const std::vector<trajectory_msgs::JointTrajectoryPoint> MobileBase::rotate(const double start, const double goal) {
  std::vector<trajectory_msgs::JointTrajectoryPoint> result;

  setSelectionVectorRotation();

  // Set current values
  reflexxesData_.inputParameters->CurrentPositionVector->VecData[2] = start;
  reflexxesData_.inputParameters->CurrentVelocityVector->VecData[2] = 0;
  reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[2] = 0;

  // Set target values
  reflexxesData_.inputParameters->TargetPositionVector->VecData[2] = goal;
  reflexxesData_.inputParameters->TargetVelocityVector->VecData[2] = 0;
  
  resultValue = 0;
  while(!finalStateReached()) {
    trajectory_msgs::JointTrajectoryPoint p = spinOnce();
    result.push_back(p);
  }

  return result;
}




// Returns true if the target has been reached
bool MobileBase::finalStateReached() {
  return (resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
}



