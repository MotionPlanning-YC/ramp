#include "prediction.h"

Prediction::Prediction() {
  reflexxesData_.rml = 0;
  reflexxesData_.inputParameters = 0;
  reflexxesData_.outputParameters = 0;
}

Prediction::~Prediction() {}



/** Initialize Reflexxes variables */
void Prediction::initReflexxes() {

  // Set DOF
  reflexxesData_.NUMBER_OF_DOFS = 3;

  // Initialize all relevant objects of the Type II Reflexxes Motion Library
  if(reflexxesData_.rml == 0) {
    reflexxesData_.rml = new ReflexxesAPI( 
            reflexxesData_.NUMBER_OF_DOFS, CYCLE_TIME_IN_SECONDS );

    reflexxesData_.inputParameters = new RMLPositionInputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );

    reflexxesData_.outputParameters = new RMLPositionOutputParameters( 
            reflexxesData_.NUMBER_OF_DOFS );
  } // end if


  
  // Use time synchronization so the robot drives in a straight line towards goal 
  reflexxesData_.flags.SynchronizationBehavior = RMLPositionFlags::ONLY_TIME_SYNCHRONIZATION;


  // Set up the motion constraints (max velocity, acceleration and jerk)
  // Maximum velocity   
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[0] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[1] = 0.33;
  reflexxesData_.inputParameters->MaxVelocityVector->VecData[2] = PI/4;
  

  // Maximum acceleration
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[0] = 1.;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[1] = 1.;
  reflexxesData_.inputParameters->MaxAccelerationVector->VecData[2] = PI/4;
  

  // As the maximum jerk values are not known, this is just to try
  reflexxesData_.inputParameters->MaxJerkVector->VecData[0] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[1] = 1;
  reflexxesData_.inputParameters->MaxJerkVector->VecData[2] = PI/3;

  // Set flag value to know if Reflexxes has been called yet
  reflexxesData_.outputParameters->NewPositionVector->VecData[0] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[1] = -99;
  reflexxesData_.outputParameters->NewPositionVector->VecData[2] = -99;
  
  // Result
  reflexxesData_.resultValue = 0;
} // End initReflexxes



void Prediction::init(const ramp_msgs::TrajectoryRequest::Request req) {

  path_ = req.path; 

  initReflexxes();
  setInitialMotion();
  setSelectionVector();
  
}



/** This method sets the SelectionVector for x,y trajectories */
void Prediction::setSelectionVector() {

  reflexxesData_.inputParameters->SelectionVector->VecData[0] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[1] = true;
  reflexxesData_.inputParameters->SelectionVector->VecData[2] = false;
} // End setSelectionVector




void Prediction::setInitialMotion() {

  // Initialise the time to use for each trajectory point
  timeFromStart_ = ros::Duration(0);
  
  
  // Set the positions of the robot as Reflexxes input
  for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
    reflexxesData_.inputParameters->CurrentPositionVector->VecData[i] = path_.points[0].motionState.positions.at(i);
  }

  // Set the current velocities of the robot as Reflexxes input
  if(path_.points.at(0).motionState.velocities.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentVelocityVector->VecData[i] = path_.points.at(0).motionState.velocities.at(i);
    }
  }
  else {//log some error
  }

  // Set the current accelerations of the robot as Reflexxes input
  if(path_.points.at(0).motionState.accelerations.size() > 0) {
    for(unsigned int i=0;i<reflexxesData_.NUMBER_OF_DOFS;i++) {
      reflexxesData_.inputParameters->CurrentAccelerationVector->VecData[i] = path_.points.at(0).motionState.accelerations.at(i);
    }
  }
  else {//log some error
  }

}



const trajectory_msgs::JointTrajectoryPoint Prediction::spinOnce() {
}


bool Prediction::trajectoryRequest(ramp_msgs::TrajectoryRequest::Request& req, ramp_msgs::TrajectoryRequest::Response& res) {
  res.trajectory.i_knotPoints.push_back(0);

  std::vector<ramp_msgs::MotionState> traj;
  if(req.path.points.size() > 1) {
    if(fabs(req.path.points.at(0).motionState.velocities.at(2)) < 0.0001) {
      std::cout<<"\nIn straight line prediction\n";
      Line li;
      li.init(req.path.points.at(0).motionState, req.path.points.at(1).motionState);
      traj = li.generatePoints(); 
    } 
  }
    else if(fabs(req.path.points.at(0).motionState.velocities.at(2)) > 0.0001 ) {
      std::cout<<"\nIn circle prediction\n";
      std::cout<<"\nPoints: "<<utility_.toString(req.path.points.at(0).motionState)<<"\n";
      Circle ci;
      ci.init(req.path.points.at(0).motionState);
      traj = ci.generatePoints(); 
    }
    else {
      traj.push_back(req.path.points.at(0).motionState);
    }

    ramp_msgs::RampTrajectory rt;
    for(int i=0;i<traj.size();i++) {
      rt.trajectory.points.push_back(utility_.getTrajectoryPoint(traj.at(i)));
    }
    res.trajectory = rt;
    res.trajectory.i_knotPoints.push_back(0);
    res.trajectory.i_knotPoints.push_back(rt.trajectory.points.size()-1);

    for(int i=0;i<res.trajectory.i_knotPoints.size();i++) {
      std::cout<<"\nKnot Point ID: "<<res.trajectory.i_knotPoints.at(i);
    }
  


  return true;
}



// Returns true if the target has been reached
bool Prediction::finalStateReached() {
  return (reflexxesData_.resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED);
  //return (resultValue_ == ReflexxesAPI::RML_FINAL_STATE_REACHED);
  //return ((resultValue == ReflexxesAPI::RML_FINAL_STATE_REACHED) || (timeFromStart_ >= timeCutoff_));
} // End finalStateReached
