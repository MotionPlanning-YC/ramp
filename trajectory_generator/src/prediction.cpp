#include "prediction.h"

Prediction::Prediction() {}

Prediction::~Prediction() {}




void Prediction::init(const ramp_msgs::TrajectoryRequest::Request req) {

  path_ = req.path; 

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

