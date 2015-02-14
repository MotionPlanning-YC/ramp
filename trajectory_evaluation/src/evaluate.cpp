#include "evaluate.h"

Evaluate::Evaluate() : Q(10000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(10000.f) {
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request req) {
  trajectory_ = req.trajectory;
  currentTheta_ = req.currentTheta;
} //End setRequest





/** This method computes the fitness of the trajectory_ member */
const double Evaluate::performFitness(CollisionDetection::QueryResult feasible) {
  double result=0, denom=0;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  time_.trajectory_ = trajectory_;
  denom += time_.perform();



  // Add the change in orientation needed to move on this trajectory
  // Check there is more than 1 points
  if(trajectory_.i_knotPoints.size() > 1) 
  {
    double thetaNec = utility_.findAngleFromAToB(trajectory_.trajectory.points.at(0),
      trajectory_.trajectory.points.at( trajectory_.i_knotPoints.at(1) ));   
    double deltaTheta = utility_.findDistanceBetweenAngles(currentTheta_, thetaNec);
    denom += deltaTheta;
  }
  

  
  // If the trajectory is infeasible
  if(feasible.collision_) {

    // Add the Penalty for being infeasible
    if (feasible.t_firstCollision_ == 0) 
      denom += (Q*Q);
    else    
      denom += (Q / feasible.t_firstCollision_);
  }

  result += (1. / denom);
  
  return result;
} //End performFitness
