#include "evaluate.h"

Evaluate::Evaluate() : Q(10000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(10000.f) {
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;
} //End setRequest





/** This method computes the fitness of the trajectory_ member */
//TODO: Automate the weights for each evaluation criteria
const double Evaluate::performFitness(CollisionDetection::QueryResult feasible) {
  double result=0, denom=0;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  time_.trajectory_ = trajectory_;
  denom += time_.perform();

  
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
