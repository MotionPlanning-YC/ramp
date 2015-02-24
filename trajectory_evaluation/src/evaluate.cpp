#include "evaluate.h"

Evaluate::Evaluate() : Q(1000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(1000.) {
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request req) {
  trajectory_ = req.trajectory;
  currentTheta_ = req.currentTheta;
} //End setRequest





/** This method computes the fitness of the trajectory_ member */
const double Evaluate::performFitness(CollisionDetection::QueryResult feasible) {
  double result=0;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  time_.trajectory_ = trajectory_;
  double T = time_.perform();
  

  orientation_.trajectory_ = trajectory_;
  orientation_.currentTheta_ = currentTheta_;
  double A = orientation_.perform();

  ROS_INFO("T: %f A: %f", T, A);

  double cost = T + A;


  double penalties = 0;

  penalties += orientation_.getPenalty();
  
  // If the trajectory is infeasible
  if(feasible.collision_) {

    // Add the Penalty for being infeasible
    if (feasible.t_firstCollision_ == 0) 
      penalties += Q;
    else    
      penalties += (Q / feasible.t_firstCollision_);
  }

  ROS_INFO("cost: %f penalties: %f", cost, penalties);
  
  
  result = (1. / (cost + penalties));

  return result;
} //End performFitness
