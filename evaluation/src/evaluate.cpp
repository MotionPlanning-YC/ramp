#include "evaluate.h"

Evaluate::Evaluate() : Q(10000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(10000.f) {
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;
  
  /*Set the i_segments_ member*/
  // First, clear
  i_segments_.clear();

  //Set the elements
  for(unsigned int i=0;i<req.i_segments.size();i++) {
    i_segments_.push_back(req.i_segments.at(i));
  } 
} //End setRequest


/** This method computes the fitness of the trajectory_ member */
//TODO: Automate the weights for each evaluation criteria
const double Evaluate::performFitness(CollisionDetection::QueryResult feasible) {
  double result=0;

  //Create the path to evaluate
  //We do this because we may only be evaluating parts of a path
  ramp_msgs::Trajectory t_eval;

  //For now, we evaluate the whole trajectory
  //TODO: Be able to specify which segments are evaluated
  t_eval = trajectory_;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  time_.trajectory_ = t_eval;
  result+=(time_.perform()) * -1;

  
  // If the trajectory is infeasible
  if(feasible.collision_) {

    // Add the Penalty for being infeasible
    if (feasible.time_until_collision_ == 0) 
      result += (Q*Q)*-1;
    else    
      result += (Q / feasible.time_until_collision_) * -1;
  }
  
  return result;
} //End performFitness
