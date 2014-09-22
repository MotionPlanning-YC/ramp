#include "evaluate.h"

Evaluate::Evaluate() : Q(10000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(10000.f) {
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;
  goal_ = req.goal;
  
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


  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  time_.trajectory_ = trajectory_;
  time_.goal_       = goal_;
  std::cout<<"\nTime: "<<time_.perform();
  std::cout<<"\n1.0 / time.perform: "<<(1.0/time_.perform());
  result += (1.0 / time_.perform());
  std::cout<<"\nresult: "<<result;


  // Set values for euclidean distace and add to result
  // Negate because for this criterion, shorter values are better
  //eucDist_.trajectory_  = trajectory_;
  //eucDist_.goal_        = goal_; 
  //std::cout<<"\nEuclid Dist: "<<eucDist_.perform();
  //result += (1.0 / eucDist_.perform());

  
  // If the trajectory is infeasible
  if(feasible.collision_) {

    // Add the Penalty for being infeasible
    if (feasible.t_firstCollision_ == 0) 
      result += (1.0 / (Q*Q));
    else    
      result += (1.0 / (Q / feasible.t_firstCollision_));
  }
  
  return result;
} //End performFitness
