#include "evaluate.h"



Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) {
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
const double Evaluate::performFitness() {
  double result=0;

  //Create the path to evaluate
  //We do this because we may only be evaluating parts of a path
  ramp_msgs::Trajectory t_eval;

  //For now, we evaluate the whole trajectory
  //TODO: Be able to specify which segments are evaluated
  t_eval = trajectory_;

  //Set values for euclidean distance and add to result
  //Negate because for this criterion, shorter values are better
  euc_dist_.trajectory_ = t_eval; 
  result+=(1.5*euc_dist_.perform()) * -1;

  //Set values for time and add to result
  //Negate because for this criterion, shorter values are better
  time_.trajectory_ = t_eval;
  result+=(time_.perform()) * -1;

  
  return result;
} //End performFitness


const bool Evaluate::performCollisionDetection() {

  if(collision_.obstacleList_.obstacles.size() == 0) 
    return false;

  collision_.obstacleList_ = obstacleList_;
  return collision_.perform();  
}
