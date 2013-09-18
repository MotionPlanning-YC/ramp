#include "evaluate.h"



Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;

  for(unsigned int i=0;i<req.i_segments.size();i++) {
    i_segments_.push_back(req.i_segments.at(i));
  }
}


const double Evaluate::perform(obstacle_struct obstacle) {
  double result=0;

  //Create the path to evaluate
  // We do this because we may only be evaluating parts of a path
  ramp_msgs::Trajectory t_eval;
  t_eval = trajectory_;

  //Set values for euclidean distance and add to result
  euc_dist_.trajectory_ = t_eval;
  
  //Modified: For more efficiency, and to be fast here, I just added the cobstacle check in the euc_dist.
  // This is not the best implementation, but is the fastest to code and to execute
  result+=(1.5*euc_dist_.perform(obstacle));

  //Set values for time and add to result
  time_.trajectory_ = t_eval;
  result+=(time_.perform());

  //Negate the result so the higher values are worse
  result *= -1;

  return result;
}
