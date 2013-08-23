#include "evaluate.h"


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) {
  path_ = req.path;

  for(unsigned int i=0;i<req.i_segments.size();i++) {
    i_segments_.push_back(req.i_segments.at(i));
  }
}


const double Evaluate::perform() {
  double result=0;

  //Create the path to evaluate
  // We do this because we may only be evaluating parts of a path
  ramp_msgs::Path p_eval;
  p_eval = path_;


  //Set values for euclidean distance
  euc_dist.path_ = p_eval;
  result+=euc_dist.perform();

  return result;
}
