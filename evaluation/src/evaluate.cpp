#include "evaluate.h"


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;

  for(unsigned int i=0;i<req.i_segments.size();i++) {
    i_segments_.push_back(req.i_segments.at(i));
  }
}


const double Evaluate::perform() {
  double result=0;

  //Create the path to evaluate
  // We do this because we may only be evaluating parts of a path
  ramp_msgs::Trajectory t_eval;
  t_eval = trajectory_;

  //Set values for euclidean distance and add to result
  euc_dist_.trajectory_ = t_eval;
  std::cout<<"\neuclidean distance value: "<<euc_dist_.perform();
  result+=euc_dist_.perform();

  //Set values for time and add to result
  time_.trajectory_ = t_eval;
  std::cout<<"\ntime value: "<<time_.perform()<<"\n";
  result+=(1.5*time_.perform());

  //Negate the result so the higher values are worse
  result *= -1;

  return result;
}
