#include "evaluate.h"



/*Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;

  for(unsigned int i=0;i<req.i_segments.size();i++) {
    i_segments_.push_back(req.i_segments.at(i));
  }
}*/

void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request& req) {
  trajectory_ = req.trajectory;

  i_segments_.clear();

  for(unsigned int i=0;i<req.i_segments.size();i++) {
    i_segments_.push_back(req.i_segments.at(i));
  }
}

const double Evaluate::performFitness() {
  double result=0;

  //Create the path to evaluate
  // We do this because we may only be evaluating parts of a path
  ramp_msgs::Trajectory t_eval;
  t_eval = trajectory_;

  //Set values for euclidean distance and add to result
  euc_dist_.trajectory_ = t_eval;
  
  //Modified: For more efficiency, and to be fast here, I just added the cobstacle check in the euc_dist.
  // This is not the best implementation, but is the fastest to code and to execute
  result+=(1.5*euc_dist_.perform());

  //Set values for time and add to result
  time_.trajectory_ = t_eval;
  result+=(time_.perform());

  //Negate the result so the higher values are worse
  result *= -1;

  return result;
}


const bool Evaluate::performCollisionDetection() {
  //Get slope
  trajectory_msgs::JointTrajectoryPoint p1 = trajectory_.trajectory.points[0];  
  trajectory_msgs::JointTrajectoryPoint p2 = trajectory_.trajectory.points[trajectory_.index_knot_points.at(1) ];  

  float a = (p2.positions.at(1) - p1.positions.at(1)) / (p2.positions.at(0) - p1.positions.at(0));

  float c = p2.positions.at(1) - (a*p2.positions.at(0));

  float y3 = (a*objList_.ir_object.x1) + c;
  float y4 = (a*objList_.ir_object.x2) + c;

  if(y3 < y4) {
    float temp = y3;
    y3 = y4;
    y4 = temp;
  }

  //Check that y1 or y2 are between y3 and y4
  if( (objList_.ir_object.y1 <= y3 && objList_.ir_object.y1 >= y4) ||
      (objList_.ir_object.y2 <= y3 && objList_.ir_object.y2 >= y4) ) {
      
      }
}
