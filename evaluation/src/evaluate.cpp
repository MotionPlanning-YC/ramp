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
  
  // Go through all the knot points in the trajectory
  for (int i = 0; i<= trajectory_.index_knot_points.size() -2 ; i++)
  {
    trajectory_msgs::JointTrajectoryPoint p1 = trajectory_.trajectory.points[trajectory_.index_knot_points.at(i)];  
    trajectory_msgs::JointTrajectoryPoint p2 = trajectory_.trajectory.points[trajectory_.index_knot_points.at(i+1) ];  

    //get slope
    float a = (p2.positions.at(0) - p1.positions.at(0)) / (p2.positions.at(1) - p1.positions.at(1));

    float c = p2.positions.at(0) - (a*p2.positions.at(1));

    float x3 = (a*objList_.ir_object.y1) + c;
    float x4 = (a*objList_.ir_object.y2) + c;

    // switch y3 and y4 if they are inverted
    if(x3 < x4) {
      float temp = x3;
      x3 = x4;
      x4 = temp;
    }
    

    std::cout << " a: " << a << " c: " << c << " y1: " << objList_.ir_object.y1 << " y2: " << objList_.ir_object.y2 << " x3: " << x3 << " x4: " << x4 << " x1: " << objList_.ir_object.x1 << " x2: " << objList_.ir_object.x2 << "\n";


    //Check that y1 or y2 are between y3 and y4. If so there is collision
    if( (objList_.ir_object.x1 <= x3 && objList_.ir_object.x1 >= x4) ||
        (objList_.ir_object.x2 <= x3 && objList_.ir_object.x2 >= x4) ) {
          return true; 
      }
  }
  return false;
}
