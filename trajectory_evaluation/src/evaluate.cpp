#include "evaluate.h"

Evaluate::Evaluate() : Q(1000.f) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest::Request& req) : Q(1000.) 
{
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest::Request req) 
{
  trajectory_ = req.trajectory;
  currentTheta_ = req.currentTheta;
  cd_.obstacle_trjs_ = req.obstacle_trjs;
  cd_.trajectory_ = req.trajectory;
  orientation_.trajectory_ = trajectory_;
  orientation_.currentTheta_ = currentTheta_;
} //End setRequest


bool Evaluate::performFeasibility() 
{
  bool result=true;

  // Check collision
  CollisionDetection::QueryResult qr = cd_.perform();
  trajectory_.feasible = !qr.collision_;

  // Check orientation
  //if(orientation_.getDeltaTheta() > (PI/3))
  //{
  if(fabs(orientation_.getDeltaTheta()) > 0.25)
  {
    if(trajectory_.i_knotPoints.size() > 2 && trajectory_.curves.size() < 2)
    {
      trajectory_.feasible = false;
    }
    else if(trajectory_.i_knotPoints.size() == 2 && trajectory_.curves.size() < 1)
    {
      trajectory_.feasible = false;
    }
  }
    //trajectory_.feasible = false;
  //}
  
  // If not feasible, set t_firstCollision
  if(!trajectory_.feasible)
  {
    trajectory_.t_firstCollision = ros::Duration(qr.t_firstCollision_);
  }


  result = trajectory_.feasible;
  return result;
}



/** This method computes the fitness of the trajectory_ member */
const double Evaluate::performFitness(bool feasible) 
{
  double result=0;
  double cost=0;
  double penalties = 0;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  
  if(feasible)
  {
    double T = trajectory_.trajectory.points.at(trajectory_.trajectory.points.size()-1).time_from_start.toSec();
    double A = 0;//orientation_.perform();
    cost = T + A;
  }

  //double A = 0.;

  //ROS_INFO("T: %f A: %f", T, A);

  else
  {
    //penalties += orientation_.getPenalty();
    
    // Add the Penalty for being infeasible
    if(trajectory_.t_firstCollision.toSec() > 0 && trajectory_.t_firstCollision.toSec() < 10.0f)
    {
      //penalties += (Q / trajectory_.t_firstCollision.toSec());
    }
    else
    {
      //penalties += Q;
    }
    penalties += Q;
  }

  result = (1. / (cost + penalties));

  return result;
} //End performFitness
