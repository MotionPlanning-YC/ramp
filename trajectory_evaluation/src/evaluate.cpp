#include "evaluate.h"

Evaluate::Evaluate() : Q(1000.f), orientation_infeasible_(0) {}


Evaluate::Evaluate(const ramp_msgs::EvaluationRequest& req) : Q(1000.) 
{
  setRequest(req);
}

/** This method accepts an EvaluationRequest and sets the appropriate members for evaluation */
void Evaluate::setRequest(const ramp_msgs::EvaluationRequest& req) 
{
  ros::Time t_start = ros::Time::now();

  //trajectory_ = req.trajectory;
  //ob_trjs_ = req.obstacle_trjs;
  //cd_.obstacle_trjs_ = req.obstacle_trjs;
  //cd_.trajectory_ = req.trajectory;
  //orientation_.trajectory_ = trajectory_;
  //orientation_.currentTheta_ = req.currentTheta;
  //orientation_.theta_at_cc_ = req.theta_cc;
  
  ROS_INFO("setRequest time: %f", (ros::Time::now() - t_start).toSec());
} //End setRequest


const void Evaluate::perform(ramp_msgs::EvaluationRequest& req, ramp_msgs::EvaluationResponse& res)
{
  ros::Time t_start = ros::Time::now();
  res.feasible = performFeasibility(req);

  if(qr_.collision_)
  {
    //ROS_INFO("Not feasible");
    res.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
  }
  else
  {
    //ROS_INFO("Feasible");
    res.t_firstCollision = ros::Duration(9999.f);
  }

  res.fitness = performFitness(req.trajectory);

  //ROS_INFO("t_perform: %f", (ros::Time::now() - t_start).toSec());
}


// Redo this method at some point
// It's modiftying trj AND returning a value
bool Evaluate::performFeasibility(ramp_msgs::EvaluationRequest& er) 
{
  ros::Time t_start = ros::Time::now();
  bool result=true;

  // Check collision
  cd_.perform(er.trajectory, er.obstacle_trjs, qr_);
  er.trajectory.feasible = !qr_.collision_;

  ros::Time t_after = ros::Time::now();
  ramp_msgs::RampTrajectory trj = er.trajectory;
  bool moving = (fabs( sqrt( pow(trj.trajectory.points.at(0).velocities.at(0), 2) +
                            pow(trj.trajectory.points.at(0).velocities.at(1), 2))) > 0)
                || (fabs(trj.trajectory.points.at(0).velocities.at(2)) > 0) ?
                true : false;

  bool moving_on_curve = 
    er.trajectory.curves.size() > 0 && 
    (er.trajectory.curves.at(0).u_0 > 0.000001 ||
    (utility_.positionDistance(trj.trajectory.points.at(0).positions, 
       er.trajectory.curves.at(0).controlPoints.at(0).positions) < 0.0001) ) ?
    true
    :
    false;


  // Check orientation
  if(moving && fabs(orientation_.getDeltaTheta(er.trajectory)) > 0.25 && !moving_on_curve)
  {

    /*if(er.trajectory.i_knotPoints.size() == 2 && er.trajectory.curves.size() < 1)
    {
      er.trajectory.feasible = false;
    }*/

    //ROS_INFO("In if");
    if(er.trajectory.i_knotPoints.size() > 2 && er.trajectory.curves.size() < 2)
    {
      //ROS_INFO("In inner if, i_knotPoints.size(): %i curves.size(): %i", (int)er.trajectory.i_knotPoints.size(), 
          //(int)er.trajectory.curves.size());
      er.trajectory.feasible = false;
      orientation_infeasible_ = true;
    }
    else if(er.trajectory.i_knotPoints.size() == 2 && er.trajectory.curves.size() < 1)
    {
      //ROS_INFO("In inner else if, i_knotPoints.size(): %i curves.size(): %i", (int)er.trajectory.i_knotPoints.size(), 
          //(int)er.trajectory.curves.size());
      er.trajectory.feasible = false;
      orientation_infeasible_ = true;
    }
  }

  // If not feasible, set t_firstCollision
  if(!er.trajectory.feasible)
  {
    //ROS_INFO("traj.t_firstCollision: %f", er.trajectory.t_firstCollision.toSec());
    //ROS_INFO("qr_.t_firstCollision: %f", qr_.t_firstCollision_);
    er.trajectory.t_firstCollision = ros::Duration(qr_.t_firstCollision_);
    //ROS_INFO("traj.t_firstCollision: %f", er.trajectory.t_firstCollision.toSec());
  }


  //result = er.trajectory.feasible;

  //ROS_INFO("t_after: %f", (ros::Time::now() - t_after).toSec());
  //ROS_INFO("performFeasibility time: %f", (ros::Time::now() - t_start).toSec());
  return er.trajectory.feasible;
}



/** This method computes the fitness of the trajectory_ member */
const double Evaluate::performFitness(ramp_msgs::RampTrajectory& trj) 
{
  ros::Time t_start = ros::Time::now();
  //ROS_INFO("In Evaluate::performFitness");
  double result=0;
  double cost=0;
  double penalties = 0;

  // Set values for time and add to result
  // Negate because for this criterion, shorter values are better
  
  if(trj.feasible)
  {
    //ROS_INFO("In if(feasible)");
    double T = trj.trajectory.points.at(trj.trajectory.points.size()-1).time_from_start.toSec();
    double A = 0;//orientation_.perform();
    //ROS_INFO("T: %f A: %f", T, A);
    cost = T + A;
  }


  else
  {
    //penalties += orientation_.getPenalty();
    
    //ROS_INFO("In else(infeasible)"); 
    // Add the Penalty for being infeasible, at some point i was limiting the time to 10s, but i don't know why
    if(trj.t_firstCollision.toSec() > 0)
    {
      //ROS_INFO("In if t_firstCollision: %f", trj.t_firstCollision.toSec());
      penalties += (Q / trj.t_firstCollision.toSec());
    }
    /*else
    {
      penalties += Q;
    }*/

    if(orientation_infeasible_)
    {
      //ROS_INFO("In if orientation_infeasible_: %f", orientation_.getDeltaTheta());
      penalties += Q*orientation_.getDeltaTheta(trj);
    }
  }

  //ROS_INFO("cost: %f penalties: %f", cost, penalties);
  result = (1. / (cost + penalties));

  //ROS_INFO("performFitness time: %f", (ros::Time::now() - t_start).toSec());
  return result;
} //End performFitness
