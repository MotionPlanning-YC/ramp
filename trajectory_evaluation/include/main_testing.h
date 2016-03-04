/* 
 * File:  trajectory_evaluation_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:03 PM
 */

/* 
 * File:  related to trajectory_evaluation_fixtureTest.h
 * Author: Annaliese Andrews, PhD
 * Author: Mahmoud Abdelgawad, PhD Candidate
 * University of Denver
 * Computer Science
 * Created on December 13, 2015, 6:05 PM
 */


#ifndef MAIN_PROCESS_H
#define	MAIN_PROCESS_H

#include <iostream>
#include "ros/ros.h"
#include "evaluate.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Obstacle.h"

Evaluate ev;
CollisionDetection cd;
Utility u;
bool received_ob = false;


/** Srv callback to evaluate a trajectory */
bool handleRequest(ramp_msgs::EvaluationRequest::Request& req,
                   ramp_msgs::EvaluationRequest::Response& res) 
{

  ev.setRequest(req);

  cd.obstacle_trjs_ = req.obstacle_trjs;
  
  // Make a QueryResult object
  CollisionDetection::QueryResult qr;

    cd.trajectory_  = req.trajectory;
    qr = cd.perform();

  // Set response
  res.feasible = !qr.collision_;
  res.t_firstCollision = ros::Duration(qr.t_firstCollision_);

  // Do fitness
  res.fitness = ev.performFitness(qr);

  return true;
} //End handleRequest

#endif	/* MAIN_PROCESS_H */

