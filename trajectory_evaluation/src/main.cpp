#include <iostream>
#include <signal.h>
#include "evaluate.h"
#include "tf/transform_datatypes.h"
#include "ramp_msgs/Obstacle.h"

Evaluate ev;
Utility u;
bool received_ob = false;
std::vector<ros::Duration> t_data;

/** Srv callback to evaluate a trajectory */
bool handleRequest(ramp_msgs::EvaluationSrv::Request& reqs,
                   ramp_msgs::EvaluationSrv::Response& resps) 
{
  ros::Time t_start = ros::Time::now();
  int s = reqs.reqs.size();
  for(uint8_t i=0;i<s;i++)
  {
    
    ramp_msgs::EvaluationResponse res;
    //ROS_INFO("Robot Evaluating trajectory %i: %s", (int)i, u.toString(reqs.reqs[i].trajectory).c_str());
    //ROS_INFO("Obstacle size: %i", (int)reqs.reqs[i].obstacle_trjs.size());

    // If more than one point
    if(reqs.reqs.at(i).trajectory.trajectory.points.size() > 1)
    {
      ev.perform(reqs.reqs[i], res);
    }
    // Else we only have one point (goal point)
    else
    {
      res.fitness = 0.f;
      res.feasible = true;
      res.t_firstCollision = ros::Duration(9999.f);
    }

    //ROS_INFO("Done evaluating, fitness: %f feasible: %s t_firstCollision: %f", res.fitness, res.feasible ? "True" : "False", res.t_firstCollision.toSec());
    ros::Time t_vec = ros::Time::now();
    resps.resps.push_back(res);
  }
  ros::Duration t_elapsed = ros::Time::now() - t_start;
  ROS_INFO("t_elapsed: %f", t_elapsed.toSec());
  t_data.push_back(t_elapsed);
  return true;
} //End handleRequest


void reportData(int sig)
{
  double avg = t_data.at(0).toSec();
  for(int i=1;i<t_data.size();i++)
  {
    avg += t_data.at(i).toSec();
    ROS_INFO("traj_eval: %f", t_data.at(i).toSec());
  }
  avg /= t_data.size();
  ROS_INFO("Average traj_eval duration: %f", avg);
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "trajectory_evaluation");
  ros::NodeHandle handle;

  int id;
 
  ros::ServiceServer service    = handle.advertiseService("trajectory_evaluation", handleRequest);

  signal(SIGINT, reportData);
  //cd.pub_population = handle.advertise<ramp_msgs::Population>("/robot_1/population", 1000);

  /** ***Testing*** */


  /*ros::AsyncSpinner spinner(8);
  std::cout<<"\nWaiting for requests...\n";
  spinner.start();
  ros::waitForShutdown();*/

  ros::spin();

  printf("\nTrajectory Evaluation exiting normally\n");
  return 0;
}
