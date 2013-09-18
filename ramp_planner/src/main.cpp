#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
 


Planner my_planner; 



int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");

  ros::NodeHandle handle;
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);
  
  Utility u;
 
  srand( time(0));
  Range range0(0, 4);
  Range range1(0, 1);
  Range range2(0, 0);
 
 
  /** Initialize the Planner's handlers */ 
  my_planner.init_handlers(handle); 
  
  //Set ranges
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  //Make Configurations
  Configuration s;
  Configuration g;
  s.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  s.K_.push_back(0);
  s.K_.push_back(0);
  s.K_.push_back(0);
  
  g.K_.push_back(4);
  g.K_.push_back(1);
  g.K_.push_back(0);
  //s.random();
  //g.random();

  //Set start and goal
  my_planner.start_ = s;
  my_planner.goal_ = g;

  /** End building Planner */

  
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();

  std::cout<<"\nPress Enter to start the planner!\n";
  std::cin.get();
  

  my_planner.go();


  //std::vector<trajectory_msgs::JointTrajectoryPoint> points = my_planner.population_.population_.at(0).msg_trajec_.trajectory.points;
  
  //Create configuration
  /*Configuration c;
  for(unsigned int i=0;i<points.at(0).positions.size();i++) {
    c.K_.push_back(points.at(10).positions.at(i));
  }
  c.ranges_ = my_planner.paths_.at(0).all_.at(0).ranges_;
  


  //std::cout<<"\nSizes of all the paths before update: ";
  /*for(unsigned int i=0;i<my_planner.population_.population_.size();i++) {
    std::cout<<"\n"<<i<<":"<<my_planner.population_.population_.at(i).msg_trajec_.trajectory.points.size();
  }*/
  /*std::cout<<"\nPaths before update: ";
  for(unsigned int i=0;i<my_planner.paths_.size();i++) {
    std::cout<<"\n"<<i<<":"<<my_planner.paths_.at(i).toString();
  }

  ros::Duration d = points.at(points.size()/2).time_from_start+ros::Duration(0.1);
  d.sleep();
  my_planner.start_ = c;
  my_planner.updatePopulation(d);

  std::cout<<"\nPaths after update: ";
  for(unsigned int i=0;i<my_planner.paths_.size();i++) {
    std::cout<<"\n"<<i<<":"<<my_planner.paths_.at(i).toString();
  }
  std::cout<<"\n";
  std::cin.get();

  std::cout<<"\nPopulation after update: ";
  std::cout<<"\n"<<my_planner.population_.toString();
  std::cin.get();
  //std::cout<<"\nSizes of all the paths after update: ";
  for(unsigned int i=0;i<my_planner.population_.population_.size();i++) {
    std::cout<<"\n"<<i<<":"<<my_planner.population_.population_.at(i).msg_trajec_.trajectory.points.size();
  }*/
  

  //std::cout<<"\npopulation size:"<<my_planner.population_.population_.size();
  //Print all the initial trajectories
  /*for(unsigned int i=0;i<my_planner.population_.population_.size();i++) {
    std::cout<<"\n"<<my_planner.population_.population_.at(i).toString(); 
    std::cout<<"\n"<<my_planner.paths_.at(i).toString();
    std::cin.get();
  }*/

  


  /*std::vector<unsigned int> i_segments;
  ramp_msgs::EvaluationRequest er = my_planner.buildEvaluationRequest(0, i_segments);
  if(my_planner.requestEvaluation(er)) {
    std::cout<<"\nResponse: fitness:"<<er.response.fitness<<" feasible:"<< ((er.response.feasible) ? "true" : "false");
    std::cin.get();
  }

  RampTrajectory bt = my_planner.evaluateAndObtainBest();
  std::cout<<"\n"<<my_planner.population_.toString();
  std::cout<<"\nPress enter to view best trajectory\n";
  std::cin.get();
  std::cout<<"\n"<<bt.toString();

  my_planner.modifier_->paths_ = my_planner.paths_;
  my_planner.modifier_->velocities_ = my_planner.velocities_;


  for(unsigned int i=0;i<my_planner.paths_.size();i++) {
    std::cout<<"\n\nPath "<<i;
    std::cout<<"\n"<<my_planner.paths_.at(i).toString(); 
  }

  std::cout<<"\nPress Enter to modify a path!\n";
  std::cin.get();

  std::vector<Path> ps = my_planner.modifyPath();

  for(unsigned int i=0;i<ps.size();i++) {
    std::cout<<"\n\nModified Path "<<i;
    std::cout<<"\n"<<ps.at(i).toString(); 
  }

  std::cout<<"\nPress Enter to modify a traj!\n";
  std::cin.get();
  std::vector<RampTrajectory> trs = my_planner.modifyTrajec();
  std::cout<<"\nTrajectory modified!\n";
  for(unsigned int i=0;i<trs.size();i++) {
    std::cout<<"\n\nModified Trajectory "<<i;
    std::cout<<trs.at(i).toString();
  }*/
 
 
  //Test sending the best trajectory 
  /*std::cout<<"\nPress enter to send the best trajectory!\n";
  std::cin.get();
  my_planner.bestTrajec_ = my_planner.population_.findBest();
  my_planner.sendBest();*/

  

  std::cout<<"\nExiting Normally\n";
  return 0;
}

