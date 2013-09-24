#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
 



Planner my_planner; 
Utility u;

int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");
  ros::NodeHandle handle;
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);
  
  
  //Make some Ranges 
  srand( time(0));
  Range range0(0, 4);
  Range range1(0, 1);
  Range range2(0, 0);

  //Set my_planner's ranges
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

  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();
  
  /** End building Planner */
 

  std::cout<<"\nPress Enter to start the planner!\n";
  std::cin.get();

  /** Initialize the Planner's handlers */ 
  my_planner.init(handle); 
  
  /******* Start the planner *******/
  my_planner.go();

  

  std::cout<<"\nExiting Normally\n";
  return 0;
}

