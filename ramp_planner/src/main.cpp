#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
#include "ramp_msgs/ModificationRequest.h"
 


int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");

  ros::NodeHandle handle;
 
  Planner my_planner; 
  my_planner.init_handlers(handle); 
  
  srand( time(NULL));
  Range range0(5.2, 911.7);
  Range range1(0, 180);
  Range range2(30, 150);


  
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  Configuration s;
  Configuration g;
  s.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  s.random();
  g.random();
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();

  my_planner.start_ = s;
  my_planner.goal_ = g;

  std::cout<<"\nPress Enter to initialize the planner\n";
  std::cin.get();
  my_planner.initialization();

  std::cout<<"\nmy_planner.population_.size():"<<my_planner.population_.size();




  std::cout<<"\nPress Enter to modify a path!\n";
  std::cin.get();

  std::cout<<"\nModifying Path:\n"<<my_planner.paths_.at(0).toString();

  ramp_msgs::Path p = my_planner.modify(0);
  std::cout<<"\nPath modified!\n";
  Utility u;
  std::cout<<u.toString(p); 

  std::cout<<"\nSpinning...\n";
  ros::spin();

  std::cout<<"\nExiting Normally\n";
  return 0;
}

