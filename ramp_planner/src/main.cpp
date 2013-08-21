#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
#include "ramp_msgs/ModificationRequest.h"
 


int main(int argc, char** argv) {
  ros::init(argc, argv, "planner");

  ros::NodeHandle handle;
  
  Utility u;
 
  srand( time(NULL));
  Range range0(0, 10);
  Range range1(0, 10);
  Range range2(30, 90);
 
 
  /** Build the Planner */ 

  Planner my_planner; 
  my_planner.init_handlers(handle); 
  
  //Set ranges
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  //Make Configurations
  Configuration s;
  Configuration m;
  Configuration g;
  s.ranges_ = my_planner.ranges_;
  m.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  s.random();
  g.random();

  //Set start and goal
  my_planner.start_ = s;
  my_planner.goal_ = g;

  /** End building Planner */



  
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();

  std::cout<<"\nPress Enter to initialize the planner\n";
  std::cin.get();
  my_planner.init_population();


  //std::cout<<"\npopulation size:"<<my_planner.population_.population_.size();
  //Print all the initial trajectories
  /*for(unsigned int i=0;i<my_planner.population_.size();i++) {
    std::cout<<"\n"<<u.toString(my_planner.population_.at(i)); 
  }*/

  my_planner.modifier_->paths_ = my_planner.paths_;
  my_planner.modifier_->velocities_ = my_planner.velocities_;


  for(unsigned int i=0;i<my_planner.paths_.size();i++) {
    std::cout<<"\n\nPath "<<i<<"\n";
    std::cout<<"\n"<<my_planner.paths_.at(i).toString(); 
  }

  std::cout<<"\nPress Enter to modify a path!\n";
  std::cin.get();

  std::vector<Path> ps = my_planner.modifyPath();

  for(unsigned int i=0;i<ps.size();i++) {
    std::cout<<"\n\nPath "<<i<<"\n";
    std::cout<<"\n"<<ps.at(i).toString(); 
  }

  std::cout<<"\nPress Enter to modify a traj!\n";
  std::cin.get();
  std::vector<RampTrajectory> trs = my_planner.modifyTrajec();
  std::cout<<"\nTrajectory modified!\n";
  std::cout<<"\ntrs.size():"<<trs.size()<<"\n";
  std::cout<<trs.at(0).toString();
  //std::cout<<trs.at(1).toString();
  

  std::cout<<"\nSpinning...\n";
  ros::spin();

  std::cout<<"\nExiting Normally\n";
  return 0;
}

