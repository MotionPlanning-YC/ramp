#include "ros/ros.h"
#include "ramp_msgs/Configuration.h"
#include "planner.h"
#include "range.h"
#include "ramp_msgs/ModificationRequest.h"
#include "ramp_msgs/EvaluationRequest.h"
 



Planner my_planner; 
Utility u;


std::vector<Configuration> getStartGoal(bool robot1) {
  std::cout<<"\nIn getStartGoal";
  std::cout<<"\nrobot1: "<<robot1<<"\n";
  std::vector<Configuration> result;

  Configuration s, g;
  
  if(robot1) {
    s.K_.push_back(1);
    s.K_.push_back(1);
    s.K_.push_back(0);
    
    g.K_.push_back(3);
    g.K_.push_back(1);
    g.K_.push_back(0);
  }
  else {
    s.K_.push_back(1);
    s.K_.push_back(3);
    s.K_.push_back(0);
    
    g.K_.push_back(3);
    g.K_.push_back(3);
    g.K_.push_back(0);
  }

  result.push_back(s);
  result.push_back(g);

  return result;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ramp_planner");
  ros::NodeHandle handle;

  std::string update_topic;
  handle.getParam("ramp_planner/robot_update", update_topic);
  // std::cout<<"\nupdate_topic:"<<update_topic;
  
  //ros::Subscriber sub_update_ = handle.subscribe(update_topic, 1000, &Planner::updateCallback, &my_planner);
  ros::Subscriber sub_update_ = handle.subscribe("update", 1000, &Planner::updateCallback, &my_planner);
  
  // Make some Ranges 
  srand( time(0));
  Range range0(0, 3.5);
  Range range1(0, 3.5);
  Range range2(-1, 1);

  // Set my_planner's ranges
  my_planner.ranges_.push_back(range0);
  my_planner.ranges_.push_back(range1);
  my_planner.ranges_.push_back(range2);
  
  // Make Configurations
  std::vector<Configuration> s_g = getStartGoal(update_topic == "/robot1/update");
  Configuration s = s_g.at(0);
  Configuration g = s_g.at(1);

  s.ranges_ = my_planner.ranges_;
  g.ranges_ = my_planner.ranges_;
  
  // Set start and goal
  my_planner.initial_ = s;
  my_planner.start_   = s;
  my_planner.goal_    = g;

  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();
  

  /** Initialize the Planner's handlers */ 
  my_planner.init(handle); 
  /** End building Planner */

  
  
  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();

  

  std::cout<<"\nExiting Normally\n";
  return 0;
}

