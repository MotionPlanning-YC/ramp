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
    s.K_.push_back(0);
    s.K_.push_back(2);
    s.K_.push_back(0);
    
    g.K_.push_back(3);
    g.K_.push_back(2.f);
    g.K_.push_back(0);
  }
  else {
    s.K_.push_back(3.f);
    s.K_.push_back(1.75f);
    s.K_.push_back(PI);
    
    g.K_.push_back(0.f);
    g.K_.push_back(1.75f);
    g.K_.push_back(PI);
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
  my_planner.start_   = s;
  my_planner.goal_    = g;
  std::cout<<"\nStart:"<<s.toString();
  std::cout<<"\nGoal:"<<g.toString();
  
  // Set transform matrix for odometry
  my_planner.setT_od_w(s.K_);


  /** Initialize the Planner's handlers */ 
  my_planner.init(handle); 
  /** End building Planner */


  // Don't start planner, just wait for updates
  //while(ros::ok()) {ros::spinOnce();}
  
  /*Configuration p_origin;
  p_origin.updatePosition(3.f, 0.f, 3.14159f);
  my_planner.setT_od_w(p_origin.K_);

  Configuration p_od;
  p_od.updatePosition(2, 0, 0);
  p_od.add(my_planner.T_od_w_, 3.14159f);
  std::cout<<"\np_w: "<<p_od.toString();*/
  

  
  
  /*ros::Publisher pub = handle.advertise<ramp_msgs::Update>("update", 1000);

  Configuration u;
  u.K_.push_back(1.f);
  u.K_.push_back(2.f);
  u.K_.push_back(-PI/4);
  u.ranges_ = my_planner.ranges_;
  ramp_msgs::Update up;
  up.configuration = u.buildConfigurationMsg();
  std::cout<<"\nAbout to publish!\n";
  pub.publish(up);
  std::cout<<"\nnew config: "<<my_planner.start_.toString();*/

  /******* Start the planner *******/
  std::cout<<"\nPress Enter to start the planner\n";
  std::cin.get(); 
  
  my_planner.go();

  while(ros::ok()) {
    my_planner.evaluatePopulation();
    std::cout<<"\nPopulation size: "<<my_planner.population_.size();
  }

  
  std::cout<<"\nPress Enter to exit!\n";
  std::cin.get();
  ros::spinOnce();
  std::cout<<"\nExiting Normally\n";
  return 0;
}

